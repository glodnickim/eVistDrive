// FW-085 host test for the RUN estimator's crank-angle averaging window.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw085_run_window.js
//
// No host C toolchain here, so this ports the moving average out of
// src/torque_input.c and the blob migration out of src/tuning_config.c, in the
// same style as the other tests in this directory. The ports are duplicated on
// purpose — edit them together with the C.
//
// The point of the card is that this filter has NO time dependence at all: it is
// clocked by the crank, so its window is the same slice of a pedal stroke at
// every cadence. Test 3 asserts that structurally, not just numerically.

'use strict';
const fs = require('fs');
const path = require('path');

const SRC = (name) => fs.readFileSync(path.join(__dirname, '..', 'src', name), 'utf8');
const INC = (name) => fs.readFileSync(path.join(__dirname, '..', 'inc', name), 'utf8');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

// Constants read from the real header, so a change there fails the test rather
// than silently drifting from it.
const header = INC('torque_input.h');
const constOf = (name) => {
    const m = header.match(new RegExp(`#define\\s+${name}\\s+(\\d+)U`));
    if (!m) throw new Error(`${name} not found in torque_input.h`);
    return Number(m[1]);
};
const DEG_MAX = constOf('TORQUE_RUN_WINDOW_DEG_MAX');
const DEG_DEFAULT = constOf('TORQUE_RUN_WINDOW_DEG_DEFAULT');
const DEG_STEP = constOf('TORQUE_RUN_WINDOW_DEG_STEP');
const STEPS_MAX = constOf('TORQUE_RUN_WINDOW_STEPS_MAX');

// --- port of the RUN window from src/torque_input.c ---
class RunWindow {
    constructor() { this.steps = 0; this.buf = []; this.head = 0; this.filled = 0; this.sum = 0; this.value = 0; }
    reset() { this.head = 0; this.filled = 0; this.sum = 0; }
    setWindowDeg(deg) {
        if (deg > DEG_MAX) deg = DEG_MAX;
        let steps = Math.floor((deg * 4) / 15);
        if (steps > STEPS_MAX) steps = STEPS_MAX;
        if (steps !== this.steps) { this.steps = steps; this.reset(); }
    }
    seed(v) {
        this.value = v;
        if (this.steps === 0) { this.reset(); return; }
        this.buf = new Array(this.steps).fill(v);
        this.head = 0; this.filled = this.steps; this.sum = v * this.steps;
    }
    step(sample) {
        if (this.steps === 0) return;
        if (this.filled >= this.steps) this.sum -= this.buf[this.head];
        else this.filled++;
        this.buf[this.head] = sample;
        this.sum += sample;
        this.head++;
        if (this.head >= this.steps) this.head = 0;
        this.value = Math.floor((this.sum + Math.floor(this.filled / 2)) / this.filled);
    }
    // Mirrors torque_input_update(): with the window off, run tracks fast exactly.
    publish(fast) { return this.steps === 0 ? fast : this.value; }
}

console.log(`window: ${DEG_DEFAULT} deg default, ${DEG_MAX} deg max, step ${DEG_STEP} deg, ${STEPS_MAX} steps/rev`);

// 1. Disabled window passes the fast signal straight through (old behaviour at 0).
{
    const w = new RunWindow();
    w.setWindowDeg(0);
    let ok = true;
    for (const v of [0, 500, 1200, 30, 0]) { w.step(v); ok = ok && w.publish(v) === v; }
    check(ok, '1. window 0 = raw passthrough');
}

// 2. Degrees map to whole quadrature steps with no rounding, across the range.
{
    let ok = true;
    for (let deg = 0; deg <= DEG_MAX; deg += DEG_STEP) {
        const w = new RunWindow();
        w.setWindowDeg(deg);
        ok = ok && w.steps === (deg * 4) / 15 && Number.isInteger((deg * 4) / 15);
    }
    const half = new RunWindow(); half.setWindowDeg(180);
    const full = new RunWindow(); full.setWindowDeg(360);
    check(ok, '2. every 15 deg maps to an exact step count');
    check(half.steps === 48, '2. 180 deg = 48 steps (one leg)');
    check(full.steps === 96, '2. 360 deg = 96 steps (one revolution)');
}

// 3. THE card's main criterion: the result depends only on the SEQUENCE of crank
//    steps, never on how fast they arrive. Drive the identical pedal ripple twice,
//    labelled as two very different cadences, and demand identical output.
{
    // Rectified-sine pedal stroke: one hump per leg, i.e. per 48 crank steps.
    const sample = (i) => Math.round(1000 * Math.abs(Math.sin((Math.PI * i) / 48)));
    const run = (deg) => {
        const w = new RunWindow();
        w.setWindowDeg(deg);
        const out = [];
        for (let i = 0; i < 480; i++) { w.step(sample(i)); out.push(w.value); }
        return out;
    };
    const slow = run(180); // as if 50 rpm — 12.5 ms between steps
    const fast = run(180); // as if 110 rpm — 5.7 ms between steps
    check(JSON.stringify(slow) === JSON.stringify(fast),
        '3. output identical regardless of how fast the steps arrive');

    // Structural guard: no time/tick math may creep into the RUN path, or the
    // cadence independence above becomes a lie.
    const src = SRC('torque_input.c');
    const runSection = src.slice(src.indexOf('static void run_window_reset'),
        src.indexOf('void torque_input_run_filter_step') + 900);
    check(!/TICKS_PER_MS|filter_ticks|_ms\b/.test(runSection),
        '3. RUN filter code contains no millisecond or tick arithmetic');

    // A window equal to a whole number of ripple periods CANCELS the ripple
    // (this is why a true average was chosen over an EMA).
    const settled = (a) => a.slice(200);
    const spread = (a) => Math.max(...settled(a)) - Math.min(...settled(a));
    check(spread(run(180)) <= 2, `3. one-leg window cancels the ripple (spread ${spread(run(180))})`);
    check(spread(run(360)) <= 2, `3. full-turn window cancels the ripple (spread ${spread(run(360))})`);
    check(spread(run(90)) > 50, `3. quarter window still shows ripple (spread ${spread(run(90))})`);
    // Mean of |sin| over a period is 2/pi -> ~637 for a 1000 amplitude.
    const mean = settled(run(360))[0];
    check(Math.abs(mean - 637) <= 3, `3. settled value is the true mean (${mean})`);
}

// 4. A stopped crank sends no steps: the estimate holds, it must not decay or zero.
{
    const w = new RunWindow();
    w.setWindowDeg(180);
    for (let i = 0; i < 48; i++) w.step(800);
    const held = w.value;
    let ok = true;
    for (let tick = 0; tick < 10000; tick++) ok = ok && w.publish(0) === held; // control ticks, no crank steps
    check(ok && held === 800, `4. stopped crank freezes the estimate at ${held}`);
}

// 5. Before the first full window, average what exists — not a value dragged down
//    by empty slots.
{
    const w = new RunWindow();
    w.setWindowDeg(360); // 96 steps, deliberately far from full
    w.step(600); check(w.value === 600, '5. one sample -> that sample');
    w.step(800); check(w.value === 700, '5. two samples -> their mean, not sum/96');
    w.step(1000); check(w.value === 800, '5. three samples -> their mean');
}

// 6. Extremes cannot overflow the running sum (96 * 65535 must stay exact).
{
    const w = new RunWindow();
    w.setWindowDeg(360);
    for (let i = 0; i < STEPS_MAX; i++) w.step(65535);
    check(w.sum === STEPS_MAX * 65535 && w.sum <= 0xFFFFFFFF, '6. full window of max samples fits a uint32');
    check(w.value === 65535, '6. average of a saturated window is the saturated value');
}

// 7. seed_run fills the whole window, so a seeded launch starts at full magnitude.
{
    const w = new RunWindow();
    w.setWindowDeg(180);
    w.seed(900);
    check(w.value === 900 && w.filled === 48, '7. seed fills the window');
    w.step(900);
    check(w.value === 900, '7. seeded window does not dip on the next step');
}

// 8. Re-sending the same setting every control tick must not disturb the average.
{
    const w = new RunWindow();
    w.setWindowDeg(180);
    for (let i = 0; i < 48; i++) w.step(700);
    for (let tick = 0; tick < 1000; tick++) w.setWindowDeg(180); // main.c does exactly this
    check(w.filled === 48 && w.value === 700, '8. re-applying the same window keeps the average');
    w.setWindowDeg(360);
    check(w.filled === 0, '8. a real change rebuilds the window');
}

// --- port of the unit migration from src/tuning_config.c ---
// v6 and older carry MILLISECONDS at offset 20; v7 carries crank degrees. The old
// number cannot be converted (its meaning depended on the cadence it was tuned at),
// so only the "off" state survives.
const migrate = (version, stored) => {
    if (version >= 7) return Math.min(stored, DEG_MAX);
    if (version >= 3) return stored === 0 ? 0 : DEG_DEFAULT;
    return DEG_DEFAULT;
};

// 9. Migration rules.
{
    check(migrate(6, 0) === 0, '9. v6 disabled stays disabled');
    check(migrate(6, 300) === DEG_DEFAULT, '9. v6 300 ms -> default window');
    check(migrate(6, 700) === DEG_DEFAULT, '9. v6 700 ms -> default window, NOT 700 deg');
    check(migrate(6, 1000) === DEG_DEFAULT, '9. v6 max ms -> default window');
    check(migrate(2, 0) === DEG_DEFAULT, '9. v2 has no field -> default');
    check(migrate(7, 270) === 270, '9. v7 value is taken literally');
    check(migrate(7, 5000) === DEG_MAX, '9. v7 out-of-range clamps to max');
    // The trap this guards: taking a stored 700 literally would clamp to 360 and
    // silently double the smoothing the rider had configured.
    check(migrate(6, 700) !== DEG_MAX, '9. v6 700 does not become 360 deg');
}

// 10. Migration must not re-fire on a v7 blob (the FW-053 lesson in tuning_config.c).
{
    let ok = true;
    for (const v of [0, 15, 180, 360]) ok = ok && migrate(7, v) === v;
    check(ok, '10. v7 round-trips every setting unchanged');
}

// 11. Bumping TUNING_VERSION must not orphan v6's start_steps field.
{
    const tc = SRC('tuning_config.c');
    check(/version\s*>=\s*TUNING_VERSION_V6/.test(tc),
        '11. start_steps is gated on V6, not on the current TUNING_VERSION');
    check(/version\s*==\s*TUNING_VERSION_V6\s*\|\|\s*version\s*==\s*TUNING_VERSION/.test(tc),
        '11. apply_blob still accepts a v6 blob');
}

console.log(failures === 0 ? '\nAll FW-085 checks passed.' : `\n${failures} FW-085 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
