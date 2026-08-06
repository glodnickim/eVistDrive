// FW-090 host test: the RUN estimator must follow a genuine rise quickly, without ever
// being fooled by the ordinary per-leg ripple.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw090_run_attack.js
//
// Two opposing requirements meet here, and the test exists to hold BOTH:
//   * re-catching assist mid-ride must not need half a crank turn of pushing (FW-090);
//   * normal pedalling must never trigger the fast path, or the per-leg pulsing that
//     FW-085 removed comes straight back. That one is the regression to fear.

'use strict';
const fs = require('fs');
const path = require('path');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

const hdr = fs.readFileSync(path.join(__dirname, '..', 'inc', 'torque_input.h'), 'utf8');
const constOf = (name) => {
    const m = hdr.match(new RegExp(`#define\\s+${name}\\s+(\\w+)`));
    if (!m) throw new Error(`${name} not found`);
    return /^\d+U?$/.test(m[1]) ? Number(m[1].replace('U', '')) : constOf(m[1]);
};
const STEPS_MAX = constOf('TORQUE_RUN_WINDOW_STEPS_MAX');
const ATTACK_NUM = constOf('TORQUE_RUN_ATTACK_NUM');
const ATTACK_DEN = constOf('TORQUE_RUN_ATTACK_DEN');
const MIN_DELTA = constOf('TORQUE_RUN_ATTACK_MIN_DELTA');
// FW-091 ships this mechanism DISABLED: the re-engagement complaint it was written for turned
// out to come from the limiter classifying pedal demand by filtered cadence, not from this
// average being slow. The behaviour below is still tested at the value the feature would use
// if switched on, so enabling it stays a one-constant change with proven behaviour.
const SHIPPED_STEPS = constOf('TORQUE_RUN_ATTACK_STEPS');
const ATTACK_STEPS = 8;

// --- port of the RUN window incl. FW-090 attack, from src/torque_input.c ---
class Run {
    constructor(deg, attack = true) {
        this.steps = Math.floor((deg * 4) / 15);
        this.attackOn = attack;
        this.buf = new Array(this.steps).fill(0);
        this.head = 0; this.filled = 0; this.sum = 0; this.value = 0; this.attackSteps = 0;
        this.seeds = 0;
    }
    seed(v) {
        this.value = v;
        this.buf = new Array(this.steps).fill(v);
        this.head = 0; this.filled = this.steps; this.sum = v * this.steps;
        this.attackSteps = 0; this.seeds++;
    }
    step(sample) {
        if (this.steps === 0) return;
        if (this.filled >= this.steps) this.sum -= this.buf[this.head];
        else this.filled++;
        this.buf[this.head] = sample;
        this.sum += sample;
        this.head++;
        if (this.head >= this.steps) this.head = 0;

        if (this.attackOn) {
            const rising = sample * ATTACK_DEN > this.value * ATTACK_NUM
                && sample >= this.value + MIN_DELTA;
            if (rising) { if (this.attackSteps < ATTACK_STEPS) this.attackSteps++; }
            else this.attackSteps = 0;
            if (this.attackSteps >= ATTACK_STEPS) { this.seed(sample); return; }
        }
        this.value = Math.floor((this.sum + Math.floor(this.filled / 2)) / this.filled);
    }
}

// A leg push shaped like a rectified sine: one hump per 48 steps (180 deg).
const legRipple = (i, amplitude) => Math.round(amplitude * Math.abs(Math.sin((Math.PI * i) / 48)));

console.log(`window ${STEPS_MAX} steps max, attack ${ATTACK_NUM}/${ATTACK_DEN}x for ${ATTACK_STEPS} steps, min delta ${MIN_DELTA}`);

// A primed window is the state the bike actually rides in: torque_input_seed_run() fills
// it when the ride latch arms, before assist is producing anything.
const primed = (amp) => {
    const r = new Run(180);
    r.seed(Math.round(amp * 2 / Math.PI)); // the true mean of a rectified sine
    r.seeds = 0;                           // count only what happens from here on
    return r;
};

// 1. THE REGRESSION GUARD: steady pedalling must never trigger the fast path once the
//    window is primed. A rectified-sine leg peaks at ~1.57x its own mean, which is exactly
//    why the 2x margin holds — this is what keeps FW-085's ripple rejection alive.
{
    const r = primed(1000);
    for (let i = 0; i < 48 * 6; i++) r.step(legRipple(i, 1000));
    check(r.seeds === 0, `1. ordinary pedalling never re-seeds (seeds: ${r.seeds})`);
    const settled = [];
    for (let i = 0; i < 48 * 2; i++) { r.step(legRipple(i, 1000)); settled.push(r.value); }
    const spread = Math.max(...settled) - Math.min(...settled);
    check(spread <= 2, `1. ripple rejection from FW-085 is intact (spread ${spread})`);
}

// 2. Same guard across the whole effort range — the margin is a ratio, so it must hold
//    equally for a gentle spin and a hard grind.
{
    for (const amp of [200, 600, 1500, 3000]) {
        const r = primed(amp);
        for (let i = 0; i < 48 * 6; i++) r.step(legRipple(i, amp));
        check(r.seeds === 0, `2. amplitude ${amp}: still no false attack`);
    }
}

// 2b. Filling from a pristine (all-zero) window is the one place a rising ripple does trip
//     the attack, at around the 8th step. That is intentional and harmless: it only makes
//     the estimate converge faster out of an empty buffer, and on the bike the latch has
//     already seeded the window before assist begins. Documented so it is not mistaken for
//     the regression that tests 1 and 2 guard against.
{
    const r = new Run(180);
    for (let i = 0; i < 48; i++) r.step(legRipple(i, 1000));
    check(r.seeds === 1, `2b. an empty window converges via one early seed (${r.seeds})`);
}

// 3. The defect FW-090 fixes: after coasting the window is full of near-zero samples, and
//    a genuine push used to need ~180 deg to show up.
{
    const before = new Run(180, false);
    const after = new Run(180, true);
    for (const r of [before, after]) for (let i = 0; i < 48; i++) r.step(0); // coasting
    const PUSH = 1200;
    const stepsTo = (r, frac) => {
        for (let n = 1; n <= 96; n++) { r.step(PUSH); if (r.value >= PUSH * frac) return n; }
        return null;
    };
    const nBefore = stepsTo(before, 0.9);
    const nAfter = stepsTo(after, 0.9);
    check(nBefore >= 40, `3. without the attack a push needed ${nBefore} steps (~${(nBefore * 3.75).toFixed(0)} deg)`);
    check(nAfter <= ATTACK_STEPS + 1, `3. with the attack it takes ${nAfter} steps (~${(nAfter * 3.75).toFixed(0)} deg)`);
    check(nAfter < nBefore / 3, '3. that is several times quicker than before');
}

// 4. A single leg peak must not be enough — the rise has to be HELD.
{
    const r = new Run(180);
    for (let i = 0; i < 48; i++) r.step(300);
    const seedsBefore = r.seeds;
    for (let i = 0; i < ATTACK_STEPS - 1; i++) r.step(2000); // a short spike, one step shy
    r.step(300);                                             // ...then it drops again
    check(r.seeds === seedsBefore, '4. a spike shorter than the hold does not re-seed');
    check(r.attackSteps === 0, '4. and the counter resets on the first quiet step');
}

// 5. Falling effort must stay slow — the whole point of the average is to ride out dips.
{
    const r = new Run(180);
    for (let i = 0; i < 48; i++) r.step(1000);
    const seedsBefore = r.seeds;
    for (let i = 0; i < 24; i++) r.step(0);
    check(r.seeds === seedsBefore, '5. a drop to zero never re-seeds');
    check(r.value > 300, `5. and the estimate decays gradually, not instantly (${r.value})`);
}

// 6. Near zero the ratio alone would be met by jitter; the absolute floor must block it.
{
    const r = new Run(180);
    for (let i = 0; i < 48; i++) r.step(3);
    const seedsBefore = r.seeds;
    for (let i = 0; i < ATTACK_STEPS * 3; i++) r.step(7); // 7 > 3*2, but only +4 absolute
    check(r.seeds === seedsBefore, '6. jitter around a near-zero average does not re-seed');
    for (let i = 0; i < ATTACK_STEPS; i++) r.step(3 + MIN_DELTA * 3);
    check(r.seeds > seedsBefore, '6. a real rise above the floor still does');
}

// 6b. The shipped default must stay OFF until FW-091 has been ridden, and the zero case
//     must be guarded explicitly — with STEPS = 0 the "counter reached target" test would
//     otherwise be true on every single step, re-seeding constantly.
{
    check(SHIPPED_STEPS === 0, `6b. shipped disabled by FW-091 (STEPS=${SHIPPED_STEPS})`);
    const src = fs.readFileSync(path.join(__dirname, '..', 'src', 'torque_input.c'), 'utf8');
    check(/if \(TORQUE_RUN_ATTACK_STEPS == 0U\) \{[^}]*run_value_native[^}]*return;/s.test(src),
        '6b. STEPS = 0 short-circuits to the plain average, not to a re-seed every step');
}

// 7. Structural: both conditions are present and the counter demands consecutive steps.
{
    const src = fs.readFileSync(path.join(__dirname, '..', 'src', 'torque_input.c'), 'utf8');
    const fn = src.slice(src.indexOf('void torque_input_run_filter_step'));
    check(/TORQUE_RUN_ATTACK_MIN_DELTA/.test(fn), '7. the absolute floor is applied');
    check(/run_attack_steps = 0U;.*only a HELD rise/s.test(fn), '7. a quiet step resets the counter');
    check(/torque_input_seed_run\(sample\)/.test(fn), '7. a confirmed attack re-seeds the window');
}

console.log(failures === 0 ? '\nAll FW-090 checks passed.' : `\n${failures} FW-090 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
