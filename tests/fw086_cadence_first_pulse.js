// FW-086 host test: the first cadence pulse after a standstill must carry a real
// measurement, not a leftover interval.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw086_cadence_first_pulse.js
//
// No host C toolchain here, so this ports the forward-step branch of the quadrature
// decoder out of src/main.c, in the same style as the other tests here. Both the
// fixed and the pre-FW-086 behaviour are modelled, so the test documents the exact
// regression it guards against rather than just asserting the happy path.

'use strict';
const fs = require('fs');
const path = require('path');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

const src = fs.readFileSync(path.join(__dirname, '..', 'src', 'main.c'), 'utf8');
const cfg = fs.readFileSync(path.join(__dirname, '..', 'inc', 'config.h'), 'utf8');
const defineOf = (name) => {
    const m = cfg.match(new RegExp(`#define\\s+${name}\\s+(\\d+)`));
    if (!m) throw new Error(`${name} not found in config.h`);
    return Number(m[1]);
};
const STEPS_PER_PULSE = defineOf('PAS_STEPS_PER_PULSE');
const TICK_HZ = 4000;
const CYCLE_TICKS_SATURATED = 64000; // src/main.c:1666 clamp

// --- port of the decoder's cadence path from src/main.c ---
// `fixed` selects the FW-086 behaviour (reset the interval on the first forward
// step after a stop) versus the original.
class Decoder {
    constructor(fixed) {
        this.fixed = fixed;
        this.cadence = 0; this.startPhase = 0;
        this.cycleTicks = 0; this.fwdSteps = 0; this.fwdRun = 0;
        this.idleTicks = 0;
    }
    tick() {
        if (this.cycleTicks < CYCLE_TICKS_SATURATED) this.cycleTicks++;
        if (this.idleTicks < CYCLE_TICKS_SATURATED) this.idleTicks++;
    }
    stop() { // src/main.c:1728 — note it never touched pas_cycle_ticks
        this.cadence = 0; this.startPhase = 0; this.fwdSteps = 0; this.fwdRun = 0;
    }
    forwardStep() {
        this.idleTicks = 0;
        // FW-086: this step is the interval's ORIGIN, not its first count.
        const restart = this.fixed && this.fwdRun === 0;
        if (restart) { this.cycleTicks = 0; this.fwdSteps = 0; }
        if (this.fwdRun < 250) this.fwdRun++;
        if (this.fwdRun >= 2 && this.cadence === 0 && !this.startPhase) { // START_PHASE_STEPS
            this.startPhase = 1; // FW-087: flag only, cadence stays 0
        }
        if (!restart && ++this.fwdSteps >= STEPS_PER_PULSE) {
            this.fwdSteps = 0;
            if (this.cycleTicks > 70) {
                this.cadence = Math.floor(10000 / this.cycleTicks);
                this.startPhase = 0;
            }
            this.cycleTicks = 0;
            return true; // a pulse fired
        }
        return false;
    }
}

// Crank at `rpm` produces one cadence pulse every 10000/rpm ticks (24 pulses/rev at
// 4 kHz — the same relation main.c inverts), i.e. this many ticks per quadrature step.
const ticksPerStep = (rpm) => Math.round(10000 / rpm / STEPS_PER_PULSE);

// Idle long enough to saturate the interval counter, then pedal off at `rpm` and
// report the cadence published by each pulse.
function startFromStandstill(fixed, rpm, pulses = 3) {
    const d = new Decoder(fixed);
    for (let i = 0; i < CYCLE_TICKS_SATURATED + 10; i++) d.tick();
    d.stop();
    const perStep = ticksPerStep(rpm);
    const published = [];
    let steps = 0;
    while (published.length < pulses && steps < 200) {
        for (let i = 0; i < perStep; i++) d.tick();
        if (d.forwardStep()) published.push({ cadence: d.cadence, startPhase: d.startPhase, step: steps + 1 });
        steps++;
    }
    return published;
}

console.log(`PAS_STEPS_PER_PULSE=${STEPS_PER_PULSE}, tick ${TICK_HZ} Hz`);

// 1. The regression this card fixes: without it the first pulse publishes 0 rpm.
{
    const before = startFromStandstill(false, 60);
    check(before[0].cadence === 0,
        `1. pre-FW-086 first pulse published 0 rpm (got ${before[0].cadence}) — the defect`);
    check(before[0].startPhase === 0,
        '1. pre-FW-086 cleared the start phase with that zero');
    check(before[1].cadence > 0,
        '1. pre-FW-086 only recovered on the SECOND pulse');
}

// 2. With the fix the very first pulse carries a true measurement.
{
    for (const rpm of [30, 45, 60, 90]) {
        const after = startFromStandstill(true, rpm);
        check(Math.abs(after[0].cadence - rpm) <= 1,
            `2. first pulse reads ${rpm} rpm (got ${after[0].cadence})`);
    }
}

// 3. the start phase is never cleared by a zero — the flag that keeps the launch on the
//    torque-derived Iq path (assist_modes.c:695) must only fall to a real value.
{
    let ok = true;
    for (const rpm of [20, 30, 60, 110]) {
        for (const p of startFromStandstill(true, rpm)) {
            if (p.startPhase === 0 && p.cadence === 0) ok = false;
        }
    }
    check(ok, '3. the start phase is never cleared by a zero cadence');
}

// 4. The first pulse still spans a full PAS_STEPS_PER_PULSE, so it is not a short
//    interval reading high.
{
    const after = startFromStandstill(true, 60);
    check(after[0].step === STEPS_PER_PULSE + 1,
        `4. first pulse fires one step later, after a full interval (step ${after[0].step})`);
    check(after[0].cadence <= 61, '4. first reading is not inflated by a short interval');
}

// 5. Steady riding is unaffected — later pulses agree with the first.
{
    const after = startFromStandstill(true, 60, 4);
    const spread = Math.max(...after.map((p) => p.cadence)) - Math.min(...after.map((p) => p.cadence));
    check(spread <= 1, `5. steady riding unchanged across pulses (spread ${spread})`);
}

// 6. The fast-reading guard is untouched: an impossibly short interval is still rejected.
{
    const d = new Decoder(true);
    d.cadence = 60; d.fwdRun = 10; d.fwdSteps = STEPS_PER_PULSE - 1;
    d.cycleTicks = 50; // < 70 -> must not publish
    d.forwardStep();
    check(d.cadence === 60, '6. interval below the >70 guard does not publish a cadence');
}

// 7. The fix is actually present in main.c, gated on fwd_run and placed before its
//    increment — after it, fwd_run is never 0 and the reset would never fire.
{
    const from = src.indexOf('if(st>0){');
    const branch = src.slice(from, src.indexOf('}else if(st<0){', from));
    check(/cadence_interval_restart\s*=\s*\(fwd_run==0\)/.test(branch),
        '7. restart is detected on fwd_run==0 in the forward-step branch');
    check(/if\(cadence_interval_restart\)\{\s*pas_cycle_ticks=0;\s*pas_fwd_steps=0;\s*\}/.test(branch),
        '7. restart zeroes the interval and the step counter');
    check(branch.indexOf('cadence_interval_restart =') < branch.indexOf('if(fwd_run<250)fwd_run++'),
        '7. restart is evaluated BEFORE the fwd_run increment');
    // Without the short-circuit the restart step would be counted, the pulse would fire a
    // step early over PAS_STEPS_PER_PULSE-1 gaps, and every first reading would be 4/3 high.
    check(/if\(!cadence_interval_restart\s*&&\s*\+\+pas_fwd_steps>=PAS_STEPS_PER_PULSE\)/.test(branch),
        '7. the restart step is not counted toward the pulse');
}

// 8. The cadence constant, the tick rate and the transitions-per-revolution figure are one
//    equation, not three independent numbers. main.c publishes 10000/ticks; with N
//    transitions per revolution that equals the true cadence only when N == 96. Field
//    readings are correct, so N IS 96 — which is also what fixes FW-085's degree scale.
//    Changing PAS_STEPS_PER_PULSE or the 10000 alone silently rescales reported cadence.
{
    const TRANSITIONS_PER_REV = 96;
    const m = src.match(/MS\.cadence\s*=\s*(\d+)\/pas_cycle_ticks/);
    check(!!m, '8. cadence formula found in main.c');
    const constant = Number(m[1]);
    // ticks per pulse at C rpm = (60/C * TICK_HZ) / (N / STEPS_PER_PULSE)
    const ticksPerPulse = (rpm) => (60 / rpm) * TICK_HZ / (TRANSITIONS_PER_REV / STEPS_PER_PULSE);
    let ok = true;
    for (const rpm of [30, 60, 90, 120]) {
        ok = ok && Math.abs(Math.floor(constant / ticksPerPulse(rpm)) - rpm) <= 1;
    }
    check(ok, `8. constant ${constant} is consistent with ${TRANSITIONS_PER_REV} transitions/rev at ${TICK_HZ} Hz`);
    // Guard the reconciliation recorded in config.h: 48 would be single-channel edges.
    check(/RESOLVED - 96 quadrature transitions/.test(cfg),
        '8. config.h records the resolved 96-transitions-per-rev finding');
    check(!/VERIFY by measuring/.test(cfg),
        '8. the stale "VERIFY by measuring" note is gone');
}

console.log(failures === 0 ? '\nAll FW-086 checks passed.' : `\n${failures} FW-086 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
