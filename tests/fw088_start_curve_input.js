// FW-088 host test: the support curve must not read a standing start as "no effort".
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw088_start_curve_input.js
//
// No host C toolchain here, so this ports calculate_human_power_mw() and the support
// window from src/assist_modes.c, in the same style as the other tests here.
//
// The defect: power = pedal load x crank speed, so a standing start is ~0 W however hard
// the pedal is pushed. Progressive and Curve fed that 0 to the support curve and got
// support_min_pct back — least help exactly when pulling away needs the most. Linear was
// immune (constant ratio), which is why only two of the three modes felt weak.

'use strict';
const fs = require('fs');
const path = require('path');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

const modes = fs.readFileSync(path.join(__dirname, '..', 'src', 'assist_modes.c'), 'utf8');
const cfg = fs.readFileSync(path.join(__dirname, '..', 'inc', 'config.h'), 'utf8');
const CURVE_RPM = Number(cfg.match(/#define\s+START_PHASE_CURVE_RPM\s+(\d+)/)[1]);

// --- port of calculate_human_power_mw() (assist_modes.c) ---
const NUM = 1694, DEN = 1000;
const humanPowerMw = (loadCentikg, cadenceRpm) => {
    const product = loadCentikg * cadenceRpm;
    return product + Math.floor((product * (NUM - DEN) + DEN / 2) / DEN);
};

// --- port of the support window: power -> permille of reference -> curve -> ratio ---
const inputPermille = (powerMw, referenceW) =>
    Math.min(1000, Math.floor((powerMw / 1000) * 1000 / referenceW));
// Straight line (progression 0 / exponent 1.0) is enough to expose the defect.
const supportRatio = (powerMw, cfgLevel) => {
    const x = inputPermille(powerMw, cfgLevel.reference_power_w);
    return cfgLevel.support_min_pct +
        Math.floor((cfgLevel.support_max_pct - cfgLevel.support_min_pct) * x / 1000);
};

const level = { reference_power_w: 200, support_min_pct: 50, support_max_pct: 300 };
const HARD_PUSH_CENTIKG = 3000; // 30 kg — a real standing-start shove

// What the firmware feeds the curve, before and after the fix. Two states count as a
// launch: start_phase (cranks turning, no interval measured yet) and without_rotation
// (deliberate push from a dead stop) — in both the cranks are too slow for power to mean
// anything, which is the whole reason for the substitution.
const curveInput = (fixed, loadCentikg, startPhase, realCadence, withoutRotation = false) => {
    const launching = startPhase || withoutRotation;
    const powerCadence = startPhase ? 0 : realCadence;
    return fixed && launching
        ? humanPowerMw(loadCentikg, CURVE_RPM)
        : humanPowerMw(loadCentikg, powerCadence);
};

console.log(`FW-088: start-phase curve cadence = ${CURVE_RPM} rpm`);

// 1. The defect: at launch the old code handed the curve a zero and got minimum support.
{
    const before = supportRatio(curveInput(false, HARD_PUSH_CENTIKG, true, 0), level);
    check(before === level.support_min_pct,
        `1. pre-FW-088 a 30 kg standing start earned only support_min (${before}%)`);
}

// 2. After the fix the same push earns the ratio it would at a normal cadence.
{
    const after = supportRatio(curveInput(true, HARD_PUSH_CENTIKG, true, 0), level);
    const riding = supportRatio(humanPowerMw(HARD_PUSH_CENTIKG, CURVE_RPM), level);
    check(after === riding,
        `2. the same load earns the same ratio launching as riding (${after}% vs ${riding}%)`);
    check(after > level.support_min_pct,
        `2. and it is more than the minimum (${after}% > ${level.support_min_pct}%)`);
}

// 3. Effort still matters at launch — this must not become a flat "max support on start".
{
    const ratios = [500, 1000, 2000, 3000].map(
        (kg) => supportRatio(curveInput(true, kg, true, 0), level));
    let monotonic = true;
    for (let i = 1; i < ratios.length; i++) if (ratios[i] < ratios[i - 1]) monotonic = false;
    check(monotonic, `3. harder pushes still earn more support at launch (${ratios.join(' < ')})`);
    check(ratios[0] < ratios[ratios.length - 1], '3. a light touch does not get a hard shove\'s help');
    check(ratios[ratios.length - 1] <= level.support_max_pct, '3. the support window still caps it');
}

// 4. Once a cadence is measured the start phase is over and nothing is substituted.
{
    for (const rpm of [30, 60, 90]) {
        const real = humanPowerMw(HARD_PUSH_CENTIKG, rpm);
        check(curveInput(true, HARD_PUSH_CENTIKG, false, rpm) === real,
            `4. riding at ${rpm} rpm uses the real power, not the nominal one`);
    }
}

// 5. Reported rider power must stay honest — only the curve input is substituted, or the
//    display and the power ceiling would both be inflated by a number nobody produced.
{
    // Anchor on the power calls themselves rather than a byte offset, so growing the
    // comment above cannot silently move the window off them.
    const from = modes.indexOf('uint8_t power_cadence');
    const block = modes.slice(from, modes.indexOf('curve_basis_power_mw', from) + 900);
    check(/uint32_t human_power_mw = calculate_human_power_mw\(\s*prepared\.human_load_centikg, power_cadence\);/.test(block),
        '5. reported human power still uses the real (zero) launch cadence');
    check(/motor_power_mw\s*=\s*\(assist_basis_power_mw \* support_ratio_pct\)/.test(block),
        '5. motor power still derives from the real basis power, not the substituted one');
    check(/calculate_support_ratio_pct\(\s*curve_basis_power_mw,/.test(block),
        '5. only the support curve reads the substituted value');
}

// 6. A without-rotation launch is the SAME situation and must get the same treatment.
//    "Assist without crank rotation" means the cranks are not turning at all, so
//    start_phase (which needs forward crank steps) never rises — gating on it alone left
//    this path on support_min, in the one case where the rider leans hardest on the pedal.
{
    const noRotation = { startPhase: false, withoutRotation: true };
    const before = supportRatio(
        curveInput(false, HARD_PUSH_CENTIKG, noRotation.startPhase, 0, noRotation.withoutRotation), level);
    check(before === level.support_min_pct,
        `6. before the fix a dead-stop push earned only support_min (${before}%)`);

    const after = supportRatio(
        curveInput(true, HARD_PUSH_CENTIKG, noRotation.startPhase, 0, noRotation.withoutRotation), level);
    const riding = supportRatio(humanPowerMw(HARD_PUSH_CENTIKG, CURVE_RPM), level);
    check(after === riding,
        `6. a dead-stop push now earns the riding ratio (${after}% vs ${riding}%)`);

    const ratios = [500, 1000, 2000, 3000].map(
        (kg) => supportRatio(curveInput(true, kg, false, 0, true), level));
    let monotonic = true;
    for (let i = 1; i < ratios.length; i++) if (ratios[i] < ratios[i - 1]) monotonic = false;
    check(monotonic, `6. effort still scales without crank rotation (${ratios.join(' < ')})`);
}

// 7. Structural: the substitution covers BOTH launch flags, and nothing else.
{
    check(/curve_basis_power_mw =\s*\(prepared\.start_phase \|\| prepared\.without_rotation_active\) \?/.test(modes),
        '7. the substitution is gated on both launch flags');
}

console.log(failures === 0 ? '\nAll FW-088 checks passed.' : `\n${failures} FW-088 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
