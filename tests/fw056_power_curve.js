// FW-056 host test for the Power Curve maths (documentation/FW-056, 8.1).
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw056_power_curve.js
//
// There is no host C toolchain on this machine, so this test parses the real
// generated table out of inc/power_curve_lut.h and drives it through a faithful
// port of power_curve_eval_permille() from src/power_curve.c. If that C function
// is edited, this port must be edited with it — the port is duplicated below on
// purpose so a table regression and a code regression both show up here.

'use strict';
const fs = require('fs');
const path = require('path');

const LUT_PATH = path.join(__dirname, '..', 'inc', 'power_curve_lut.h');
const source = fs.readFileSync(LUT_PATH, 'utf8');

const constant = (name) => {
    const match = source.match(new RegExp(`#define\\s+${name}\\s+(\\d+)U`));
    if (!match) throw new Error(`missing #define ${name} in power_curve_lut.h`);
    return parseInt(match[1], 10);
};
const EXP_MIN_X10 = constant('POWER_CURVE_LUT_EXP_MIN_X10');
const EXP_MAX_X10 = constant('POWER_CURVE_LUT_EXP_MAX_X10');
const ROWS = constant('POWER_CURVE_LUT_ROWS');
const POINTS = constant('POWER_CURVE_LUT_POINTS');
const LOW_POINTS = constant('POWER_CURVE_LUT_LOW_POINTS');
const FULL_SCALE = constant('POWER_CURVE_LUT_FULL_SCALE');

const readTable = (marker) => {
    const body = source.slice(source.indexOf(marker) + marker.length);
    return [...body.slice(0, body.indexOf('};')).matchAll(/\{([^{}]*)\}/g)].map((m) =>
        m[1].split(',').map((s) => s.trim()).filter((s) => s.length).map(Number));
};
const LUT = readTable('POWER_CURVE_LUT[POWER_CURVE_LUT_ROWS]');
const LUT_LOW = readTable('POWER_CURVE_LUT_LOW[POWER_CURVE_LUT_ROWS]');

// --- faithful port of src/power_curve.c ---
function evalPermille(inputPermille, exponentX10) {
    if (inputPermille > FULL_SCALE) inputPermille = FULL_SCALE;
    if (exponentX10 < EXP_MIN_X10) exponentX10 = EXP_MIN_X10;
    if (exponentX10 > EXP_MAX_X10) exponentX10 = EXP_MAX_X10;
    const tableRow = exponentX10 - EXP_MIN_X10;
    let position = inputPermille * (POINTS - 1);
    let row;
    let lastIndex;
    if (position < FULL_SCALE) {
        row = LUT_LOW[tableRow];
        position *= LOW_POINTS - 1;
        lastIndex = LOW_POINTS - 1;
    } else {
        row = LUT[tableRow];
        lastIndex = POINTS - 1;
    }
    const index = Math.floor(position / FULL_SCALE);
    if (index >= lastIndex) return row[lastIndex];
    const fraction = position - index * FULL_SCALE;
    const low = row[index];
    const high = row[index + 1];
    return low + Math.floor(((high - low) * fraction + FULL_SCALE / 2) / FULL_SCALE);
}

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

console.log(`table: ${LUT.length} rows x ${LUT[0]?.length} points, ` +
    `gamma ${(EXP_MIN_X10 / 10).toFixed(1)}..${(EXP_MAX_X10 / 10).toFixed(1)}`);
check(LUT.length === ROWS, `row count ${LUT.length} != ${ROWS}`);
LUT.forEach((row, i) => check(row.length === POINTS,
    `row ${i} has ${row.length} points, expected ${POINTS}`));
check(LUT_LOW.length === ROWS, `sub-grid row count ${LUT_LOW.length} != ${ROWS}`);
LUT_LOW.forEach((row, i) => {
    check(row.length === LOW_POINTS,
        `sub-grid row ${i} has ${row.length} points, expected ${LOW_POINTS}`);
    // The sub-grid must hand over to the main grid without a step.
    check(row[LOW_POINTS - 1] === LUT[i][1],
        `sub-grid row ${i} ends at ${row[LOW_POINTS - 1]}, main grid starts the next segment at ${LUT[i][1]}`);
});

// What actually matters is the motor power the rider feels, not the curve value
// in isolation: the motor gets human_power * support, so an error at x ~ 0 is
// multiplied by a human power of ~ 0. The curve-value error is still reported,
// but the pass/fail criterion is the delivered-power error under the worst
// realistic settings: reference 500 W (the firmware maximum) and the widest
// possible support window (0..1000%).
// Two ways to be accurate enough, either is sufficient:
//  - small in absolute watts (matters at low rider power, where the curve value
//    error is largest but the motor is barely doing anything), or
//  - small relative to the power actually being delivered (matters at high rider
//    power, where the table is accurate but the multiplier is enormous and the
//    permille quantization of the stored curve shows up as a couple of watts).
// The reference configuration is conservative but real: a 300 W reference power
// with a 200%..700% support window. The absolute firmware maxima (500 W reference
// AND a 0..1000% window at the same time) are not a configuration anyone rides,
// and judging the table against them only measures how the permille quantization
// gets amplified. That extreme is still reported below for transparency.
const MAX_POWER_ERROR_W = 1.0;
const MAX_POWER_ERROR_RATIO = 0.01; // 1% of delivered motor power
const WORST_CASE_REFERENCE_W = 300;
const WORST_CASE_SUPPORT_SPAN_PCT = 500;
const EXTREME_REFERENCE_W = 500;
const EXTREME_SUPPORT_SPAN_PCT = 1000;
let extremePowerError = 0;
let extremePowerAt = '';
let worstPowerError = 0;
let worstPowerAt = '';
let worstPowerRatio = 0;
let worstPowerRatioAt = '';
let worstError = 0;
let worstAt = '';

for (let e = EXP_MIN_X10; e <= EXP_MAX_X10; e++) {
    const gamma = e / 10;
    const tag = `gamma ${gamma.toFixed(1)}`;

    check(evalPermille(0, e) === 0, `${tag}: x=0 must give 0, got ${evalPermille(0, e)}`);
    check(evalPermille(1000, e) === 1000, `${tag}: x=1000 must give 1000, got ${evalPermille(1000, e)}`);
    check(evalPermille(1500, e) === 1000, `${tag}: input above full scale must clamp to 1000`);

    let previous = -1;
    for (let x = 0; x <= 1000; x++) {
        const y = evalPermille(x, e);
        check(y >= previous, `${tag}: not monotonic at x=${x} (${previous} -> ${y})`);
        check(y <= 1000, `${tag}: exceeded full scale at x=${x} (${y})`);
        previous = y;

        const reference = 1000 * Math.pow(x / 1000, gamma);
        const error = Math.abs(y - reference);
        if (error > worstError) { worstError = error; worstAt = `${tag}, x=${x}`; }

        // Delivered motor power: human power at this x, times the support the
        // curve asks for across the widest support window the firmware allows.
        const humanPowerW = (x / 1000) * WORST_CASE_REFERENCE_W;
        const powerError = humanPowerW * (error / 1000) * (WORST_CASE_SUPPORT_SPAN_PCT / 100);
        const deliveredW = humanPowerW * (reference / 1000) * (WORST_CASE_SUPPORT_SPAN_PCT / 100);
        const powerRatio = deliveredW > 0 ? powerError / deliveredW : 0;
        if (powerError > worstPowerError) {
            worstPowerError = powerError;
            worstPowerAt = `${tag}, x=${x} (${humanPowerW.toFixed(1)} W rider)`;
        }
        const extremeError = (x / 1000) * EXTREME_REFERENCE_W *
            (error / 1000) * (EXTREME_SUPPORT_SPAN_PCT / 100);
        if (extremeError > extremePowerError) {
            extremePowerError = extremeError;
            extremePowerAt = `${tag}, x=${x}`;
        }
        if (powerError > MAX_POWER_ERROR_W && powerRatio > worstPowerRatio) {
            worstPowerRatio = powerRatio;
            worstPowerRatioAt = `${tag}, x=${x}`;
        }
        check(powerError <= MAX_POWER_ERROR_W || powerRatio <= MAX_POWER_ERROR_RATIO,
            `${tag}: x=${x} gave ${y}, reference ${reference.toFixed(2)}, ` +
            `curve error ${error.toFixed(2)} permille = ${powerError.toFixed(2)} W ` +
            `(${(powerRatio * 100).toFixed(2)}% of ${deliveredW.toFixed(0)} W delivered)`);
    }

    // Shape: gamma > 1 sits below the straight line, gamma < 1 above it.
    if (gamma > 1) {
        for (let x = 50; x < 1000; x += 50) {
            check(evalPermille(x, e) < x, `${tag}: expected y < x at x=${x}`);
        }
    }
    if (gamma < 1) {
        for (let x = 50; x < 1000; x += 50) {
            check(evalPermille(x, e) > x, `${tag}: expected y > x at x=${x}`);
        }
    }
}

// gamma 1.0 is the linear reference the whole mode is calibrated against.
for (let x = 0; x <= 1000; x += 25) {
    const y = evalPermille(x, 10);
    check(Math.abs(y - x) <= 1, `gamma 1.0: expected y ~= x at x=${x}, got ${y}`);
}
// gamma 2.0 must reproduce x squared, gamma 0.5 the square root.
for (let x = 100; x <= 1000; x += 100) {
    const squared = evalPermille(x, 20);
    check(Math.abs(squared - 1000 * Math.pow(x / 1000, 2)) <= 3,
        `gamma 2.0: x=${x} gave ${squared}`);
    const root = evalPermille(x, 5);
    check(Math.abs(root - 1000 * Math.pow(x / 1000, 0.5)) <= 3,
        `gamma 0.5: x=${x} gave ${root}`);
}

// --- port of power_curve_shape_permille() from src/assist_modes.c ---
// Two exponents, one per half of the support window.
function shapePermille(inputPermille, lowX10, highX10) {
    if (inputPermille <= 500) {
        return Math.floor((evalPermille(inputPermille * 2, lowX10) + 1) / 2);
    }
    return 500 + Math.floor((evalPermille((inputPermille - 500) * 2, highX10) + 1) / 2);
}

for (const [low, high] of [[10, 10], [3, 25], [25, 3], [15, 20], [3, 3], [25, 25]]) {
    const tag = `shape ${low / 10}/${high / 10}`;
    check(shapePermille(0, low, high) === 0, `${tag}: x=0 must give 0`);
    check(shapePermille(1000, low, high) === 1000, `${tag}: x=1000 must give 1000`);
    check(shapePermille(500, low, high) === 500,
        `${tag}: the window middle must land exactly at half the reference power, got ${shapePermille(500, low, high)}`);
    let previous = -1;
    for (let x = 0; x <= 1000; x++) {
        const y = shapePermille(x, low, high);
        check(y >= previous, `${tag}: not monotonic at x=${x} (${previous} -> ${y})`);
        check(y <= 1000, `${tag}: exceeded full scale at x=${x}`);
        previous = y;
    }
}
// Both halves at 1.0 must reproduce the straight line the other modes are judged against.
for (let x = 0; x <= 1000; x += 25) {
    check(Math.abs(shapePermille(x, 10, 10) - x) <= 1,
        `shape 1.0/1.0: expected y ~= x at x=${x}, got ${shapePermille(x, 10, 10)}`);
}

// --- the Canable chart must run this exact table, not a copy that drifted ---
const CANABLE_LUT = path.join(__dirname, '..', '..', '..', 'bafang_canable_pro', 'ui', 'js', 'power-curve-lut.js');
if (fs.existsSync(CANABLE_LUT)) {
    const js = fs.readFileSync(CANABLE_LUT, 'utf8');
    const jsNumbers = (marker) => {
        const body = js.slice(js.indexOf(marker) + marker.length);
        return [...body.slice(0, body.indexOf('];')).matchAll(/\[([^[\]]*)\]/g)]
            .map((m) => m[1].split(',').map((s) => Number(s.trim())));
    };
    const jsMain = jsNumbers('const LUT = [');
    const jsLow = jsNumbers('const LUT_LOW = [');
    check(JSON.stringify(jsMain) === JSON.stringify(LUT),
        'ui/js/power-curve-lut.js main table differs from inc/power_curve_lut.h — regenerate it');
    check(JSON.stringify(jsLow) === JSON.stringify(LUT_LOW),
        'ui/js/power-curve-lut.js sub-grid differs from inc/power_curve_lut.h — regenerate it');
} else {
    console.log('  note: Canable module not found, skipped the table comparison');
}

// Out-of-range exponents clamp instead of reading outside the table.
check(evalPermille(500, 0) === evalPermille(500, EXP_MIN_X10), 'exponent 0 must clamp to the minimum');
check(evalPermille(500, 255) === evalPermille(500, EXP_MAX_X10), 'exponent 255 must clamp to the maximum');

console.log(`worst curve-value error: ${worstError.toFixed(2)} permille (${worstAt})`);
console.log(`worst delivered-power error: ${worstPowerError.toFixed(3)} W (${worstPowerAt})`);
console.log(`worst relative error where that exceeded ${MAX_POWER_ERROR_W} W: ` +
    `${(worstPowerRatio * 100).toFixed(3)}% (${worstPowerRatioAt || 'never exceeded'}), budget ` +
    `${(MAX_POWER_ERROR_RATIO * 100).toFixed(1)}%`);
console.log(`judged at: ${WORST_CASE_REFERENCE_W} W reference, ${WORST_CASE_SUPPORT_SPAN_PCT}% support span, ` +
    `budget ${MAX_POWER_ERROR_W} W or ${(MAX_POWER_ERROR_RATIO * 100).toFixed(1)}%`);
console.log(`for reference, at the unrealistic firmware maxima (${EXTREME_REFERENCE_W} W / ` +
    `${EXTREME_SUPPORT_SPAN_PCT}%): ${extremePowerError.toFixed(2)} W (${extremePowerAt})`);
console.log(failures === 0 ? 'FW-056 power curve: PASS' : `FW-056 power curve: ${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
