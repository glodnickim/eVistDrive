// FW-056: generator of inc/power_curve_lut.h — the y = x^gamma lookup table
// used by ASSIST_MODE_POWER_CURVE.
//
// Run from BAFANG_GD32F303RCT6/:  node tools/generate_power_curve_lut.js
//
// Written in Node (not Python) because this machine has no Python toolchain;
// Node is already required for the Canable side, so one runtime covers both.
//
// Accuracy note (see documentation/FW-056_POWER_CURVE_MODE.md, 4.3): for gamma < 1
// the curve is vertical at x = 0, so a uniform-X table is inaccurate in the first
// segment when measured on the curve value itself. That measure is the wrong one —
// the motor gets human_power * support, and the first segment is exactly where
// human power is near zero, so the delivered-power error there is a fraction of a
// watt. The table is validated on delivered motor power in tests/fw056_power_curve.js.

'use strict';
const fs = require('fs');
const path = require('path');

const EXP_MIN_X10 = 3;    // gamma 0.3 — very aggressive, bends the other way
const EXP_MAX_X10 = 25;   // gamma 2.5
const POINTS = 65;        // x = 0/64 .. 64/64
// For gamma < 1 essentially all of the interpolation error sits inside the very
// first main segment, where the curve leaves zero vertically. This sub-grid
// splits that one segment into LOW_POINTS-1 pieces. It costs 414 B and is inert
// for gamma >= 1, where the curve is flat there anyway.
const LOW_POINTS = 9;     // x = 0 .. 1/64, in 8 steps
const FULL_SCALE = 1000;  // permille

function sample(gamma, x) {
    return Math.round(FULL_SCALE * Math.pow(x, gamma));
}

function monotonic(row) {
    for (let i = 1; i < row.length; i++) {
        if (row[i] < row[i - 1]) row[i] = row[i - 1];
    }
    return row;
}

const rows = [];
const lowRows = [];
for (let e = EXP_MIN_X10; e <= EXP_MAX_X10; e++) {
    const gamma = e / 10;
    const row = [];
    for (let i = 0; i < POINTS; i++) row.push(sample(gamma, i / (POINTS - 1)));
    // The contract the firmware relies on: 0 -> 0 and full scale -> full scale.
    row[0] = 0;
    row[POINTS - 1] = FULL_SCALE;
    rows.push(monotonic(row));

    const lowRow = [];
    for (let i = 0; i < LOW_POINTS; i++) {
        lowRow.push(sample(gamma, (i / (LOW_POINTS - 1)) / (POINTS - 1)));
    }
    lowRow[0] = 0;
    lowRow[LOW_POINTS - 1] = row[1]; // must meet the main grid exactly
    lowRows.push(monotonic(lowRow));
}

const width = String(FULL_SCALE).length;
const format = (table, perLine) => table.map((row, index) => {
    const gamma = ((EXP_MIN_X10 + index) / 10).toFixed(1);
    const cells = row.map((v) => String(v).padStart(width, ' '));
    const lines = [];
    for (let i = 0; i < cells.length; i += perLine) {
        lines.push('\t\t' + cells.slice(i, i + perLine).join(', ') + ',');
    }
    return `\t/* gamma ${gamma} */\n\t{\n${lines.join('\n')}\n\t},`;
}).join('\n');
const body = format(rows, 11);
const lowBody = format(lowRows, 9);

const header = `/*
 * FW-056: y = x^gamma lookup table for ASSIST_MODE_POWER_CURVE.
 *
 * GENERATED FILE — do not edit by hand.
 * Regenerate with: node tools/generate_power_curve_lut.js
 *
 * Generator parameters:
 *   exponent range : ${EXP_MIN_X10} .. ${EXP_MAX_X10} (x10, i.e. gamma ${(EXP_MIN_X10 / 10).toFixed(1)} .. ${(EXP_MAX_X10 / 10).toFixed(1)}, step 0.1)
 *   rows           : ${rows.length}
 *   points per row : ${POINTS} (x = 0/${POINTS - 1} .. ${POINTS - 1}/${POINTS - 1}, uniform)
 *   low sub-grid   : ${LOW_POINTS} points across the first segment (x = 0 .. 1/${POINTS - 1})
 *   value scale    : 0 .. ${FULL_SCALE} permille
 *   flash cost     : ${rows.length} * (${POINTS} + ${LOW_POINTS}) * 2 B = ${rows.length * (POINTS + LOW_POINTS) * 2} B
 *
 * Included only by src/power_curve.c.
 */
#ifndef POWER_CURVE_LUT_H_
#define POWER_CURVE_LUT_H_

#include <stdint.h>

#define POWER_CURVE_LUT_EXP_MIN_X10 ${EXP_MIN_X10}U
#define POWER_CURVE_LUT_EXP_MAX_X10 ${EXP_MAX_X10}U
#define POWER_CURVE_LUT_ROWS ${rows.length}U
#define POWER_CURVE_LUT_POINTS ${POINTS}U
#define POWER_CURVE_LUT_LOW_POINTS ${LOW_POINTS}U
#define POWER_CURVE_LUT_FULL_SCALE ${FULL_SCALE}U

static const uint16_t POWER_CURVE_LUT[POWER_CURVE_LUT_ROWS][POWER_CURVE_LUT_POINTS] = {
${body}
};

/* Same curves, sampled across the first main segment only (x = 0 .. 1/${POINTS - 1}). */
static const uint16_t POWER_CURVE_LUT_LOW[POWER_CURVE_LUT_ROWS][POWER_CURVE_LUT_LOW_POINTS] = {
${lowBody}
};

#endif /* POWER_CURVE_LUT_H_ */
`;

const out = path.join(__dirname, '..', 'inc', 'power_curve_lut.h');
fs.writeFileSync(out, header, 'utf8'); // repo uses LF in inc/ and src/
console.log(`wrote ${out} (${rows.length} rows x ${POINTS} points + ${LOW_POINTS}-point sub-grid, ` +
    `${rows.length * (POINTS + LOW_POINTS) * 2} B of flash)`);

// The Canable preview chart must draw the curve the controller actually runs, not
// a floating-point lookalike, so the same generator emits the same table as a JS
// module together with a port of power_curve_eval_permille().
const jsTable = (table) => table.map((row, index) =>
    `    /* gamma ${((EXP_MIN_X10 + index) / 10).toFixed(1)} */\n    [${row.join(', ')}],`).join('\n');

const jsModule = `// GENERATED FILE — do not edit by hand.
// Regenerate with: node tools/generate_power_curve_lut.js in the EBICS firmware repo.
//
// Byte-for-byte the table the controller runs (inc/power_curve_lut.h), plus a port
// of power_curve_eval_permille() from src/power_curve.c, so the profile preview
// chart draws the real curve instead of a floating-point approximation.
// Note this covers the curve shape only — startup boost, the power filters, smooth
// start, the Iq ramp and the speed/temperature limits are not modelled anywhere in
// the chart. Use the 0x6029 diagnostics blob to compare against the real ride.

export const POWER_CURVE_EXP_MIN_X10 = ${EXP_MIN_X10};
export const POWER_CURVE_EXP_MAX_X10 = ${EXP_MAX_X10};
export const POWER_CURVE_EXP_DEFAULT_X10 = 15;
const POINTS = ${POINTS};
const LOW_POINTS = ${LOW_POINTS};
const FULL_SCALE = ${FULL_SCALE};

const LUT = [
${jsTable(rows)}
];

const LUT_LOW = [
${jsTable(lowRows)}
];

export function evalPowerCurvePermille(inputPermille, exponentX10) {
    if (inputPermille > FULL_SCALE) inputPermille = FULL_SCALE;
    if (inputPermille < 0) inputPermille = 0;
    if (exponentX10 < POWER_CURVE_EXP_MIN_X10) exponentX10 = POWER_CURVE_EXP_MIN_X10;
    if (exponentX10 > POWER_CURVE_EXP_MAX_X10) exponentX10 = POWER_CURVE_EXP_MAX_X10;
    const tableRow = exponentX10 - POWER_CURVE_EXP_MIN_X10;
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
`;

const canableDir = path.join(__dirname, '..', '..', '..', 'bafang_canable_pro', 'ui', 'js');
if (fs.existsSync(canableDir)) {
    const jsOut = path.join(canableDir, 'power-curve-lut.js');
    fs.writeFileSync(jsOut, jsModule, 'utf8');
    console.log(`wrote ${jsOut}`);
} else {
    console.log(`skipped the Canable module: ${canableDir} not found`);
}
