// FW-057 host test for the cadence compensation map.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw057_cadence_comp.js
//
// No host C toolchain here, so this parses the breakpoint table straight out of
// src/cadence_comp.c and drives it through a port of
// cadence_comp_multiplier_permille(). The table under test is the real one; the
// interpolation is duplicated on purpose, so edit both together.

'use strict';
const fs = require('fs');
const path = require('path');

const source = fs.readFileSync(path.join(__dirname, '..', 'src', 'cadence_comp.c'), 'utf8');
const mapBody = source.slice(source.indexOf('cadence_comp_map[] = {'));
const MAP = [...mapBody.slice(0, mapBody.indexOf('};')).matchAll(/\{\s*(\d+)\s*,\s*(\d+)U\s*\}/g)]
    .map((m) => ({ rpm: Number(m[1]), permille: Number(m[2]) }));

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

// --- port of src/cadence_comp.c ---
function multiplier(cadence) {
    const last = MAP[MAP.length - 1];
    if (cadence >= last.rpm) return last.permille;
    for (let i = 1; i < MAP.length; i++) {
        const high = MAP[i];
        if (cadence >= high.rpm) continue;
        const low = MAP[i - 1];
        const span = high.rpm - low.rpm;
        const into = cadence - low.rpm;
        if (high.permille >= low.permille) {
            return low.permille + Math.floor(((high.permille - low.permille) * into + span / 2) / span);
        }
        return low.permille - Math.floor(((low.permille - high.permille) * into + span / 2) / span);
    }
    return 1000;
}

console.log(`map: ${MAP.map((p) => `${p.rpm}rpm=${p.permille / 10}%`).join(', ')}`);

// The characteristic as specified.
const SPEC = [[0, 1000], [40, 1000], [70, 1000], [80, 820], [100, 930], [110, 1060], [120, 1320]];
SPEC.forEach(([rpm, expected]) => {
    check(multiplier(rpm) === expected,
        `${rpm} rpm must give ${expected / 10}%, got ${multiplier(rpm) / 10}%`);
});

// Above the last breakpoint the value is held, never extrapolated.
for (let rpm = 120; rpm <= 255; rpm++) {
    check(multiplier(rpm) === 1320, `${rpm} rpm must hold at 132%, got ${multiplier(rpm) / 10}%`);
}

// No steps anywhere: the rider must not feel a jump between breakpoints.
let previous = multiplier(0);
let worstStep = 0;
for (let rpm = 1; rpm <= 255; rpm++) {
    const value = multiplier(rpm);
    const step = Math.abs(value - previous);
    if (step > worstStep) worstStep = step;
    check(step <= 30,
        `step of ${step / 10}% between ${rpm - 1} and ${rpm} rpm is too abrupt`);
    previous = value;
}

// Monotonic within each declared segment, matching the map's own direction.
for (let i = 1; i < MAP.length; i++) {
    const rising = MAP[i].permille >= MAP[i - 1].permille;
    for (let rpm = MAP[i - 1].rpm; rpm < MAP[i].rpm; rpm++) {
        const step = multiplier(rpm + 1) - multiplier(rpm);
        check(rising ? step >= 0 : step <= 0,
            `segment ${MAP[i - 1].rpm}-${MAP[i].rpm} rpm changes direction at ${rpm} rpm`);
    }
}

// The compensation may only ever raise the request, never cut it below the
// hardware ceiling assumptions — and never above the documented maximum.
for (let rpm = 0; rpm <= 255; rpm++) {
    check(multiplier(rpm) <= 1320, `${rpm} rpm exceeded the 132% ceiling`);
    check(multiplier(rpm) >= 820, `${rpm} rpm dropped below the 82% floor`);
}

console.log(`worst step between neighbouring rpm: ${worstStep / 10}%`);
console.log(failures === 0 ? 'FW-057 cadence compensation: PASS' : `FW-057 cadence compensation: ${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
