// FW-077 host regression: all public Start condition loads use kg.
// Run from BAFANG_GD32F303RCT6/: node tests/fw077_start_condition_kg.js

'use strict';
const fs = require('fs');
const path = require('path');

const root = path.join(__dirname, '..');
const assistC = fs.readFileSync(path.join(root, 'src', 'assist_modes.c'), 'utf8');
const assistH = fs.readFileSync(path.join(root, 'inc', 'assist_modes.h'), 'utf8');
const rideC = fs.readFileSync(path.join(root, 'src', 'ride_control.c'), 'utf8');
const mainC = fs.readFileSync(path.join(root, 'src', 'main.c'), 'utf8');
const torqueH = fs.readFileSync(path.join(root, 'inc', 'torque_input.h'), 'utf8');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
};
const constant = (source, name) => {
    const match = source.match(new RegExp(`#define\\s+${name}\\s+(\\d+)U`));
    if (!match) throw new Error(`missing ${name}`);
    return Number(match[1]);
};

// FW-150: the default characteristic is a three-point table, not two points.
const curve = [
    [constant(torqueH, 'TORQUE_CURVE_P1_NATIVE'), constant(torqueH, 'TORQUE_CURVE_P1_CENTIKG')],
    [constant(torqueH, 'TORQUE_CURVE_P2_NATIVE'), constant(torqueH, 'TORQUE_CURVE_P2_CENTIKG')],
    [constant(torqueH, 'TORQUE_CURVE_P3_NATIVE'), constant(torqueH, 'TORQUE_CURVE_P3_CENTIKG')],
];
const maxCentikg = constant(torqueH, 'TORQUE_INPUT_MAX_CENTIKG');
const wireStep = constant(assistH, 'ASSIST_START_LOAD_WIRE_STEP_CENTIKG');

// Faithful port of the default native-delta -> centikg conversion used during
// v1..v6 migration. User calibration is a gain on this curve, so the migration
// invariant is the same: convert once, then compare in the calibrated domain.
function nativeToCentikg(delta) {
    const lerp = (x, x0, x1, y0, y1) =>
        y0 + Math.floor(((x - x0) * (y1 - y0) + (x1 - x0) / 2) / (x1 - x0));
    let load;
    if (delta <= curve[0][0]) {
        load = lerp(delta, 0, curve[0][0], 0, curve[0][1]);
    } else {
        // The last segment also carries the extrapolation above the table.
        let i = curve.length - 1;
        while (i > 1 && delta <= curve[i - 1][0]) { i--; }
        load = lerp(delta, curve[i - 1][0], curve[i][0], curve[i - 1][1], curve[i][1]);
    }
    return Math.min(load, maxCentikg);
}

const roundToDecikg = (centikg) =>
    Math.floor((centikg + wireStep / 2) / wireStep) * wireStep;

// FW-084 grew the record to 48 B and the current version to v8. What FW-077 still owns is
// the v7 GEOMETRY: a v7 blob must keep being accepted at exactly 46 B, or every profile
// saved before FW-084 is rejected on load and silently replaced by defaults.
check(constant(assistC, 'BANK_RECORD_LEN_V7') === 46, 'v7 record must remain 46 B');
check(/version == BANK_BLOB_VERSION_V7 && record_len != BANK_RECORD_LEN_V7/.test(assistC),
    'a v7 blob must still be required to be exactly 46 B per record');
check(/record_len >= BANK_RECORD_LEN_V7/.test(assistC),
    'the FW-068/069/077 tail fields must stay gated on the v7 length, not the newest one');
check(wireStep === 10, 'start-load fields must use 0.1 kg resolution');

for (const oldName of ['without_rotation_threshold_mv', 'start_load_reduction_mv',
    'start_rise_mv', 'start_rise_centikg', 'start_rise_window_ms']) {
    check(!assistC.includes(oldName) && !assistH.includes(oldName) && !rideC.includes(oldName),
        `${oldName} must not remain in live firmware code`);
}
for (const newName of ['minimum_pedal_load_centikg', 'riding_start_load_centikg']) {
    check(assistC.includes(newName) && assistH.includes(newName),
        `${newName} must be present in config and serialization`);
}

// Historical default: 18 mV. It read as 0.74 kg on the OLD two-point curve, and the
// expectation here was 0.7 kg. FW-150 replaced that curve with the 2026-09-17
// reference-weight measurement, on which 18 mV above zero is 2.7 kg of real force -
// so a v6 bank migrates to a HARDER start threshold than its label ever implied.
// The number below tracks the curve because the curve is a measurement; whether the
// migration should instead land on the configured 0.7 kg default is a product
// decision, and changing it belongs in assist_modes.c, not here.
check(roundToDecikg(nativeToCentikg(18)) === 270,
    `legacy 18 mV must migrate to 2.7 kg, got ${(roundToDecikg(nativeToCentikg(18)) / 100).toFixed(1)} kg`);

// v6 rolling threshold was (minimum - reduction). Verify the v7 direct value.
const legacyMinimumMv = 18;
const legacyReductionMv = 10;
const rollingMv = Math.max(0, legacyMinimumMv - legacyReductionMv);
check(roundToDecikg(nativeToCentikg(rollingMv)) === 120,
    `18-10 mV must migrate to direct rolling threshold 1.2 kg, got ${roundToDecikg(nativeToCentikg(rollingMv)) / 100}`);
// Both public fields round to 0.1 kg. The maximum error is 0.05 kg.
for (let centikg = 0; centikg <= 2250; centikg++) {
    const wire = Math.floor((centikg + wireStep / 2) / wireStep);
    const restored = wire * wireStep;
    check(Math.abs(restored - centikg) <= 5,
        `${centikg} ckg round-trip error exceeded 0.05 kg`);
}
check(assistC.includes('put_u16(&record[19], round_start_load_centikg('),
    'the u16 minimum-load field must also be serialized at 0.1 kg precision');

// Migration has to run after the persisted sensor calibration is restored.
check(mainC.indexOf('torque_input_restore_persist(') < mainC.indexOf('assist_modes_apply_bank_blob('),
    'torque calibration must be restored before old bank thresholds are migrated');

// The rolling setting is a direct threshold, never another subtraction.
check(rideC.includes('engage_threshold_centikg = level->riding_start_load_centikg'),
    'rolling start must use the direct kg threshold');
check(!rideC.includes('engage_threshold_centikg -'),
    'rolling start must not subtract a hidden reduction');
for (const removed of ['START_RISE_CONFIRM', 'start_window_open', 'rise_engaged']) {
    check(!rideC.includes(removed), `${removed} rise-detector code must be removed`);
}
// FW-084 spent the two bytes FW-077 reserved. What must hold now is that they are read as
// Extended Boost ONLY from v8 on — a v6/v7 blob still carries the dead rise-detector value
// there, and interpreting it would turn stale bytes into a live boost setting.
check(/version >= BANK_BLOB_VERSION_V8 && record_len >= BANK_RECORD_LEN_V8/.test(assistC),
    'record[36..37] may only be read as Extended Boost from v8 on');

console.log(failures === 0
    ? 'FW-077 Start condition kg: PASS'
    : `FW-077 Start condition kg: ${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
