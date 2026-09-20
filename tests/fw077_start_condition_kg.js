// FW-077/151 host regression: the Start condition is configured in kg and STORED in the
// frozen control domain. This guard is the source-text half of the contract that a
// re-measurement of the kg table cannot move a start threshold; the executable half is
// tests/host/torque/torque_threshold_migration_host.c.
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
const pipelineC = fs.readFileSync(path.join(root, 'src', 'assist_pipeline.c'), 'utf8');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
};
const constant = (source, name) => {
    const match = source.match(new RegExp(`#define\\s+${name}\\s+(\\d+)U`));
    if (!match) throw new Error(`missing ${name}`);
    return Number(match[1]);
};

// FW-151: v1..v6 thresholds migrate on the FROZEN CONTROL characteristic, a two-segment
// curve through the break and high points. The kg table (TORQUE_CURVE_P*) is deliberately NOT
// read here - keeping the two apart is the entire point of the separation.
const curve = [
    [constant(torqueH, 'TORQUE_CTRL_BREAK_NATIVE'), constant(torqueH, 'TORQUE_CTRL_BREAK_CLU')],
    [constant(torqueH, 'TORQUE_CTRL_HIGH_NATIVE'), constant(torqueH, 'TORQUE_CTRL_HIGH_CLU')],
];
const maxCtrl = constant(torqueH, 'TORQUE_CTRL_MAX_CLU');
const wireStep = constant(assistH, 'ASSIST_START_LOAD_WIRE_STEP_CTRL');

// Faithful port of the default native-delta -> CLU conversion used during v1..v6 migration.
// User calibration is a gain on this curve, so the migration invariant is the same: convert
// once, then compare in the calibrated domain.
function nativeToCtrl(delta) {
    const lerp = (x, x0, x1, y0, y1) =>
        y0 + Math.floor(((x - x0) * (y1 - y0) + (x1 - x0) / 2) / (x1 - x0));
    let load;
    if (delta <= curve[0][0]) {
        load = lerp(delta, 0, curve[0][0], 0, curve[0][1]);
    } else {
        // The second segment also carries the extrapolation above the table.
        load = lerp(delta, curve[0][0], curve[1][0], curve[0][1], curve[1][1]);
    }
    return Math.min(load, maxCtrl);
}

// FW-151: quantization to the stored wire step, in the control domain.
const roundToWireStep = (ctrl) =>
    Math.floor((ctrl + wireStep / 2) / wireStep) * wireStep;

// FW-084 grew the record to 48 B and the current version to v8. What FW-077 still owns is
// the v7 GEOMETRY: a v7 blob must keep being accepted at exactly 46 B, or every profile
// saved before FW-084 is rejected on load and silently replaced by defaults.
check(constant(assistC, 'BANK_RECORD_LEN_V7') === 46, 'v7 record must remain 46 B');
check(/version == BANK_BLOB_VERSION_V7 && record_len != BANK_RECORD_LEN_V7/.test(assistC),
    'a v7 blob must still be required to be exactly 46 B per record');
check(/record_len >= BANK_RECORD_LEN_V7/.test(assistC),
    'the FW-068/069/077 tail fields must stay gated on the v7 length, not the newest one');
check(wireStep === 10, 'start-load fields must keep their 10-unit wire step');

for (const oldName of ['without_rotation_threshold_mv', 'start_load_reduction_mv',
    'start_rise_mv', 'start_rise_centikg', 'start_rise_window_ms']) {
    check(!assistC.includes(oldName) && !assistH.includes(oldName) && !rideC.includes(oldName),
        `${oldName} must not remain in live firmware code`);
}
for (const newName of ['minimum_pedal_load_ctrl', 'riding_start_load_ctrl']) {
    check(assistC.includes(newName) && assistH.includes(newName),
        `${newName} must be present in config and serialization`);
}
// FW-151: and the kilogram-domain names must be GONE from the start-load and permission path.
// Their presence was the defect: a control threshold stored in a unit whose definition is a
// measurement, so improving the measurement retuned the bike.
for (const goneName of ['minimum_pedal_load_centikg', 'riding_start_load_centikg',
    'engage_load_centikg', 'AP2_EFFORT_DEADBAND_CENTIKG']) {
    check(!assistC.includes(goneName) && !assistH.includes(goneName) &&
        !rideC.includes(goneName) && !pipelineC.includes(goneName),
        `${goneName} must not remain: control thresholds do not live in kilograms`);
}

// FW-151: the migration is stated as a SENSOR TRIP POINT, which is the only thing the rider
// can actually feel. A v6 bank that asked for 18 mV must keep asking for 18 mV, whatever a
// later kilogram measurement says. The previous version of this check asserted 2.7 kg and
// carried a note that a v6 bank therefore "migrates to a HARDER start threshold than its label
// ever implied" - documenting the defect instead of fixing it.
const legacyMinimumMv = 18;
const legacyReductionMv = 10;
const rollingMv = Math.max(0, legacyMinimumMv - legacyReductionMv);
const firstNativeReaching = (ctrl) => {
    for (let d = 0; d <= 4200; d++) { if (nativeToCtrl(d) >= ctrl) { return d; } }
    return 4200;
};
const standTrip = firstNativeReaching(roundToWireStep(nativeToCtrl(legacyMinimumMv)));
const rollTrip = firstNativeReaching(roundToWireStep(nativeToCtrl(rollingMv)));
// The bound is the WIRE FORMAT's own quantization, not a domain error: the stored field has a
// 10 CLU step and the curve runs at 4.1 CLU/mV over this range, so half a step is ~1.2 mV. That
// rounding existed in every version of this format. What the control domain removes is the
// 3.4x shift that came from re-measuring the kilogram table - a different order of magnitude.
const TRIP_QUANTIZATION_MV = 2;
check(Math.abs(standTrip - legacyMinimumMv) <= TRIP_QUANTIZATION_MV,
    `a legacy ${legacyMinimumMv} mV standing threshold must still trip within ` +
    `${TRIP_QUANTIZATION_MV} mV of ${legacyMinimumMv} mV, got ${standTrip} mV`);
check(Math.abs(rollTrip - rollingMv) <= TRIP_QUANTIZATION_MV,
    `a legacy ${rollingMv} mV rolling threshold must still trip within ` +
    `${TRIP_QUANTIZATION_MV} mV of ${rollingMv} mV, got ${rollTrip} mV`);
// Both stored fields quantize to the wire step; the maximum error is half a step.
for (let ctrl = 0; ctrl <= 2250; ctrl++) {
    const wire = Math.floor((ctrl + wireStep / 2) / wireStep);
    const restored = wire * wireStep;
    check(Math.abs(restored - ctrl) <= 5,
        `${ctrl} CLU round-trip error exceeded half a wire step`);
}
check(assistC.includes('put_u16(&record[19], round_start_load_ctrl('),
    'the u16 minimum-load field must be serialized in the control domain');

// Migration has to run after the persisted sensor calibration is restored.
check(mainC.indexOf('torque_input_restore_persist(') < mainC.indexOf('assist_modes_apply_bank_blob('),
    'torque calibration must be restored before old bank thresholds are migrated');

// The rolling setting is a direct threshold, never another subtraction.
// The rolling threshold is selected by the ONE pipeline owner and is a DIRECT value, never
// another subtraction the rider would have to do in their head.
check(pipelineC.includes('level->riding_start_load_ctrl : level->minimum_pedal_load_ctrl'),
    'rolling start must use the direct control-domain threshold');
check(!pipelineC.includes('engage_load_ctrl -'),
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
    ? 'FW-077/151 Start condition (kg in, control domain stored): PASS'
    : `FW-077/151 Start condition: ${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
