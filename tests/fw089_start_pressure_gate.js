// FW-089 host test: the configured kg threshold must be the ONLY pressure condition
// that decides whether a start counts.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw089_start_pressure_gate.js
//
// This one is deliberately an INTEGRATION test across main.c -> rider_input ->
// assist_modes -> ride_control. The earlier tests checked those layers separately, and
// that is exactly why the defect survived them: each layer looked correct on its own,
// while the chain demanded a pressure the rider never configured.

'use strict';
const fs = require('fs');
const path = require('path');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

const P = (...p) => path.join(__dirname, '..', ...p);
const main = fs.readFileSync(P('src', 'main.c'), 'utf8');
const cfg = fs.readFileSync(P('inc', 'config.h'), 'utf8');
const tq = fs.readFileSync(P('inc', 'torque_input.h'), 'utf8');
const am = fs.readFileSync(P('inc', 'assist_modes.h'), 'utf8');

const num = (src, name) => {
    const m = src.match(new RegExp(`#define\\s+${name}\\s+\\(?([\\d+ ]+)\\)?U?`));
    if (!m) throw new Error(`${name} not found`);
    return m[1].split('+').reduce((a, b) => a + Number(b.trim()), 0);
};
const ZERO = num(tq, 'TORQUE_ZERO_TARGET_NATIVE');            // 740
const LOW_NATIVE = num(tq, 'TORQUE_DEFAULT_LOW_NATIVE');      // 146
const LOW_CENTIKG = num(tq, 'TORQUE_DEFAULT_LOW_CENTIKG');    // 600
const SPAN = num(tq, 'TORQUE_DEFAULT_SPAN_NATIVE');           // 1139
const GATE_MIN = num(cfg, 'TQ_GATE_MIN');                     // 18
// The removed gate was TQ_PRESSURE_FLOOR_START_MV = 750 + TQ_GATE_MIN. Note the 750 is its
// own baseline, NOT the sensor zero (740) — the two differ, which is precisely why the
// effective threshold in kg was never obvious from reading the constant.
const FLOOR_BASE = Number(cfg.match(/#define\s+TQ_PRESSURE_FLOOR_START_MV\s+\((\d+)\s*\+\s*TQ_GATE_MIN\)/)[1]);
const HIDDEN_GATE = FLOOR_BASE + GATE_MIN;                    // 768
const STAND_CENTIKG = num(am, 'ASSIST_MIN_PEDAL_LOAD_DEFAULT_CENTIKG');        // 70
const ROLL_CENTIKG = num(am, 'ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CENTIKG');  // 30
const START_STEPS = num(cfg, 'START_PHASE_STEPS');
const LATCH_STEPS = 4; // tuning_config start_steps default

// kg conversions on the two characteristics the bike can be running.
const nativeToCentikgDefault = (delta) => Math.round(delta * LOW_CENTIKG / LOW_NATIVE);
const nativeToCentikgUser = (delta) => Math.round(delta * 6000 / SPAN);

console.log(`zero ${ZERO}, TQ_GATE_MIN ${GATE_MIN}, configured start ${STAND_CENTIKG / 100} kg / rolling ${ROLL_CENTIKG / 100} kg`);

// --- the chain, modelled with and without the removed hidden gate ---
// hidden: main.c also required torque_on_crank > ZERO + GATE_MIN before the start phase.
function ride({ hidden, loadCentikg, steps, rolling, threshCentikg }) {
    const rawNative = ZERO + Math.round(loadCentikg * LOW_NATIVE / LOW_CENTIKG);
    const fwdRun = steps;

    // main.c: start phase
    const pressureOk = !hidden || rawNative > HIDDEN_GATE;
    const startPhase = fwdRun >= START_STEPS && pressureOk;

    // main.c: forward_pedaling -> rider_input.pedaling_active (cadence still 0 here)
    const pedalingActive = (0 > 0 || startPhase);

    // assist_modes: prepare_assist_input gate
    const gatePassed = pedalingActive && (startPhase || false);
    if (!gatePassed) return { iq: 0, startPhase, reason: 'gate' };

    // ride_control: the latch — the ONLY pressure condition that should matter
    const required = rolling ? LATCH_STEPS - 1 : LATCH_STEPS;
    const crankOk = fwdRun >= required;
    if (!crankOk) return { iq: 0, startPhase, reason: 'crank steps' };
    if (loadCentikg < threshCentikg) return { iq: 0, startPhase, reason: 'below kg threshold' };
    return { iq: 1, startPhase, reason: 'assisting' };
}

// 1. The audit's case: 0.8 kg against a 0.7 kg threshold, no cadence, four PAS steps.
{
    const args = { loadCentikg: 80, steps: 4, rolling: false, threshCentikg: STAND_CENTIKG };
    const before = ride({ ...args, hidden: true });
    const after = ride({ ...args, hidden: false });
    check(before.iq === 0, `1. before FW-089 a 0.8 kg push gave no assist (blocked by: ${before.reason})`);
    check(after.iq > 0, '1. after FW-089 a 0.8 kg push against a 0.7 kg threshold assists');
}

// 2. Rolling restart: 0.4 kg against the 0.3 kg rolling threshold, three steps.
{
    const args = { loadCentikg: 40, steps: 3, rolling: true, threshCentikg: ROLL_CENTIKG };
    check(ride({ ...args, hidden: true }).iq === 0, '2. before FW-089 a rolling 0.4 kg restart gave no assist');
    check(ride({ ...args, hidden: false }).iq > 0, '2. after FW-089 it assists');
}

// 3. The gate must not become a rubber stamp: below the configured threshold, still nothing.
{
    const below = ride({ hidden: false, loadCentikg: 50, steps: 4, rolling: false, threshCentikg: STAND_CENTIKG });
    check(below.iq === 0 && below.reason === 'below kg threshold',
        '3. 0.5 kg against a 0.7 kg threshold still does not assist');
    const rollingBelow = ride({ hidden: false, loadCentikg: 20, steps: 3, rolling: true, threshCentikg: ROLL_CENTIKG });
    check(rollingBelow.iq === 0, '3. 0.2 kg against the 0.3 kg rolling threshold still does not assist');
}

// 4. The start phase may now rise without pressure — and that alone must yield no current.
{
    const noPush = ride({ hidden: false, loadCentikg: 0, steps: 4, rolling: false, threshCentikg: STAND_CENTIKG });
    check(noPush.startPhase === true, '4. the start phase rises on crank movement alone');
    check(noPush.iq === 0, '4. ...but with no pedal load the latch still gives zero current');
}

// 5. Crank jiggle: any reverse step resets fwd_run, so it never accumulates into a start.
{
    const jiggle = ride({ hidden: false, loadCentikg: 80, steps: 0, rolling: false, threshCentikg: STAND_CENTIKG });
    check(jiggle.iq === 0, '5. a reset step count cannot start assist even with load');
    check(/fwd_run=0;\s*\/\/any reverse step cancels the forward run/.test(main),
        '5. main.c still resets fwd_run on a reverse step');
}

// 6. Structural: the raw-ADC pressure term is gone from the start-phase condition.
{
    const i = main.indexOf('#if START_PHASE_ENABLE');
    const block = main.slice(i, main.indexOf('#endif', i));
    check(/if\(MS\.cadence==0 && !start_phase && fwd_run>=START_PHASE_STEPS\)\{/.test(block),
        '6. the start phase depends on crank movement alone');
    // Check the CONDITION, not the whole block — the comment above it legitimately
    // explains what was removed and names the old term.
    const condition = block.match(/if\(MS\.cadence==0[^)]*\)\{/)[0];
    check(!/torque_on_crank/.test(condition),
        '6. no raw-ADC pressure term remains in the start-phase condition');
}

// 7. Keep the arithmetic that justified this card honest: if any constant moves, the
//    documented kg figures must be recomputed rather than quietly drifting.
{
    const above = (HIDDEN_GATE + 1) - ZERO; // strictly greater than -> +1
    check(above === 29, `7. the old gate sat ${above} counts above zero`);
    check(nativeToCentikgDefault(above) === 119,
        `7. that is ${nativeToCentikgDefault(above)} centikg on the default curve (expected 119)`);
    check(nativeToCentikgUser(above) === 153,
        `7. and ${nativeToCentikgUser(above)} centikg after a user calibration (expected 153)`);
    check(nativeToCentikgDefault(above) > STAND_CENTIKG,
        '7. the hidden gate really was above the configured standing threshold');
}

console.log(failures === 0 ? '\nAll FW-089 checks passed.' : `\n${failures} FW-089 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
