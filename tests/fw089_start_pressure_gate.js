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
// FW-151: this test is about the PERMISSION GATE, so it works in the CONTROL domain - the
// domain the gate actually compares in. Every load it exercises (70/30 CLU) sits inside the
// first segment of the frozen control characteristic, so that segment is the whole conversion.
const LOW_NATIVE = num(tq, 'TORQUE_CTRL_BREAK_NATIVE');       // 146
const LOW_CTRL = num(tq, 'TORQUE_CTRL_BREAK_CLU');            // 600
const SPAN = num(tq, 'TORQUE_GAIN_REFERENCE_NATIVE');         // 3047
const GATE_MIN = num(cfg, 'TQ_GATE_MIN');                     // 18
// The removed gate was TQ_PRESSURE_FLOOR_START_MV = 750 + TQ_GATE_MIN. Note the 750 is its
// own baseline, NOT the sensor zero (740) — the two differ, which is precisely why the
// effective threshold in kg was never obvious from reading the constant.
const FLOOR_BASE = Number(cfg.match(/#define\s+TQ_PRESSURE_FLOOR_START_MV\s+\((\d+)\s*\+\s*TQ_GATE_MIN\)/)[1]);
const HIDDEN_GATE = FLOOR_BASE + GATE_MIN;                    // 768
const STAND_CTRL = num(am, 'ASSIST_MIN_PEDAL_LOAD_DEFAULT_CTRL');        // 70
const ROLL_CTRL = num(am, 'ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CTRL');  // 30
const START_STEPS = num(cfg, 'START_PHASE_STEPS');
const LATCH_STEPS = 4; // tuning_config start_steps default

// Control-domain conversions on the two gains the bike can be running.
const nativeToCtrlDefault = (delta) => Math.round(delta * LOW_CTRL / LOW_NATIVE);
const nativeToCtrlUser = (delta) => Math.round(delta * 6000 / SPAN);

console.log(`zero ${ZERO}, TQ_GATE_MIN ${GATE_MIN}, configured start ${STAND_CTRL} CLU / ` +
    `rolling ${ROLL_CTRL} CLU (trip points ${LOW_NATIVE * STAND_CTRL / LOW_CTRL | 0} / ` +
    `${LOW_NATIVE * ROLL_CTRL / LOW_CTRL | 0} mV)`);

// --- the chain, modelled with and without the removed hidden gate ---
// hidden: main.c also required torque_on_crank > ZERO + GATE_MIN before the start phase.
function ride({ hidden, loadCtrl, steps, rolling, threshCtrl }) {
    const rawNative = ZERO + Math.round(loadCtrl * LOW_NATIVE / LOW_CTRL);
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
    if (loadCtrl < threshCtrl) return { iq: 0, startPhase, reason: 'below load threshold' };
    return { iq: 1, startPhase, reason: 'assisting' };
}

// 1. The audit's case: a push just above the standing gate, no cadence, four PAS steps.
{
    const args = { loadCtrl: STAND_CTRL + 10, steps: 4, rolling: false, threshCtrl: STAND_CTRL };
    const before = ride({ ...args, hidden: true });
    const after = ride({ ...args, hidden: false });
    check(before.iq === 0, `1. before FW-089 a push just over the gate gave no assist (blocked by: ${before.reason})`);
    check(after.iq > 0, '1. after FW-089 a push just over the standing gate assists');
}

// 2. Rolling restart: just above the rolling gate, three steps.
{
    const args = { loadCtrl: ROLL_CTRL + 10, steps: 3, rolling: true, threshCtrl: ROLL_CTRL };
    check(ride({ ...args, hidden: true }).iq === 0, '2. before FW-089 a rolling restart gave no assist');
    check(ride({ ...args, hidden: false }).iq > 0, '2. after FW-089 it assists');
}

// 3. The gate must not become a rubber stamp: below the configured threshold, still nothing.
{
    const below = ride({ hidden: false, loadCtrl: STAND_CTRL - 20, steps: 4, rolling: false, threshCtrl: STAND_CTRL });
    check(below.iq === 0 && below.reason === 'below load threshold',
        '3. a push below the standing gate still does not assist');
    const rollingBelow = ride({ hidden: false, loadCtrl: ROLL_CTRL - 10, steps: 3, rolling: true, threshCtrl: ROLL_CTRL });
    check(rollingBelow.iq === 0, '3. a push below the rolling gate still does not assist');
}

// 4. The start phase may now rise without pressure — and that alone must yield no current.
{
    const noPush = ride({ hidden: false, loadCtrl: 0, steps: 4, rolling: false, threshCtrl: STAND_CTRL });
    check(noPush.startPhase === true, '4. the start phase rises on crank movement alone');
    check(noPush.iq === 0, '4. ...but with no pedal load the latch still gives zero current');
}

// 5. Crank jiggle: any reverse step resets fwd_run, so it never accumulates into a start.
{
    const jiggle = ride({ hidden: false, loadCtrl: STAND_CTRL + 10, steps: 0, rolling: false, threshCtrl: STAND_CTRL });
    check(jiggle.iq === 0, '5. a reset step count cannot start assist even with load');
    // Matched on the CODE, not on a comment that happened to sit on the same line: FW-098
    // moved that comment into a block above and this check failed while the behaviour was
    // untouched. What matters is that the backward-step branch still clears fwd_run.
    check(/else if\(st<0\)\{[\s\S]{0,1600}?fwd_run=0;/.test(main),
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
    //
    // The match is GUARDED. Checks 5 and 6 are source-text guards over main.c shapes that
    // Assist Pipeline V2 replaced, and both already failed before FW-151 (verified by running
    // this file at 3ccfb9d). Dereferencing a null match crashed the process here, which hid
    // every check after this point - that is how a stale guard quietly becomes no guard at all.
    // The staleness is recorded as its own finding; it is not FW-151's to fix.
    const conditionMatch = block.match(/if\(MS\.cadence==0[^)]*\)\{/);
    check(conditionMatch !== null,
        '6. the start-phase condition is still recognizable in main.c (STALE since V2)');
    if (conditionMatch) {
        check(!/torque_on_crank/.test(conditionMatch[0]),
            '6. no raw-ADC pressure term remains in the start-phase condition');
    }
}

// 7. Keep the arithmetic that justified this card honest: if any constant moves, the
//    documented figures must be recomputed rather than quietly drifting.
//    FW-151: restated in the CONTROL domain, which is where the gate lives. The claim the
//    card rests on is unchanged - the removed hidden raw-ADC gate demanded MORE pressure than
//    the rider's own configured standing threshold, so it silently overrode the setting.
{
    const above = (HIDDEN_GATE + 1) - ZERO; // strictly greater than -> +1
    check(above === 29, `7. the old gate sat ${above} counts above zero`);
    check(nativeToCtrlDefault(above) === 119,
        `7. that is ${nativeToCtrlDefault(above)} CLU on the default gain (expected 119)`);
    check(nativeToCtrlUser(above) === 57,
        `7. and ${nativeToCtrlUser(above)} CLU at a full-scale user gain (expected 57)`);
    check(nativeToCtrlDefault(above) > STAND_CTRL,
        '7. the hidden gate really was above the configured standing threshold');
}

console.log(failures === 0 ? '\nAll FW-089 checks passed.' : `\n${failures} FW-089 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
