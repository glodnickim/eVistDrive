// FW-093 host test: zero torque must become a real Hi-Z coast, not electrical damping.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw093_power_stage_coast.js
//
// Before this card the bridge was released only after uint16_half_rotation_counter had aged
// past POWER_STAGE_STOP_TICKS (~3 s without a Hall half-rotation). The Hall ISR resets that
// counter on every half rotation, so for as long as the motor turned the half bridges stayed
// enabled with MS.i_q_setpoint == 0 — and a FOC regulating the MEASURED current to zero is a
// damper, not a coast. This test pins the three rules that separate the concepts:
//   * a turning rotor must NEVER be what keeps the bridge driving at zero torque,
//   * the bridge must NEVER be released while torque is being asked for,
//   * the release must wait for the REAL current, with a bounded worst case.

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
const dyn = fs.readFileSync(P('src', 'assist_dynamics.c'), 'utf8');

const def = (name) => Number(cfg.match(new RegExp(`#define\\s+${name}\\s+(\\d+)`))[1]);
const COAST_CURRENT = def('POWER_STAGE_COAST_CURRENT');
const STABLE_TICKS = def('POWER_STAGE_COAST_STABLE_TICKS');
const MAX_WAIT_TICKS = def('POWER_STAGE_COAST_MAX_WAIT_TICKS');
const STOP_TICKS = def('POWER_STAGE_STOP_TICKS');

const COAST = 0, DRIVE = 1, WAIT = 2;

// --- port of the main-loop power-stage block + its two 4 kHz counters ---
// `legacy` selects the pre-FW-093 behaviour (release gated on the rotor-stopped counter)
// so the defect itself can be shown, not just the fix.
class PowerStage {
    constructor(legacy = false) {
        this.legacy = legacy;
        this.stage = COAST;
        this.pwmOn = false;          // ui_8_PWM_ON_Flag: FOC owns the phases
        this.moe = false;            // TIMER0 primary output: false = true Hi-Z
        this.zeroCurrentTicks = 0;
        this.zeroWaitTicks = 0;
        this.halfRotation = 0;       // uint16_half_rotation_counter
        this.enteredOnTimeout = false;
    }
    // reg_ADC_processing(), @4 kHz
    tick({ iq = 0, id = 0, hallEvent = false }) {
        if (Math.abs(iq) <= COAST_CURRENT && Math.abs(id) <= COAST_CURRENT) this.zeroCurrentTicks++;
        else this.zeroCurrentTicks = 0;
        this.zeroWaitTicks++;
        this.halfRotation = hallEvent ? 0 : this.halfRotation + 1;
    }
    // the main-loop block
    update({ iqSetpoint = 0, idSetpoint = 0 }) {
        if (iqSetpoint || idSetpoint) {
            this.zeroCurrentTicks = 0;
            this.zeroWaitTicks = 0;
            if (this.stage !== DRIVE) this.stage = DRIVE;
            if (!this.pwmOn) { this.pwmOn = true; this.moe = true; this.halfRotation = 0; }
            return;
        }
        if (this.legacy) {
            // pre-FW-093: zero torque alone changed nothing; only the rotor-stopped timer did
            if (this.halfRotation > STOP_TICKS && this.pwmOn) { this.pwmOn = false; this.moe = false; this.stage = COAST; }
            return;
        }
        if (!this.pwmOn) return;
        if (this.stage !== WAIT) { this.stage = WAIT; this.zeroCurrentTicks = 0; this.zeroWaitTicks = 0; }
        if (this.zeroCurrentTicks >= STABLE_TICKS || this.zeroWaitTicks >= MAX_WAIT_TICKS) {
            this.enteredOnTimeout = this.zeroCurrentTicks < STABLE_TICKS;
            this.pwmOn = false; this.moe = false; this.stage = COAST;
        }
    }
    // one control tick: counters first, then the decision (the order main.c runs them in)
    step(state) { this.tick(state); this.update(state); }
}

// Drive the stage until it reaches Hi-Z; returns the number of ticks, or -1 if it never does.
const ticksToCoast = (ps, state, limit = 40000) => {
    for (let t = 1; t <= limit; t++) { ps.step(state); if (!ps.moe) return t; }
    return -1;
};

const RIDING = { iqSetpoint: 300, iq: 300, id: 0, hallEvent: true };
// released: no torque asked for, current already decayed, rotor STILL TURNING
const RELEASED_SPINNING = { iqSetpoint: 0, iq: 0, id: 0, hallEvent: true };

// 1. The defect: with the rotor turning, the old gate never released the bridge.
{
    const ps = new PowerStage(true);
    ps.update(RIDING);
    check(ps.moe === true, '1. legacy: the bridge is driving while riding');
    check(ticksToCoast(ps, RELEASED_SPINNING, STOP_TICKS * 3) === -1,
        '1. legacy: zero torque with a turning rotor NEVER released the bridge');
}

// 2. The fix: the same situation reaches a real Hi-Z in a few ms.
{
    const ps = new PowerStage();
    ps.update(RIDING);
    const t = ticksToCoast(ps, RELEASED_SPINNING);
    check(t > 0 && t <= STABLE_TICKS + 2, `2. zero torque reaches Hi-Z in ${t} ticks (<= ~6 ms)`);
    check(ps.pwmOn === false && ps.stage === COAST, '2. FOC is stopped and the stage says COAST');
    check(ps.enteredOnTimeout === false, '2. it was the measured current that released it, not the timeout');
}

// 3. A turning rotor is not a reason to keep driving: with Hall events on every tick the
//    rotor-stopped counter never ages, and it must make no difference at all.
{
    const spinning = ticksToCoast(Object.assign(new PowerStage(), { pwmOn: true, moe: true, stage: DRIVE }),
        RELEASED_SPINNING);
    const stopped = ticksToCoast(Object.assign(new PowerStage(), { pwmOn: true, moe: true, stage: DRIVE }),
        { iqSetpoint: 0, iq: 0, id: 0, hallEvent: false });
    check(spinning === stopped, '3. the release takes the same time whether the rotor turns or not');
}

// 4. The bridge is never released while torque is being asked for, and a new request
//    cancels a pending coast in the same tick.
{
    const ps = new PowerStage();
    ps.update(RIDING);
    for (let t = 0; t < STABLE_TICKS - 1; t++) ps.step(RELEASED_SPINNING);   // coast pending
    check(ps.stage === WAIT && ps.moe === true, '4. a pending coast has not released the bridge yet');
    ps.step(RIDING);
    check(ps.stage === DRIVE && ps.zeroCurrentTicks === 0,
        '4. a fresh torque request cancels the pending coast immediately');
    for (let t = 0; t < 1000; t++) { ps.step(RIDING); check(ps.moe === true, '4. never Hi-Z under torque'); }
}

// 5. Real current, not the target, decides. A target of zero whose current has NOT decayed
//    must hold the bridge — but only up to the bounded worst case. (+1: the tick that makes
//    the DRIVE -> ZERO_CURRENT_WAIT transition re-zeroes both counters, so the wait is
//    measured from the tick after it.)
const WAIT_CEILING = MAX_WAIT_TICKS + 1;
{
    const stuck = { iqSetpoint: 0, iq: COAST_CURRENT + 1, id: 0, hallEvent: true };
    const ps = new PowerStage();
    ps.update(RIDING);
    const t = ticksToCoast(ps, stuck);
    check(t === WAIT_CEILING, `5. an undecayed current holds the bridge, but only ${t} ticks (~50 ms)`);
    check(ps.enteredOnTimeout === true, '5. and the timeout path is flagged as such for the log');
}

// 6. Id counts too: a zero i_q with a live i_d is still current in the windings.
{
    const ps = new PowerStage();
    ps.update(RIDING);
    const t = ticksToCoast(ps, { iqSetpoint: 0, iq: 0, id: COAST_CURRENT + 1, hallEvent: true });
    check(t === WAIT_CEILING, '6. a live i_d alone also prevents the measured-zero release');
}

// 7. Ordinary release: current decays over a few ticks, then the bridge goes.
{
    const ps = new PowerStage();
    ps.update(RIDING);
    let iq = 300;
    let t = 0;
    for (; t < 1000 && ps.moe; t++) { iq = Math.max(0, iq - 20); ps.step({ iqSetpoint: 0, iq, id: 0, hallEvent: true }); }
    check(ps.moe === false && t < WAIT_CEILING,
        `7. a decaying current releases on measurement after ${t} ticks, well inside the ceiling`);
}

// 8. A d-axis request is a current request too: motor_core carries iq_target AND id_target,
//    and current on the d axis is current in the windings. A bridge must never be released
//    while either axis is asking for it.
{
    const ps = new PowerStage();
    ps.update({ iqSetpoint: 0, idSetpoint: 200 });
    check(ps.moe === true && ps.stage === DRIVE, '8. a d-axis-only request drives the bridge');
    for (let t = 0; t < 500; t++) {
        ps.step({ iqSetpoint: 0, idSetpoint: 200, iq: 0, id: 0, hallEvent: true });
        if (!ps.moe) break;
    }
    check(ps.moe === true, '8. and it is never released while that request stands');
}

// 9. The current loop must keep its integral while the bridge still drives. Wiping it at the
//    control rate leaves a proportional-only loop, and a proportional-only loop cannot
//    synthesize the back-EMF needed to hold zero current on a spinning rotor — so the
//    measured current never enters the coast window and the latched u_q is not the back-EMF.
{
    const P_GAIN = Number(fs.readFileSync(P('inc', 'config.h'), 'utf8').match(/#define\s+P_FACTOR_I_Q\s+([\d.]+)/)[1]);
    const I_GAIN = Number(fs.readFileSync(P('inc', 'config.h'), 'utf8').match(/#define\s+I_FACTOR_I_Q\s+([\d.]+)/)[1]);
    // FOC runs at 16 kHz, the reset ran at 4 kHz -> at most 4 accumulations survived
    const survivingShare = (4 * I_GAIN) / P_GAIN;
    check(survivingShare < 0.05,
        `9. a 4 kHz reset left the integral at only ${(survivingShare * 100).toFixed(1)} % of the P term`);
    // and the resets themselves must be gone from the driving path
    const afterRide = main.match(/ride_control_update\(&ride_input\);[\s\S]{0,2600}/)[0];
    check(!/if\(MS\.i_q_setpoint==0\)\{\s*\n\s*PI_iq\.integral_part=0/.test(afterRide),
        '9. the FW-028 zero-target integral reset no longer runs while the bridge drives');
    check(!/if \(!MS\.i_q_setpoint\)\{[\s\S]{0,80}PI_iq\.integral_part=0/.test(main),
        '9. neither does the second one in the 1 s-without-torque cleanup');
    check(!/if\(!MS\.i_q_setpoint_temp&&PI_iq\.integral_part\)/.test(main),
        '9. nor the monolith one that Walk Assist runs through');
    // the ONLY zero-request reset left must be the one that accompanies a bridge release
    const coastBody9 = main.match(/void power_stage_enter_coast\(void\)\s*\n\{[\s\S]*?\n\}/)[0];
    check(/PI_iq\.integral_part=0/.test(coastBody9) &&
          coastBody9.indexOf('coast_u_q_latched') < coastBody9.indexOf('PI_iq.integral_part=0'),
        '9. coast entry latches u_q FIRST and only then clears the regulators');
}

// 10. Structural: the three concepts must stay three separate things in the source.
{
    check(/POWER_STAGE_COAST_CURRENT/.test(main) && /POWER_STAGE_COAST_STABLE_TICKS/.test(main),
        '10. the coast decision is made on the measured current');
    check(/void power_stage_enter_coast\(void\)/.test(main) && /void power_stage_enter_drive\(void\)/.test(main),
        '10. DRIVE and COAST are one shared path, not per-module shutdowns');
    // the release condition must not mention the rotor-stopped counter at all
    const releaseBlock = main.match(/else if\(ui_8_PWM_ON_Flag\)\{[\s\S]*?\n            \}/);
    check(releaseBlock !== null, '10. the zero-torque release block is present');
    check(releaseBlock && !/uint16_half_rotation_counter/.test(releaseBlock[0]),
        '10. and it does NOT consult the rotor-stopped counter');
    // the DEFINITION, not the prototype near the top of the file
    const coastBody = main.match(/void power_stage_enter_coast\(void\)\s*\n\{[\s\S]*?\n\}/)[0];
    check(/timer_primary_output_config\(TIMER0,DISABLE\)/.test(coastBody),
        '10. entering coast really clears MOE (Hi-Z), not just the Iq target');
    // the coast path must not run through the _T/2 soft-cutoff window
    check(!/pwm_cutoff_active=1/.test(coastBody),
        '10. the coast path does not arm the 50 % PWM fade (that vector is a brake, not Hi-Z)');
    const driveBody = main.match(/void power_stage_enter_drive\(void\)\s*\n\{[\s\S]*?\n\}/)[0];
    check(/pwm_enable_request=1/.test(driveBody) && !/timer_primary_output_config\(TIMER0,ENABLE\)/.test(driveBody),
        '10. MOE-on is handed to the FOC ISR, so the first driven period is not the neutral vector');
    check(/coast_u_q_latched/.test(coastBody) && /coast_u_q_latched/.test(driveBody),
        '10. the back-EMF is latched at coast entry and pre-loaded at the next bridge-on');
    check(/FW-093/.test(dyn),
        '10. FW-048 coast_release points at the power-stage owner instead of implying one');
}

console.log(failures === 0 ? '\nAll FW-093 checks passed.' : `\n${failures} FW-093 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
