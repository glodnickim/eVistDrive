// FW-084 design test: Extended Boost — the deliberate drive hold after the cranks stop.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw084_extended_boost.js
//
// WHAT THIS IS: a MODEL of the state machine, re-implemented in JavaScript, plus structural
// checks on the C sources. It proves the design behaves as the card says and that the wiring
// in ride_control.c is in the right order — it does NOT prove the compiled module does the
// same thing, and it cannot see an integer promotion or an uninitialized field at all.
//
// The shipped C is exercised by tests/host/fw084_extended_boost_host.c, which links against
// src/assist_extended_boost.c itself:  powershell -File tests/host/run-host-tests.ps1
// Keep the two in step; when they disagree, the C harness is the one that is right.
//
// The feature can hold current with the cranks stationary, so the tests that matter most
// here are the ones that say NO: a short spike must not arm it, a stale arming must not
// fire, the timer must not be refreshed by anything, and every cancel path must act in the
// same control tick. The current formula is checked against the plan's reference
// arithmetic, integer division and all.

'use strict';
const fs = require('fs');
const path = require('path');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

const read = (...p) => fs.readFileSync(path.join(__dirname, '..', ...p), 'utf8');
const boostH = read('inc', 'assist_extended_boost.h');
const boostC = read('src', 'assist_extended_boost.c');
const assistC = read('src', 'assist_modes.c');
const assistH = read('inc', 'assist_modes.h');
const rideC = read('src', 'ride_control.c');
const canC = read('src', 'CAN_Display.c');
const torqueH = read('inc', 'torque_input.h');

const constOf = (src, name) => {
    const m = src.match(new RegExp(`#define\\s+${name}\\s+(\\w+)`));
    if (!m) throw new Error(`${name} not found`);
    return Number(m[1].replace(/U$/, ''));
};

const TICKS_PER_MS = constOf(boostH, 'EXT_BOOST_CONTROL_TICKS_PER_MS');
const CONFIRM_MS = constOf(boostH, 'EXT_BOOST_CONFIRM_MS');
const HYST = constOf(boostH, 'EXT_BOOST_RELEASE_HYST_CENTIKG');
const ARM_TIMEOUT_MS = constOf(boostH, 'EXT_BOOST_ARM_TIMEOUT_MS');
const TRIG_MIN = constOf(boostH, 'ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG');
const TRIG_MAX = constOf(boostH, 'ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG');
const DUR_MAX = constOf(boostH, 'ASSIST_EXT_BOOST_DURATION_MAX_MS');
const FULL_SCALE = constOf(torqueH, 'TORQUE_PUBLIC_FULL_SCALE_CENTIKG');
const CONFIRM_TICKS = CONFIRM_MS * TICKS_PER_MS;
const ARM_TIMEOUT_TICKS = ARM_TIMEOUT_MS * TICKS_PER_MS;

const CANCEL = {
    NONE: 0, DISABLED: 1, SAFETY_CUT: 2, REVERSE: 3, SENSOR_INVALID: 4, WALK: 5,
    CALIBRATION: 6, LEVEL_OR_BANK: 7, MOTION_LOST: 8, PEDALING_RESUMED: 9,
    ARM_TIMEOUT: 10, COMPLETED: 11
};
const IDLE = 0, QUALIFY = 1, ARMED = 2, ACTIVE = 3;

// --- faithful port of src/assist_extended_boost.c ---
class Boost {
    constructor() {
        this.clear();
        this.prevPedaling = false;
        this.armExpired = false;
        this.cancelReason = CANCEL.NONE;
        this.lastBank = 0; this.lastLevel = 0; this.haveContext = false;
    }
    clear() {
        this.state = IDLE;
        this.candidatePeak = 0; this.armedPeak = 0;
        this.confirmTicks = 0; this.armIdleTicks = 0; this.activeTicksLeft = 0;
        this.windowOpen = false; this.boostIq = 0;
    }
    reset(reason) { this.clear(); if (reason !== CANCEL.NONE) this.cancelReason = reason; }

    static trigger(v) { return Math.min(Math.max(v, TRIG_MIN), TRIG_MAX); }
    static duration(v) { return Math.min(v, DUR_MAX); }
    static iq(peak, trigger, strengthPct, iqLimit) {
        if (iqLimit <= 0 || strengthPct === 0) return 0;
        const p = Math.min(peak, FULL_SCALE);
        if (p <= trigger) return 0;
        const span = FULL_SCALE - trigger;
        const above = p - trigger;
        const base = Math.floor((iqLimit * above + Math.floor(span / 2)) / span);
        return Math.min(Math.floor((base * strengthPct + 50) / 100), iqLimit);
    }

    qualify(inp, trigger) {
        const mayQualify = inp.pedaling && inp.latched;
        const load = inp.load;
        if (mayQualify && load >= trigger) {
            if (!this.windowOpen) {
                this.windowOpen = true;
                this.confirmTicks = 0;
                this.candidatePeak = 0;
                this.armExpired = false;
                if (this.state === IDLE) this.state = QUALIFY;
            }
            if (load > this.candidatePeak) this.candidatePeak = load;
            if (this.confirmTicks < CONFIRM_TICKS) this.confirmTicks++;
            if (this.confirmTicks >= CONFIRM_TICKS) {
                this.armedPeak = this.candidatePeak;
                this.state = ARMED;
                this.armIdleTicks = 0;
            }
            return;
        }
        if (this.windowOpen) {
            if (!mayQualify || (load + HYST) < trigger) {
                this.windowOpen = false;
                this.confirmTicks = 0;
                this.candidatePeak = 0;
                if (this.state === QUALIFY) this.state = IDLE;
            }
            return;
        }
        if (this.state === ARMED) {
            if (++this.armIdleTicks >= ARM_TIMEOUT_TICKS) {
                this.armExpired = true;
                this.reset(CANCEL.ARM_TIMEOUT);
            }
        }
    }

    update(inp, cfg) {
        const out = { iq: 0, hold: false, armed: false, active: false };
        const trigger = Boost.trigger(cfg.trigger);
        const duration = Boost.duration(cfg.duration);
        const disabled = duration === 0 || cfg.strength === 0;

        let cancel = CANCEL.NONE;
        if (disabled) cancel = CANCEL.DISABLED;
        else if (inp.safetyCut) cancel = CANCEL.SAFETY_CUT;
        else if (inp.reverse) cancel = CANCEL.REVERSE;
        else if (!inp.torqueValid || !inp.pasValid) cancel = CANCEL.SENSOR_INVALID;
        else if (inp.walk) cancel = CANCEL.WALK;
        else if (inp.calibration) cancel = CANCEL.CALIBRATION;
        else if (inp.level === 0) cancel = CANCEL.LEVEL_OR_BANK;
        else if (this.haveContext && (inp.bank !== this.lastBank || inp.level !== this.lastLevel))
            cancel = CANCEL.LEVEL_OR_BANK;
        else if (this.state === ACTIVE && !inp.motion) cancel = CANCEL.MOTION_LOST;
        else if (this.state === ACTIVE && inp.pedaling) cancel = CANCEL.PEDALING_RESUMED;

        this.lastBank = inp.bank; this.lastLevel = inp.level; this.haveContext = true;

        if (cancel !== CANCEL.NONE) {
            this.reset(cancel);
            this.prevPedaling = inp.pedaling;
            return out;
        }

        if (this.state !== ACTIVE) {
            this.qualify(inp, trigger);
            const edge = this.prevPedaling && !inp.pedaling;
            if (this.state === ARMED && edge && inp.motion) {
                const candidate = Boost.iq(this.armedPeak, trigger, cfg.strength, inp.iqLimit);
                if (candidate > 0) {
                    this.boostIq = candidate;
                    this.activeTicksLeft = duration * TICKS_PER_MS;
                    this.state = ACTIVE;
                    this.windowOpen = false;
                    this.confirmTicks = 0;
                    this.candidatePeak = 0;
                } else {
                    this.clear();
                }
            }
        }
        if (this.state === ACTIVE) {
            if (this.activeTicksLeft === 0) {
                this.reset(CANCEL.COMPLETED);
            } else {
                this.activeTicksLeft--;
                out.iq = this.boostIq; out.hold = true; out.active = true;
            }
        }
        out.armed = this.state === ARMED || this.state === ACTIVE;
        this.prevPedaling = inp.pedaling;
        return out;
    }
}

const IQ_LIMIT = 3000;
const CFG = (over = {}) => Object.assign(
    { trigger: 800, strength: 100, duration: 200 }, over);
const IN = (over = {}) => Object.assign({
    pedaling: true, latched: true, motion: true, safetyCut: false, walk: false,
    calibration: false, torqueValid: true, pasValid: true, reverse: false,
    bank: 0, level: 3, load: 0, iqLimit: IQ_LIMIT
}, over);

// Drives the module and returns how many ticks it reported ACTIVE.
const run = (b, cfg, ticks, inputFor) => {
    let activeTicks = 0;
    let peakIq = 0;
    for (let t = 0; t < ticks; t++) {
        const out = b.update(inputFor(t), cfg);
        if (out.active) { activeTicks++; peakIq = Math.max(peakIq, out.iq); }
    }
    return { activeTicks, peakIq };
};

// A push held above the threshold, then the cranks stop.
const pushThenStop = (b, cfg, opts = {}) => {
    const hold = opts.holdTicks !== undefined ? opts.holdTicks : CONFIRM_TICKS;
    const load = opts.load !== undefined ? opts.load : 2000;
    const after = opts.afterTicks !== undefined ? opts.afterTicks : 8000;
    const extra = opts.extra || {};
    return run(b, cfg, hold + after, (t) => t < hold
        ? IN(Object.assign({ load }, extra))
        : IN(Object.assign({ pedaling: false, latched: false, load: 0 }, extra)));
};

console.log(`confirm ${CONFIRM_MS} ms (${CONFIRM_TICKS} ticks), arm timeout ${ARM_TIMEOUT_MS} ms, ` +
    `hysteresis ${HYST / 100} kg, trigger ${TRIG_MIN / 100}-${TRIG_MAX / 100} kg`);

// 1. OFF by default: duration 0 must behave exactly like firmware without FW-084.
{
    const r = pushThenStop(new Boost(), CFG({ duration: 0 }));
    check(r.activeTicks === 0, `1. duration 0 never drives (${r.activeTicks} ticks)`);
    const s = pushThenStop(new Boost(), CFG({ strength: 0 }));
    check(s.activeTicks === 0, '1. strength 0 never drives either');
}

// 2. Load below the threshold is not a decision the rider made.
{
    const r = pushThenStop(new Boost(), CFG(), { load: 799 });
    check(r.activeTicks === 0, `2. below the trigger nothing arms (${r.activeTicks})`);
}

// 3. THE GLITCH GUARD: a big spike shorter than 30 ms is a chain slap, not a push.
{
    const r = pushThenStop(new Boost(), CFG(), { holdTicks: CONFIRM_TICKS - 1, load: 5000 });
    check(r.activeTicks === 0, `3. a ${CONFIRM_TICKS - 1}-tick spike does not arm (${r.activeTicks})`);
}

// 4. Exactly the required hold arms it.
{
    const b = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 2000 }), CFG());
    check(b.state === ARMED, `4. ${CONFIRM_TICKS} ticks above the trigger reaches ARMED (state ${b.state})`);
}

// 5. Rate of rise is irrelevant: only the held level counts. A steep ramp that never stays
//    above the trigger long enough must not arm, and a slow one that does must.
{
    const steep = new Boost();
    for (let t = 0; t < CONFIRM_TICKS - 1; t++) steep.update(IN({ load: 6000 }), CFG());
    check(steep.state !== ARMED, '5. a violent but short rise does not arm');
    const slow = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) slow.update(IN({ load: 810 + t }), CFG());
    check(slow.state === ARMED, '5. a gentle sustained load does arm');
}

// 6. The peak of the CURRENT qualifying window is what gets stored.
{
    const b = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: t === 10 ? 2500 : 1200 }), CFG());
    check(b.armedPeak === 2500, `6. the window peak is kept (${b.armedPeak})`);
}

// 7. A later confirmed window replaces the earlier one — even when it is WEAKER. What the
//    rider just did is what the boost reproduces; a monster push from 10 s ago is not.
{
    const b = new Boost();
    const cfg = CFG();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 4000 }), cfg);
    check(b.armedPeak === 4000, '7. first window armed at its peak');
    for (let t = 0; t < 200; t++) b.update(IN({ load: 0 }), cfg);          // window closes
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 1200 }), cfg);
    check(b.armedPeak === 1200, `7. the weaker later push replaces it (${b.armedPeak})`);
    // ...and the hysteresis is what ended the first window, without moving the threshold.
    const h = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) h.update(IN({ load: 4000 }), cfg);
    for (let t = 0; t < 50; t++) h.update(IN({ load: 800 - HYST + 1 }), cfg); // inside the hysteresis
    check(h.windowOpen, '7. a dip inside the hysteresis does not end the window');
}

// 8. An arming goes stale, and a stale one never fires.
{
    const b = new Boost();
    const cfg = CFG();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 4000 }), cfg);
    for (let t = 0; t < ARM_TIMEOUT_TICKS + 1; t++) b.update(IN({ load: 0 }), cfg);
    check(b.state === IDLE && b.cancelReason === CANCEL.ARM_TIMEOUT,
        `8. the arming expires after ${ARM_TIMEOUT_MS} ms (state ${b.state}, reason ${b.cancelReason})`);
    b.update(IN({ pedaling: true, load: 0 }), cfg);
    const out = b.update(IN({ pedaling: false, load: 0 }), cfg);
    check(!out.active, '8. and the stale arming cannot be replayed by a later pedal stop');
}

// 9. ACTIVE starts on the EDGE of pedalling stopping, never on the level.
{
    const b = new Boost();
    const cfg = CFG();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 2000 }), cfg);
    const still = b.update(IN({ load: 2000 }), cfg);
    check(!still.active, '9. still pedalling: no boost');
    const edge = b.update(IN({ pedaling: false, latched: false, load: 0 }), cfg);
    check(edge.active, '9. the boost starts on the pedal-stop edge');
    // A second "not pedalling" tick is not another edge, and never restarts the timer.
    const before = b.activeTicksLeft;
    b.update(IN({ pedaling: false, latched: false, load: 0 }), cfg);
    check(b.activeTicksLeft === before - 1, '9. no re-trigger while already stopped');
}

// 10. The timer is exact, in control ticks.
{
    for (const ms of [1, 200, 1000]) {
        const r = pushThenStop(new Boost(), CFG({ duration: ms }));
        check(r.activeTicks === ms * TICKS_PER_MS,
            `10. ${ms} ms lasts ${ms * TICKS_PER_MS} ticks (got ${r.activeTicks})`);
    }
}

// 11. The formula matches the plan's reference arithmetic at 100 / 150 / 255 %.
{
    const reference = (peak, trigger, pct, limit) => {
        const p = Math.min(peak, FULL_SCALE);
        const span = FULL_SCALE - trigger;
        const above = p > trigger ? p - trigger : 0;
        const base = Math.floor((limit * above + Math.floor(span / 2)) / span);
        return Math.min(Math.floor((base * pct + 50) / 100), limit);
    };
    for (const pct of [100, 150, 255]) {
        const r = pushThenStop(new Boost(), CFG({ strength: pct }), { load: 2000 });
        const want = reference(2000, 800, pct, IQ_LIMIT);
        check(r.peakIq === want, `11. ${pct}%: boost ${r.peakIq}, reference ${want}`);
    }
    // Monotonic in the peak, and near zero right at the threshold.
    const light = pushThenStop(new Boost(), CFG(), { load: 810 }).peakIq;
    const heavy = pushThenStop(new Boost(), CFG(), { load: 4000 }).peakIq;
    check(light < heavy, `11. a harder push gives more current (${light} < ${heavy})`);
    check(light * 20 < IQ_LIMIT, `11. touching the threshold gives almost nothing (${light})`);
}

// 12. Never above the level's own current limit, whatever the strength.
{
    for (const pct of [100, 200, 255]) {
        const r = pushThenStop(new Boost(), CFG({ strength: pct }), { load: FULL_SCALE });
        check(r.peakIq <= IQ_LIMIT, `12. ${pct}% at full scale stays capped (${r.peakIq})`);
    }
    check(Boost.iq(FULL_SCALE, TRIG_MIN, 255, IQ_LIMIT) === IQ_LIMIT,
        '12. the extreme case lands exactly on the cap');
}

// 13. Edge arithmetic must not overflow or divide by zero. The trigger now reaches full
//     scale, so the zero-span case is REAL and the early return is what covers it.
{
    check(Boost.trigger(0) === TRIG_MIN && Boost.trigger(60000) === TRIG_MAX,
        '13. the trigger is clamped into range');
    check(TRIG_MAX === FULL_SCALE, '13. the trigger reaches the full sensor scale');
    check(Boost.iq(FULL_SCALE, FULL_SCALE, 255, IQ_LIMIT) === 0,
        '13. a trigger AT full scale yields zero, and never divides by a zero span');
    check(Boost.iq(FULL_SCALE, FULL_SCALE - 50, 100, IQ_LIMIT) > 0,
        '13. one wire step below full scale still works');
    // Quantization: what the rider sets, the controller stores and the loop compares
    // against must be the same number.
    const step = constOf(boostH, 'ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG');
    check(step === 50, `13. the trigger grid is 0.5 kg (${step} centikg)`);
    check(FULL_SCALE % step === 0, '13. full scale lands exactly on the grid');
    check(TRIG_MAX / step <= 255, '13. the whole range fits one wire byte');
    const big = Boost.iq(FULL_SCALE, TRIG_MIN, 255, 32767);
    check(Number.isSafeInteger(big) && big === 32767, `13. no overflow at the extremes (${big})`);
    check(Boost.iq(2000, 800, 100, 0) === 0, '13. a zero current limit yields zero');
}

// 14. Every safety path cancels in the SAME tick, from any state.
{
    const paths = [
        ['safetyCut', CANCEL.SAFETY_CUT], ['reverse', CANCEL.REVERSE],
        ['walk', CANCEL.WALK], ['calibration', CANCEL.CALIBRATION]
    ];
    for (const [flag, reason] of paths) {
        for (const stage of ['qualify', 'armed', 'active']) {
            const b = new Boost();
            const cfg = CFG();
            const ticks = stage === 'qualify' ? CONFIRM_TICKS - 5 : CONFIRM_TICKS;
            for (let t = 0; t < ticks; t++) b.update(IN({ load: 2000 }), cfg);
            if (stage === 'active') b.update(IN({ pedaling: false, latched: false, load: 0 }), cfg);
            const out = b.update(IN(Object.assign(
                { pedaling: false, latched: false, load: 0 }, { [flag]: true })), cfg);
            check(!out.active && b.state === IDLE && b.cancelReason === reason,
                `14. ${flag} cancels immediately from ${stage} (reason ${b.cancelReason})`);
        }
    }
    for (const bad of ['torqueValid', 'pasValid']) {
        const b = new Boost();
        const cfg = CFG();
        for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 2000 }), cfg);
        b.update(IN(Object.assign({ pedaling: false, latched: false }, { [bad]: false })), cfg);
        check(b.state === IDLE && b.cancelReason === CANCEL.SENSOR_INVALID,
            `14. an invalid ${bad} cancels`);
    }
    const zero = new Boost();
    const cfg0 = CFG();
    for (let t = 0; t < CONFIRM_TICKS; t++) zero.update(IN({ load: 2000 }), cfg0);
    zero.update(IN({ pedaling: false, latched: false, level: 0 }), cfg0);
    check(zero.state === IDLE, '14. assist level 0 cancels');
}

// 15. Changing bank or level throws the arming away — the boost belongs to the settings it
//     was armed under.
{
    for (const change of [{ bank: 1 }, { level: 4 }]) {
        const b = new Boost();
        const cfg = CFG();
        for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 2000 }), cfg);
        b.update(IN(Object.assign({ pedaling: false, latched: false, load: 0 }, change)), cfg);
        check(b.state === IDLE && b.cancelReason === CANCEL.LEVEL_OR_BANK,
            `15. ${JSON.stringify(change)} cancels the arming`);
    }
}

// 16. Losing motion during ACTIVE stops it: no pushing a stationary bike.
{
    const b = new Boost();
    const cfg = CFG();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 2000 }), cfg);
    b.update(IN({ pedaling: false, latched: false, load: 0 }), cfg);
    const out = b.update(IN({ pedaling: false, latched: false, load: 0, motion: false }), cfg);
    check(!out.active && b.cancelReason === CANCEL.MOTION_LOST, '16. motion lost ends ACTIVE');
    // And it cannot arm into ACTIVE without motion in the first place.
    const c = new Boost();
    const r = pushThenStop(c, cfg, { extra: { motion: false } });
    check(r.activeTicks === 0, '16. no boost without motion at the trigger edge');
}

// 17. Pedalling again cancels ACTIVE and demands a completely fresh push.
{
    const b = new Boost();
    const cfg = CFG({ duration: 1000 });
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 2000 }), cfg);
    b.update(IN({ pedaling: false, latched: false, load: 0 }), cfg);
    for (let t = 0; t < 100; t++) b.update(IN({ pedaling: false, latched: false, load: 0 }), cfg);
    b.update(IN({ pedaling: true, load: 0 }), cfg);
    check(b.state === IDLE && b.cancelReason === CANCEL.PEDALING_RESUMED,
        '17. resumed pedalling cancels ACTIVE');
    const out = b.update(IN({ pedaling: false, latched: false, load: 0 }), cfg);
    check(!out.active, '17. and no leftover arming fires on the next stop');
    check(b.activeTicksLeft === 0, '17. the half-finished timer is gone, not paused');
}

// 18. THE ANTI-REFRESH RULE: nothing extends a running boost. Load, motion and repeated
//     stop conditions all keep arriving during ACTIVE and must change nothing.
{
    const b = new Boost();
    const cfg = CFG({ duration: 200 });
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 2000 }), cfg);
    let active = 0;
    for (let t = 0; t < 4000; t++) {
        // Load keeps wobbling above the trigger with the cranks stopped (rider leaning on
        // the pedal over a rock) — this must not re-arm or extend anything.
        const out = b.update(IN({
            pedaling: false, latched: false, load: t % 2 ? 3000 : 0
        }), cfg);
        if (out.active) active++;
    }
    check(active === 200 * TICKS_PER_MS, `18. the boost still lasts exactly 800 ticks (${active})`);
}

// 19. Throttle can neither arm the function nor change the stored peak: the module is fed
//     pedal load only, and ride_control hands it the pedal-only target.
{
    check(!/throttle/i.test(boostC), '19. the module knows nothing about throttle');
    const boostBlock = rideC.slice(rideC.indexOf('FW-084: Extended Boost'));
    const throttleAt = rideC.indexOf('int32_t throttle_iq = 0;');
    const boostAt = rideC.indexOf('assist_extended_boost_update');
    check(boostAt > 0 && throttleAt > boostAt,
        '19. the boost runs on the pedal-only target, before the throttle floor is merged');
    check(/pedal_load_centikg = rider->torque_load_centikg/.test(boostBlock),
        '19. it is fed calibrated pedal load, not a current');
}

// 20. Exactly one release ramp runs afterwards: the module holds the profile "pedalling"
//     while ACTIVE and lets go the moment the timer ends. It owns no ramp of its own.
{
    const b = new Boost();
    const cfg = CFG();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(IN({ load: 2000 }), cfg);
    let holds = 0, lastHold = -1;
    for (let t = 0; t < 2000; t++) {
        const out = b.update(IN({ pedaling: false, latched: false, load: 0 }), cfg);
        if (out.hold) { holds++; lastHold = t; }
    }
    check(holds === 200 * TICKS_PER_MS, `20. the hold covers exactly the boost (${holds})`);
    check(lastHold === holds - 1, '20. the hold is one contiguous block, then it is released');
    check(!/release_ms|ramp/i.test(boostC), '20. no second ramp inside the module');
    check(/profile_pedaling_active = true;/.test(rideC.slice(
        rideC.indexOf('FW-084: Extended Boost'),
        rideC.indexOf('if (input->safety_cut) {   // FW-037'))),
        '20. ride_control holds the profile pedalling flag during the boost');
}

// --- the level's own ceiling still applies to a substituted target ---------------------
//
// The audit defect: max_iq_pct and max_motor_power_w are applied inside
// assist_modes_calculate(), to the MODE's result. Extended Boost replaces that result, so
// without re-applying the ceiling a level limited to 20 % could be handed the full global
// limit by one hard push. assist_limits_apply() does not help — it owns voltage,
// temperature and speed, not the two per-level ceilings.
{
    const modes = read('src', 'assist_modes.c');
    const modesH = read('inc', 'assist_modes.h');
    const CAL_I = constOf(read('inc', 'config.h'), 'CAL_I');
    const UTIL_SCALE = constOf(modes, 'MOTOR_VOLTAGE_UTILIZATION_SCALE');
    const HARD_MAX_W = constOf(modes, 'ASSIST_MOTOR_POWER_HARD_MAX_W');

    // Model of assist_modes_profile_iq_ceiling().
    const ceiling = (level, rider, packMv, iqLimit) => {
        let limit = Math.floor((iqLimit * level.max_iq_pct) / 100);
        if (limit < 0) limit = 0;
        let powerW = level.max_motor_power_w;
        if (powerW === 0 || powerW > HARD_MAX_W) powerW = HARD_MAX_W;
        if (!rider.start_phase && rider.utilization > 0 && packMv > 0) {
            const ma = Math.floor((powerW * 1000000) / packMv);
            const cap = Math.floor((ma * UTIL_SCALE) / (rider.utilization * CAL_I));
            if (limit > cap) limit = cap;
        }
        return limit;
    };
    const rolling = { start_phase: false, utilization: 1024 };

    // 1. The audit's own acceptance example.
    {
        const level = { max_iq_pct: 20, max_motor_power_w: 0 };
        const raw = Boost.iq(FULL_SCALE, 800, 255, 700);
        const capped = Math.min(raw, ceiling(level, rolling, 36000, 700));
        check(raw === 700, `ceiling: an unlimited boost would reach ${raw}`);
        check(capped === 140, `ceiling: 20 % of 700 is 140, got ${capped}`);
    }
    // 2. The power ceiling binds the same way it does for ordinary assist.
    {
        const level = { max_iq_pct: 100, max_motor_power_w: 100 };
        const cap = ceiling(level, rolling, 36000, 3000);
        check(cap < 3000, `ceiling: a 100 W level caps below the global limit (${cap})`);
        const generous = ceiling({ max_iq_pct: 100, max_motor_power_w: 1500 }, rolling, 36000, 3000);
        check(generous > cap, 'ceiling: a higher power limit allows more current');
    }
    // 3. At launch the duty is meaningless, so the power half must not bind — the same
    //    exclusion the mode path makes, or a standstill boost would be strangled.
    {
        const launching = { start_phase: true, utilization: 0 };
        const cap = ceiling({ max_iq_pct: 100, max_motor_power_w: 100 }, launching, 36000, 3000);
        check(cap === 3000, `ceiling: no power cap during the start phase (${cap})`);
    }
    // 4. Structural: one owner for the percentage, and ride_control actually applies it.
    check(/static int32_t profile_iq_pct_limit\(/.test(modes) &&
        (modes.match(/profile_iq_pct_limit\(/g) || []).length >= 3,
        'ceiling: the percentage limit has a single implementation, used by both paths');
    check(/int32_t assist_modes_profile_iq_ceiling\(/.test(modes) &&
        /assist_modes_profile_iq_ceiling\(/.test(modesH),
        'ceiling: the shared ceiling is exported');
    const boostBlock = rideC.slice(
        rideC.indexOf('FW-084: Extended Boost'),
        rideC.indexOf('if (input->safety_cut) {   // FW-037'));
    check(/assist_modes_profile_iq_ceiling\(/.test(boostBlock),
        'ceiling: ride_control re-applies it to the boost target');
    check(/iq_target = \(boost_output\.iq_target > profile_ceiling\) \?/.test(boostBlock),
        'ceiling: the boost target is clamped, not merely compared');
    check(rideC.indexOf('assist_modes_profile_iq_ceiling') <
        rideC.indexOf('int32_t throttle_iq = 0;'),
        'ceiling: it is applied before the throttle floor, so throttle keeps its own path');
}

// --- speed-limit classification is a DECISION, not a leftover --------------------------
//
// On the PAS STOP edge the ride latch drops in the same tick the boost starts, so the old
// code classified an active boost as non-pedal purely by accident of ordering. The policy is
// now written out: Extended Boost is NON_PEDAL, which in legal mode means it tapers from
// 5 km/h and is zero from 7. Changing that is a product decision, not a refactor.
{
    const decision = rideC.slice(
        rideC.indexOf('limits_input.source = assist_latched ?'),
        rideC.indexOf('int32_t pedal_iq = assist_limits_apply'));
    check(/if \(boost_active\) \{\s*limits_input\.source = ASSIST_LIMIT_SOURCE_NON_PEDAL;/.test(decision),
        'classification: an active boost is set NON_PEDAL explicitly');
    check(/DECIDED POLICY/.test(decision),
        'classification: the choice is documented where it is made');
    check(/boost_active = true;/.test(rideC),
        'classification: the flag comes from the boost, not from the latch');

    // The consequence the rider is promised, modelled against assist_limits.c.
    const limitsC = read('src', 'assist_limits.c');
    const nonPedalMax = /NON_PEDAL/.test(limitsC);
    check(nonPedalMax, 'classification: assist_limits still distinguishes the two sources');
    // And the UI has to say so, or the rider discovers it on a climb at 8 km/h.
    const help = fs.readFileSync(path.join('C:', 'Projekty', 'bafang_canable_pro',
        'ui', 'js', 'evistdrive', 'profiles.js'), 'utf8');
    check(/legal mode/i.test(help.slice(help.indexOf("key: 'extended_boost_duration_ms'"),
        help.indexOf("key: 'power_fall_filter_ms'") + 2000)) ||
        /legal/i.test(help.slice(help.indexOf("id: 'extendedBoost'"),
            help.indexOf("id: 'extendedBoost'") + 1500)),
        'classification: the Canable card tells the rider about the legal-mode speed limit');
}

// --- a bank write throws away an arming made under the old settings ---------------------
{
    const modes = read('src', 'assist_modes.c');
    check(/assist_extended_boost_reset\(ASSIST_EXT_BOOST_CANCEL_CONFIG_CHANGED\)/.test(modes),
        'config change: applying a bank blob resets the module');
    const applyTail = modes.slice(modes.indexOf('bool assist_modes_apply_bank_blob'));
    check(applyTail.indexOf('assist_extended_boost_reset') <
        applyTail.indexOf('return true;'),
        'config change: the reset happens on the accepted path, before returning success');
    check(/ASSIST_EXT_BOOST_CANCEL_CONFIG_CHANGED = 12/.test(boostH),
        'config change: it has its own diagnostic reason');
}

// --- integration: the shared limits still own the result -------------------------------
{
    const boostAt = rideC.indexOf('assist_extended_boost_update');
    const safetyAt = rideC.indexOf('if (input->safety_cut) {   // FW-037');
    const limitsAt = rideC.indexOf('int32_t pedal_iq = assist_limits_apply');
    const smoothAt = rideC.indexOf('assist_start_apply_smooth');
    const preloadAt = rideC.indexOf('FW-041: gear preload — cap the target');
    const coastAt = rideC.indexOf('coast_release = true;');
    check(boostAt < safetyAt && safetyAt < limitsAt && limitsAt < smoothAt &&
        smoothAt < preloadAt && preloadAt < coastAt,
        'integration: boost -> safety_cut -> limits -> smooth start -> preload -> coast');
    check(/assist_extended_boost_init\(\);/.test(rideC), 'integration: ride_control_init inits the module');
    check(/assist_extended_boost_reset\(ASSIST_EXT_BOOST_CANCEL_WALK\)/.test(rideC),
        'integration: the Walk Assist path resets the module before returning');
    check(/assist_extended_boost_reset\(ASSIST_EXT_BOOST_CANCEL_CALIBRATION\)/.test(rideC),
        'integration: the calibration path resets the module before returning');
}

// --- bank blob v8 and the CAN transport ------------------------------------------------
{
    const recV7 = constOf(assistC, 'BANK_RECORD_LEN_V7');
    const recV8 = constOf(assistC, 'BANK_RECORD_LEN_V8');
    const header = constOf(assistC, 'BANK_BLOB_HEADER_LEN');
    const blob = constOf(assistH, 'ASSIST_BANK_BLOB_LEN');
    check(recV7 === 46 && recV8 === 48, `blob: record 46 -> 48 B (${recV7}/${recV8})`);
    check(header + 5 * recV8 + 2 === 255 && blob === 255, `blob: v8 is exactly 255 B (${blob})`);
    check(header + 5 * recV8 === 253, 'blob: CRC sits at bytes 253-254');
    check(blob <= 255, 'blob: the single length byte of the transport still carries it');
    check(/#define\s+BANK_BLOB_VERSION\s+BANK_BLOB_VERSION_V8/.test(assistC),
        'blob: the serializer advertises v8');

    // The write path must accept the 32nd frame (index 31, 7 bytes) in BOTH branches, or
    // the closing bytes are dropped in silence and the write fails on CRC.
    const frameGuards = canC.match(/if\(Ext_ID_Rx\.command < 31\) append_multiframe/g) || [];
    check(frameGuards.length === 2, `transport: both 0x6021 branches allow frame 31 (${frameGuards.length})`);
    const lastFrameBytes = 255 - 31 * 8;
    check(lastFrameBytes === 7, `transport: the last frame carries 7 B (${lastFrameBytes})`);
    check(/rx_data_length/.test(canC) && !/rx_data_length\+\+/.test(canC),
        'transport: the u8 length is never incremented (255 would wrap to 0)');

    // Field placement, exactly as the plan specifies.
    check(/record\[36\] = \(uint8_t\)\(valid_ext_boost_trigger_centikg/.test(assistC) &&
        /ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG/.test(assistC),
        'blob: trigger load at record[36] as u8 on the 0.5 kg grid');
    check(/record\[37\] = cfg->extended_boost\.strength_pct;/.test(assistC),
        'blob: strength at record[37]');
    check(/put_u16\(&record\[46\], valid_ext_boost_duration_ms/.test(assistC),
        'blob: duration at record[46..47] as u16 LE');

    // Migration: an older profile comes back with the function OFF, never with whatever
    // those bytes used to mean.
    const migration = assistC.slice(assistC.indexOf('FW-084: bytes 36..37'));
    check(/ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG/.test(migration) &&
        /extended_boost\.duration_ms = 0;/.test(migration),
        'blob: v1..v7 migrate to 20.0 kg / 100 % / 0 ms');
    check(constOf(boostH, 'ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG') === 2000,
        'blob: the migration default is 20.0 kg');

    // Defaults ship OFF for every level of every bank.
    const defaults = (assistC.match(/\.extended_boost = \{[^}]*\}/g) || []);
    check(defaults.length === 2, `defaults: both level templates carry the setting (${defaults.length})`);
    check(defaults.every((d) => /\b0\b\s*\\?\s*$/m.test(d.split(',').pop())),
        'defaults: duration 0 — Extended Boost is off out of the box');
}

// --- diagnostics 0x6029 v5 -------------------------------------------------------------
{
    check(/dg\[2\]=5;/.test(canC), 'diag: the block advertises v5');
    check(/send_multiframe\(Ext_ID_Rx\.command, \(char\*\)&dg\[0\], 55\);/.test(canC),
        'diag: v5 is 55 B long');
    check(/for\(uint8_t i=0;i<53;i\+\+\)/.test(canC) && /dg\[53\]=c&0xFF; dg\[54\]=\(c>>8\)&0xFF;/.test(canC),
        'diag: CRC covers bytes 0..52 and sits at 53..54');
    check(/uint8_t dg\[56\];/.test(canC), 'diag: the buffer holds 55 B');
    for (const field of ['eb.peak_load_centikg', 'eb.remaining_ms', 'eb.cancel_reason'])
        check(canC.includes(field), `diag: ${field} is reported`);
}

console.log(failures === 0 ? '\nAll FW-084 checks passed.' : `\n${failures} FW-084 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
