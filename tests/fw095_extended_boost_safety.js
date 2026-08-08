// FW-095 design test: Extended Boost may not survive a real PAS STOP, and may not fake the
// pedalling state it is judged by.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw095_extended_boost_safety.js
//
// WHAT THIS IS: structural checks on the C sources, plus a MODEL of the state machine
// re-implemented in JavaScript. It proves the design says what the card says and that the
// wiring in ride_control.c is in the right order — it does NOT prove the compiled module
// behaves the same way, and it cannot see an integer promotion or an uninitialized field.
//
// The shipped C is exercised by tests/host/fw095_extended_boost_host.c, which links against
// src/assist_extended_boost.c itself:  powershell -File tests/host/run-host-tests.ps1
// Keep the two in step; when they disagree, the C harness is the one that is right.
//
// The safety rule under test, in one sentence: on a bike whose brake input we cannot rely on,
// no ride-feel feature may keep the motor pulling once the rider has actually stopped
// pedalling, and none may report pedalling that is not happening.

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
const rideC = read('src', 'ride_control.c');
const torqueH = read('inc', 'torque_input.h');

/*
 * Structural checks must read CODE, not prose. The sources deliberately explain at length what
 * was removed and why it must not come back, so a bare text match on a removed identifier
 * fires on its own obituary. Strip comments first and match against what the compiler sees.
 */
const codeOf = (src) => src
    .replace(/\/\*[\s\S]*?\*\//g, ' ')
    .replace(/\/\/[^\n]*/g, ' ');

const boostHCode = codeOf(boostH);
const boostCCode = codeOf(boostC);
const rideCCode = codeOf(rideC);

const constOf = (src, name) => {
    const m = src.match(new RegExp(`#define\\s+${name}\\s+(\\w+)`));
    if (!m) throw new Error(`${name} not found`);
    return Number(m[1].replace(/U$/, ''));
};

const TICKS_PER_MS = constOf(boostH, 'EXT_BOOST_CONTROL_TICKS_PER_MS');
const CONFIRM_MS = constOf(boostH, 'EXT_BOOST_CONFIRM_MS');
const HYST = constOf(boostH, 'EXT_BOOST_RELEASE_HYST_CENTIKG');
const TRIG_MAX = constOf(boostH, 'ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG');
const DUR_MAX = constOf(boostH, 'ASSIST_EXT_BOOST_DURATION_MAX_MS');
const FULL_SCALE = constOf(torqueH, 'TORQUE_PUBLIC_FULL_SCALE_CENTIKG');
const CONFIRM_TICKS = CONFIRM_MS * TICKS_PER_MS;

const CANCEL = {
    NONE: 0, DISABLED: 1, SAFETY_CUT: 2, REVERSE: 3, SENSOR_INVALID: 4, WALK: 5,
    CALIBRATION: 6, LEVEL_OR_BANK: 7, MOTION_LOST: 8, CONFIG_CHANGED: 12,
    PEDALING_STOPPED: 13, COMPLETED: 11
};
const IDLE = 0, QUALIFY = 1, ACTIVE = 3;

console.log('FW-095 Extended Boost safety');
console.log(`  confirm ${CONFIRM_MS} ms, hysteresis ${(HYST / 100).toFixed(2)} kg, ` +
    `duration ceiling ${DUR_MAX} ms`);

// --- faithful port of src/assist_extended_boost.c ---------------------------------------
class Boost {
    constructor() {
        this.clear();
        this.rearmBlocked = false;
        this.cancelReason = CANCEL.NONE;
        this.lastBank = 0; this.lastLevel = 0; this.haveContext = false;
    }
    clear() {
        this.state = IDLE;
        this.candidatePeak = 0; this.firedPeak = 0;
        this.confirmTicks = 0; this.activeTicksLeft = 0;
        this.windowOpen = false; this.boostIq = 0;
    }
    reset(reason) {
        this.clear();
        if (reason !== CANCEL.NONE) this.cancelReason = reason;
    }
    static boostIq(peak, trigger, strengthPct, iqLimit) {
        if (iqLimit <= 0 || strengthPct === 0) return 0;
        const p = Math.min(peak, FULL_SCALE);
        if (p <= trigger) return 0;
        const span = FULL_SCALE - trigger;
        const above = p - trigger;
        const base = Math.floor((iqLimit * above + Math.floor(span / 2)) / span);
        return Math.min(Math.floor((base * strengthPct + 50) / 100), iqLimit);
    }
    qualify(input, trigger) {
        const mayQualify = input.pedalingActive && input.latched;
        const load = input.load;
        if (mayQualify && load >= trigger) {
            if (!this.windowOpen) {
                this.windowOpen = true;
                this.confirmTicks = 0;
                this.candidatePeak = 0;
                if (this.state === IDLE) this.state = QUALIFY;
            }
            if (load > this.candidatePeak) this.candidatePeak = load;
            if (this.rearmBlocked) return false;
            if (this.confirmTicks < CONFIRM_TICKS) this.confirmTicks++;
            return this.confirmTicks >= CONFIRM_TICKS;
        }
        const closed = !mayQualify || (load + HYST) < trigger;
        if (closed) {
            this.windowOpen = false;
            this.confirmTicks = 0;
            this.candidatePeak = 0;
            this.rearmBlocked = false;
            if (this.state === QUALIFY) this.state = IDLE;
        }
        return false;
    }
    update(input, cfg) {
        const out = { iq: 0, active: false };
        const trigger = Math.min(Math.max(cfg.trigger, 100), TRIG_MAX);
        const duration = Math.min(cfg.duration, DUR_MAX);
        const disabled = duration === 0 || cfg.strength === 0;

        let cancel = CANCEL.NONE;
        if (disabled) cancel = CANCEL.DISABLED;
        else if (input.safetyCut) cancel = CANCEL.SAFETY_CUT;
        else if (input.reverse) cancel = CANCEL.REVERSE;
        else if (!input.torqueValid || !input.pasValid) cancel = CANCEL.SENSOR_INVALID;
        else if (input.walk) cancel = CANCEL.WALK;
        else if (input.calibration) cancel = CANCEL.CALIBRATION;
        else if (input.level === 0) cancel = CANCEL.LEVEL_OR_BANK;
        else if (this.haveContext &&
            (input.bank !== this.lastBank || input.level !== this.lastLevel))
            cancel = CANCEL.LEVEL_OR_BANK;
        else if (this.state === ACTIVE && !input.motionValid) cancel = CANCEL.MOTION_LOST;
        else if (!input.pedalingActive) cancel = CANCEL.PEDALING_STOPPED;
        else if (!input.latched) cancel = CANCEL.PEDALING_STOPPED;

        this.lastBank = input.bank; this.lastLevel = input.level; this.haveContext = true;

        if (cancel !== CANCEL.NONE) {
            // A boost that reached ACTIVE has paid out, so it blocks re-arming whatever
            // ended it — otherwise a stop-start under one unbroken push gives two boosts.
            const blocked = this.rearmBlocked || this.state === ACTIVE;
            this.reset(cancel);
            this.rearmBlocked = blocked;
            return out;
        }
        if (this.state !== ACTIVE) {
            if (this.qualify(input, trigger) && input.motionValid) {
                const candidate = Boost.boostIq(
                    this.candidatePeak, trigger, cfg.strength, input.iqLimit);
                if (candidate > 0) {
                    this.boostIq = candidate;
                    this.firedPeak = this.candidatePeak;
                    this.activeTicksLeft = duration * TICKS_PER_MS;
                    this.state = ACTIVE;
                } else {
                    this.rearmBlocked = true;
                }
            }
        }
        if (this.state === ACTIVE) {
            if (this.activeTicksLeft === 0) {
                this.reset(CANCEL.COMPLETED);
                this.rearmBlocked = true;
            } else {
                this.activeTicksLeft--;
                out.iq = this.boostIq;
                out.active = true;
            }
        }
        return out;
    }
}

const TRIGGER = 2000, IQ_LIMIT = 3000, HARD = TRIGGER + 1000, DURATION = 200;
const cfg = { trigger: TRIGGER, strength: 100, duration: DURATION };
const riding = (load) => ({
    pedalingActive: true, latched: true, motionValid: true, torqueValid: true,
    pasValid: true, safetyCut: false, reverse: false, walk: false, calibration: false,
    bank: 0, level: 3, load, iqLimit: IQ_LIMIT
});
const stopped = (load) => ({ ...riding(load), pedalingActive: false, latched: false });

// --- Test 2: boost runs during a confirmed push, while pedalling ------------------------
{
    const b = new Boost();
    let started = -1;
    for (let t = 1; t <= CONFIRM_TICKS + 4 && started < 0; t++) {
        if (b.update(riding(HARD), cfg).active) started = t;
    }
    check(started === CONFIRM_TICKS, 'boost starts exactly on the confirm tick');

    let ran = 1;
    for (let t = 0; t < DURATION * TICKS_PER_MS + 10; t++) {
        const o = b.update(riding(HARD), cfg);
        if (!o.active) break;
        ran++;
    }
    check(ran === DURATION * TICKS_PER_MS, 'boost runs for exactly the configured duration');
}

// --- Test 3: PAS STOP ends it in the same tick, with timer left -------------------------
{
    const b = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(riding(HARD), cfg);
    check(b.state === ACTIVE, 'precondition: boost is running');

    // Worst case: the cranks stop with the rider's weight still on the pedal.
    const o = b.update(stopped(HARD), cfg);
    check(!o.active && o.iq === 0, 'PAS STOP ends the boost in the same control tick');
    check(b.cancelReason === CANCEL.PEDALING_STOPPED,
        'PAS STOP is reported as PEDALING_STOPPED, not COMPLETED');

    let drove = false;
    for (let t = 0; t < DURATION * TICKS_PER_MS * 4; t++) {
        const x = b.update(stopped(HARD), cfg);
        if (x.active || x.iq !== 0) drove = true;
    }
    check(!drove, 'no motor overrun after a real PAS STOP');
}

// --- The removed FW-084 trigger must not come back --------------------------------------
{
    const b = new Boost();
    for (let t = 0; t < CONFIRM_TICKS - 1; t++) b.update(riding(HARD), cfg);
    let fired = false;
    for (let t = 0; t < DURATION * TICKS_PER_MS * 4; t++) {
        const o = b.update(stopped(0), cfg);
        if (o.active || o.iq !== 0) fired = true;
    }
    check(!fired, 'the pedal-stop EDGE never starts a boost');
}

// --- Tests 4/5/6/7: every hard condition stops it in the same tick ----------------------
{
    const cases = [
        ['brake / hard cut', { safetyCut: true }, CANCEL.SAFETY_CUT],
        ['backward pedalling', { reverse: true }, CANCEL.REVERSE],
        ['torque sensor fault', { torqueValid: false }, CANCEL.SENSOR_INVALID],
        ['PAS sensor fault', { pasValid: false }, CANCEL.SENSOR_INVALID],
        ['walk assist', { walk: true }, CANCEL.WALK],
        ['position calibration', { calibration: true }, CANCEL.CALIBRATION],
        ['assist level 0', { level: 0 }, CANCEL.LEVEL_OR_BANK],
        ['bike stopped moving', { motionValid: false }, CANCEL.MOTION_LOST],
    ];
    for (const [name, patch, expect] of cases) {
        const b = new Boost();
        for (let t = 0; t < CONFIRM_TICKS; t++) b.update(riding(HARD), cfg);
        const o = b.update({ ...riding(HARD), ...patch }, cfg);
        check(!o.active && o.iq === 0, `${name} stops the boost in the same tick`);
        check(b.cancelReason === expect, `${name} reports its own cancel reason`);
    }
}

// --- A held push cannot chain boosts back to back ---------------------------------------
{
    const b = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(riding(HARD), cfg);
    let activeTicks = 0;
    for (let t = 0; t < DURATION * TICKS_PER_MS * 5; t++) {
        if (b.update(riding(HARD), cfg).active) activeTicks++;
    }
    check(activeTicks < DURATION * TICKS_PER_MS,
        'holding the push does not restart the boost for ever');

    for (let t = 0; t < 8; t++) b.update(riding(TRIGGER - HYST - 1), cfg);
    let restarted = false;
    for (let t = 0; t < CONFIRM_TICKS + 4; t++) {
        if (b.update(riding(HARD), cfg).active) restarted = true;
    }
    check(restarted, 'a fresh push after easing off does start a new boost');
}

// --- One push, one boost, ACROSS a stop-start with the pedal never released -------------
//
// The case that is easy to get wrong: PAS STOP cancels an ACTIVE boost correctly, but the
// PUSH never ended. If the cancel does not block re-arming, resuming the cranks under that
// same unbroken press confirms a fresh window 30 ms later and pays out a second boost.
// Not a post-PAS safety hole — the second boost still needs live pedalling — but it breaks
// "one push gives one boost", which is what the rider is told.
{
    const b = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(riding(HARD), cfg);
    check(b.state === ACTIVE, 'precondition: a boost is running');

    // Cranks stop, full weight still on the pedal.
    b.update(stopped(HARD), cfg);
    check(b.cancelReason === CANCEL.PEDALING_STOPPED, 'precondition: cancelled by PAS STOP');
    for (let t = 0; t < 40; t++) b.update(stopped(HARD), cfg);

    // Pedalling resumes, load never having dropped below the trigger.
    let secondBoost = false;
    for (let t = 0; t < CONFIRM_TICKS * 3; t++) {
        if (b.update(riding(HARD), cfg).active) secondBoost = true;
    }
    check(!secondBoost,
        'a stop-start under one unbroken push does not pay out a second boost');

    // Releasing the pedal is what re-arms it, exactly as the rider is told.
    for (let t = 0; t < 8; t++) b.update(riding(TRIGGER - HYST - 1), cfg);
    let afterRelease = false;
    for (let t = 0; t < CONFIRM_TICKS + 4; t++) {
        if (b.update(riding(HARD), cfg).active) afterRelease = true;
    }
    check(afterRelease, 'easing off and pushing again is what gives the next boost');
}

// --- What is and is not re-checked while the boost runs ---------------------------------
{
    // Easing off AFTER the trigger does not shorten the boost: the load is not re-tested.
    const b = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(riding(HARD), cfg);
    let ran = 0;
    for (let t = 0; t < DURATION * TICKS_PER_MS + 10; t++) {
        if (!b.update(riding(0), cfg).active) break;   // pedalling, but no load at all
        ran++;
    }
    check(ran === DURATION * TICKS_PER_MS - 1,
        'easing off after the trigger does not cut the boost short');
}

// --- Off by default, and a spike is not a decision --------------------------------------
{
    const b = new Boost();
    let any = false;
    for (let t = 0; t < CONFIRM_TICKS * 4; t++) {
        if (b.update(riding(HARD), { ...cfg, duration: 0 }).active) any = true;
    }
    check(!any, 'duration 0 means the feature does nothing at all');
}

// --- Test 10 / structural: the safety rule is written into the C, not just the model -----
{
    // The module must not export a way to claim pedalling.
    check(!/profile_hold_active/.test(boostHCode) && !/profile_hold_active/.test(boostCCode),
        'structural: profile_hold_active is gone from the module');
    check(!/profile_hold_active/.test(rideCCode),
        'structural: ride_control no longer holds the pedalling flag for a boost');

    // Nothing anywhere may raise the pedalling flag from a boost condition. Every remaining
    // assignment must come from throttle or the latch floor, never from the boost.
    const pedalingAssignments = [...rideCCode.matchAll(
        /profile_pedaling_active\s*=\s*true\s*;/g)].map((m) => {
        const before = rideCCode.slice(Math.max(0, m.index - 260), m.index);
        return /boost/i.test(before);
    });
    check(pedalingAssignments.every((fromBoost) => !fromBoost),
        'structural: no "boost -> pedaling = true" anywhere in ride_control');

    // The PAS STOP cancel must exist and be unconditional in the cancel chain.
    check(/ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED/.test(boostHCode),
        'structural: a dedicated PAS STOP cancel reason exists');
    check(/else if \(!input->pedaling_active\) \{/.test(boostCCode),
        'structural: the PAS STOP cancel is a plain unconditional test');
    check(!/pedal_stop_edge/.test(boostCCode),
        'structural: the pedal-stop edge trigger is gone');
    check(!/EXT_BOOST_ARM_TIMEOUT_MS/.test(boostCCode),
        'structural: the pending-arming timeout is gone with the waiting state');

    // The boost must no longer force the non-pedal speed classification, because it now only
    // ever runs while pedalling is confirmed. The latch ternary is the ONLY assignment to
    // limits_input.source before the pedal limiter call.
    const decision = rideCCode.slice(
        rideCCode.indexOf('limits_input.source = assist_latched ?'),
        rideCCode.indexOf('int32_t pedal_iq = assist_limits_apply'));
    check([...decision.matchAll(/limits_input\.source\s*=/g)].length === 1,
        'structural: no boost override of the limiter source');

    // Hard cut must use its own bounded ramp, never the rider-configurable release_ms.
    check(/#define RIDE_HARD_CUT_RAMP_MS/.test(rideCCode),
        'structural: the hard-cut ramp is a named firmware constant');
    check(/_Static_assert\(RIDE_HARD_CUT_RAMP_MS <= \d+/.test(rideCCode),
        'structural: the hard-cut ramp is bounded by an assertion');
    check(/profile_release_ms = RIDE_HARD_CUT_RAMP_MS;/.test(rideCCode),
        'structural: a hard cut clamps the fade to that constant');
    check(!/profile_release_ms\s*=\s*level->release_ms[\s\S]{0,200}hard_cut/.test(rideCCode),
        'structural: the hard cut never falls back to the level release time');
    const hardCutBlock = rideCCode.slice(rideCCode.indexOf('if (hard_cut) {'),
        rideCCode.indexOf('if (hard_cut) {') + 400);
    check(/iq_target = 0;/.test(hardCutBlock) && /assist_latched = false;/.test(hardCutBlock),
        'structural: a hard cut zeroes the demand and drops the latch');

    // Ordering: the boost is applied before the hard cut and before both limiter calls, so
    // brake and the speed/power/voltage limits all still have the last word over it.
    const boostAt = rideCCode.indexOf('assist_extended_boost_update');
    const cutAt = rideCCode.indexOf('if (hard_cut) {');
    const limitsAt = rideCCode.indexOf('int32_t pedal_iq = assist_limits_apply');
    check(boostAt > 0 && boostAt < cutAt && cutAt < limitsAt,
        'structural: boost -> hard cut -> shared limits, in that order');
}

if (failures === 0) {
    console.log('\nAll FW-095 checks passed.');
} else {
    console.log(`\n${failures} FW-095 check(s) FAILED.`);
    process.exitCode = 1;
}
