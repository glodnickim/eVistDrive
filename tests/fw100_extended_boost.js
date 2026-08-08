// FW-100 design test: Extended Boost — the deliberate drive hold after the cranks stop.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw100_extended_boost.js
//
// WHAT THIS IS: structural checks on the C sources, plus a MODEL of the state machine
// re-implemented in JavaScript. It proves the design behaves as the card says and that the
// wiring in ride_control.c is in the right order — it does NOT prove the compiled module does
// the same thing, and it cannot see an integer promotion or an uninitialized field at all.
//
// The shipped C is exercised by tests/host/fw100_extended_boost_host.c, which links against
// src/assist_extended_boost.c itself:  powershell -File tests/host/run-host-tests.ps1
// Keep the two in step; when they disagree, the C harness is the one that is right.
//
// THE FEATURE DRIVES THE MOTOR WITH THE CRANKS STATIONARY. That is a deliberate owner
// decision (see the accepted-risk block in the header), which makes the checks that say NO the
// important ones here: a short spike must not arm it, a stale arming must not fire, the timer
// must not be refreshed by anything, and every cancel must act in the same control tick.

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

/*
 * Structural checks must read CODE, not prose. The sources deliberately explain at length what
 * was removed and why, so a bare text match on a removed identifier fires on its own obituary.
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
const ARM_TIMEOUT_MS = constOf(boostH, 'EXT_BOOST_ARM_TIMEOUT_MS');
const TRIG_MAX = constOf(boostH, 'ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG');
const DUR_MAX = constOf(boostH, 'ASSIST_EXT_BOOST_DURATION_MAX_MS');
const CONFIRM_TICKS = CONFIRM_MS * TICKS_PER_MS;
const ARM_TIMEOUT_TICKS = ARM_TIMEOUT_MS * TICKS_PER_MS;

const CANCEL = {
    NONE: 0, DISABLED: 1, SAFETY_CUT: 2, REVERSE: 3, SENSOR_INVALID: 4, WALK: 5,
    CALIBRATION: 6, LEVEL_OR_BANK: 7, MOTION_LOST: 8, PEDALING_RESUMED: 9,
    ARM_TIMEOUT: 10, COMPLETED: 11, CONFIG_CHANGED: 12,
};
const IDLE = 0, QUALIFY = 1, ARMED = 2, ACTIVE = 3;

console.log('FW-100 Extended Boost — drive hold after the cranks stop');
console.log(`  confirm ${CONFIRM_MS} ms, arm timeout ${ARM_TIMEOUT_MS} ms, ` +
    `hysteresis ${(HYST / 100).toFixed(2)} kg, duration ceiling ${DUR_MAX} ms`);

// --- faithful port of src/assist_extended_boost.c ---------------------------------------
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
    static boostIqOf(lastPedalIq, strengthPct, iqLimit) {
        if (lastPedalIq <= 0 || strengthPct === 0) return 0;
        let scaled = Math.floor((lastPedalIq * strengthPct + 50) / 100);
        if (iqLimit > 0 && scaled > iqLimit) scaled = iqLimit;
        return scaled;
    }
    qualifyAndArm(input, trigger) {
        const mayQualify = input.pedalingActive && input.latched;
        const load = input.load;
        if (mayQualify && load >= trigger) {
            if (!this.windowOpen) {
                this.windowOpen = true; this.confirmTicks = 0; this.candidatePeak = 0;
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
            const ended = !mayQualify || (load + HYST) < trigger;
            if (ended) {
                this.windowOpen = false; this.confirmTicks = 0; this.candidatePeak = 0;
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
        else if (this.state === ACTIVE && input.pedalingActive) cancel = CANCEL.PEDALING_RESUMED;

        this.lastBank = input.bank; this.lastLevel = input.level; this.haveContext = true;

        if (cancel !== CANCEL.NONE) {
            this.reset(cancel);
            this.prevPedaling = input.pedalingActive;
            return out;
        }
        if (this.state !== ACTIVE) {
            this.qualifyAndArm(input, trigger);
            const stopEdge = this.prevPedaling && !input.pedalingActive;
            if (this.state === ARMED && stopEdge && input.motionValid) {
                const candidate = Boost.boostIqOf(
                    input.lastPedalIq, cfg.strength, input.iqLimit);
                if (candidate > 0) {
                    this.boostIq = candidate;
                    this.activeTicksLeft = duration * TICKS_PER_MS;
                    this.state = ACTIVE;
                    this.windowOpen = false; this.confirmTicks = 0; this.candidatePeak = 0;
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
                out.iq = this.boostIq;
                out.active = true;
            }
        }
        this.prevPedaling = input.pedalingActive;
        return out;
    }
}

const TRIGGER = 2000, IQ_LIMIT = 3000, HARD = TRIGGER + 1000, DURATION = 500;
const LAST_IQ = 120;
const cfg = { trigger: TRIGGER, strength: 100, duration: DURATION };
const riding = (load) => ({
    pedalingActive: true, latched: true, motionValid: true, torqueValid: true,
    pasValid: true, safetyCut: false, reverse: false, walk: false, calibration: false,
    bank: 0, level: 3, load, iqLimit: IQ_LIMIT, lastPedalIq: LAST_IQ,
});
const cranksStopped = (load = 0) => ({ ...riding(load), pedalingActive: false, latched: false });

const armAndStop = (b, load = HARD) => {
    for (let t = 0; t < CONFIRM_TICKS; t++) b.update(riding(load), cfg);
    return b.update(cranksStopped(), cfg);
};

// --- the boost holds what the rider already had ------------------------------------------
{
    const b = new Boost();
    const first = armAndStop(b);
    check(first.active, 'a confirmed push then stopping the cranks starts the boost');
    check(first.iq === LAST_IQ,
        'at 100 % the held current is exactly the last pedal request (continuity)');

    let ran = 1;
    for (let t = 0; t < DURATION * TICKS_PER_MS + 10; t++) {
        const o = b.update(cranksStopped(), cfg);
        if (!o.active) break;
        check(o.iq === LAST_IQ, 'the held current does not drift during the boost');
        ran++;
    }
    check(ran === DURATION * TICKS_PER_MS, 'the boost runs for exactly the configured time');
    check(b.cancelReason === CANCEL.COMPLETED, 'a boost that ran its time reports COMPLETED');
}

// --- strength scales it, the trigger does NOT ---------------------------------------------
{
    const b = new Boost();
    const out = armAndStop(b);
    const gentle = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) gentle.update(riding(TRIGGER), cfg);
    const gentleOut = gentle.update(cranksStopped(), cfg);
    check(out.iq === gentleOut.iq,
        'a harder arming push gives the SAME current — the trigger only arms');

    const strong = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) {
        strong.update(riding(HARD), { ...cfg, strength: 150 });
    }
    const strongOut = strong.update(cranksStopped(), { ...cfg, strength: 150 });
    check(strongOut.iq === Math.floor((LAST_IQ * 150 + 50) / 100),
        'strength_pct scales the held current');

    const capped = new Boost();
    const smallLimit = { ...riding(HARD), iqLimit: 50 };
    for (let t = 0; t < CONFIRM_TICKS; t++) capped.update(smallLimit, cfg);
    const cappedOut = capped.update({ ...cranksStopped(), iqLimit: 50 }, cfg);
    check(cappedOut.iq <= 50, 'the level limit still caps the held current');
}

// --- resuming pedalling hands back at once ------------------------------------------------
{
    const b = new Boost();
    check(armAndStop(b).active, 'precondition: boost running');
    for (let t = 0; t < 20; t++) b.update(cranksStopped(), cfg);
    const o = b.update(riding(0), cfg);
    check(!o.active && o.iq === 0, 'pedalling again ends the boost in the same tick');
    check(b.cancelReason === CANCEL.PEDALING_RESUMED, 'and reports PEDALING_RESUMED');
}

// --- every hard condition stops it in the same tick ---------------------------------------
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
        check(armAndStop(b).active, `precondition for ${name}`);
        const o = b.update({ ...cranksStopped(), ...patch }, cfg);
        check(!o.active && o.iq === 0, `${name} stops the boost in the same tick`);
        check(b.cancelReason === expect, `${name} reports its own cancel reason`);
    }
}

// --- the checks that say NO ---------------------------------------------------------------
{
    // A spike shorter than the confirm time never arms.
    const b = new Boost();
    for (let t = 0; t < CONFIRM_TICKS - 1; t++) b.update(riding(HARD), cfg);
    let fired = false;
    for (let t = 0; t < DURATION * TICKS_PER_MS; t++) {
        if (b.update(cranksStopped(), cfg).active) fired = true;
    }
    check(!fired, 'a push shorter than the confirm time never fires');

    // A stale arming expires instead of firing later.
    const s = new Boost();
    for (let t = 0; t < CONFIRM_TICKS; t++) s.update(riding(HARD), cfg);
    check(s.state === ARMED, 'precondition: armed');
    for (let t = 0; t < ARM_TIMEOUT_TICKS + 2; t++) s.update(riding(0), cfg);
    check(s.state === IDLE && s.cancelReason === CANCEL.ARM_TIMEOUT,
        'an arming that waits too long expires');
    let lateFire = false;
    for (let t = 0; t < DURATION * TICKS_PER_MS; t++) {
        if (s.update(cranksStopped(), cfg).active) lateFire = true;
    }
    check(!lateFire, 'and cannot be cashed in afterwards');

    // Off by default.
    const off = new Boost();
    let any = false;
    for (let t = 0; t < CONFIRM_TICKS; t++) off.update(riding(HARD), { ...cfg, duration: 0 });
    for (let t = 0; t < 100; t++) {
        if (off.update(cranksStopped(), { ...cfg, duration: 0 }).active) any = true;
    }
    check(!any, 'duration 0 means the feature does nothing at all');

    // Nothing was flowing -> nothing to hold.
    const zero = new Boost();
    const noAssist = { ...riding(HARD), lastPedalIq: 0 };
    for (let t = 0; t < CONFIRM_TICKS; t++) zero.update(noAssist, cfg);
    const zo = zero.update({ ...cranksStopped(), lastPedalIq: 0 }, cfg);
    check(!zo.active, 'with no current flowing beforehand there is nothing to hold');

    // The timer is not refreshed by anything while it runs.
    const t2 = new Boost();
    armAndStop(t2);
    let ticks = 1;
    while (t2.update({ ...cranksStopped(), load: HARD }, cfg).active) ticks++;
    check(ticks === DURATION * TICKS_PER_MS,
        'load still on the pedal does not extend the timer');
}

// --- structural: the wiring, and the decisions that must not drift ------------------------
{
    check(/EXT_BOOST_ARM_TIMEOUT_TICKS/.test(boostCCode),
        'structural: the stale-arming timeout is live again');
    check(/pedal_stop_edge/.test(boostCCode),
        'structural: the boost starts on the pedal-stop edge');
    check(/input->last_pedal_iq/.test(boostCCode),
        'structural: the current comes from the last pedal request');
    check(!/TORQUE_PUBLIC_FULL_SCALE_CENTIKG/.test(boostCCode),
        'structural: the old kg-to-current map is gone');

    // The module must never claim the rider is pedalling.
    check(!/profile_hold_active/.test(boostHCode) && !/profile_hold_active/.test(boostCCode),
        'structural: the module cannot raise the pedalling flag');
    const pedalingAssignments = [...rideCCode.matchAll(
        /profile_pedaling_active\s*=\s*true\s*;/g)].map((m) =>
        /boost/i.test(rideCCode.slice(Math.max(0, m.index - 260), m.index)));
    check(pedalingAssignments.every((fromBoost) => !fromBoost),
        'structural: no "boost -> pedaling = true" anywhere in ride_control');

    // The owner's speed decision, written out where it is made.
    check(/if \(boost_active\) \{\s*limits_input\.source = ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED;/
        .test(rideCCode),
        'structural: an active boost is classified PEDAL_CONFIRMED explicitly');
    check(/OWNER DECISION/.test(rideC),
        'structural: that choice is documented as a decision where it is made');

    // Capture point: after the limiter, and not while a boost is running.
    const capture = rideCCode.slice(rideCCode.indexOf('int32_t pedal_iq = assist_limits_apply'));
    check(/rider->pedaling_active && !boost_active/.test(capture),
        'structural: the reference is captured only while pedalling, never from a boost');

    // Ordering: boost -> hard cut -> shared limits.
    const boostAt = rideCCode.indexOf('assist_extended_boost_update');
    const cutAt = rideCCode.indexOf('if (hard_cut) {');
    const limitsAt = rideCCode.indexOf('int32_t pedal_iq = assist_limits_apply');
    check(boostAt > 0 && boostAt < cutAt && cutAt < limitsAt,
        'structural: boost -> hard cut -> shared limits, in that order');

    check(DUR_MAX === 2000, 'structural: the duration ceiling is the agreed 2000 ms');
}

if (failures === 0) {
    console.log('\nAll FW-100 checks passed.');
} else {
    console.log(`\n${failures} FW-100 check(s) FAILED.`);
    process.exitCode = 1;
}
