// FW-058 + FW-059 host test for the coast re-zero: how often it may run (FW-058)
// and where in the coast the sample comes from (FW-059).
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw058_coast_rezero.js
//
// No host C toolchain here, so the thresholds are read out of the real inc/config.h
// and driven through a port of the coast path from src/torque_input.c
// (coast_accumulate / coast_evaluate / torque_input_coast_update). The port is
// duplicated on purpose — edit it together with the C.

'use strict';
const fs = require('fs');
const path = require('path');

const headers = ['config.h', 'torque_input.h']
    .map((name) => fs.readFileSync(path.join(__dirname, '..', 'inc', name), 'utf8'))
    .join('\n');
const constant = (name) => {
    const match = headers.match(new RegExp(`#define\\s+${name}\\s+(\\d+)U?`));
    if (!match) throw new Error(`missing #define ${name} in inc/`);
    return parseInt(match[1], 10);
};
const IDLE_TICKS = constant('TQ_RECAL_IDLE_TICKS');
const SETTLE_TICKS = constant('TQ_RECAL_SETTLE_TICKS');
const MIN_PERIOD_TICKS = constant('TQ_RECAL_MIN_PERIOD_TICKS');
const MOVING_X100 = constant('TQ_RECAL_MOVING_X100');
const BAND_MV = constant('TQ_RECAL_BAND_MV');
const MAX_STEP = constant('TQ_RECAL_MAX_STEP');
const STABLE_MV = constant('TQ_RECAL_STABLE_MV');
const REACQUIRE_MAX_MV = constant('TQ_REACQUIRE_MAX_MV');
const REACQUIRE_TOL_MV = constant('TQ_REACQUIRE_TOL_MV');
const REACQUIRE_COASTS = constant('TQ_REACQUIRE_COASTS');
const REST_RAW_MIN = constant('TQ_REST_RAW_MIN');
const REST_RAW_MAX = constant('TQ_REST_RAW_MAX');
const REST_TARGET = constant('TORQUE_ZERO_TARGET_NATIVE');
const TICKS_PER_S = 4000;

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

// --- port of the coast path in src/torque_input.c ---
const RESULT = {
    NONE: 'NONE', APPLIED: 'APPLIED', NO_CHANGE: 'NO_CHANGE', TOO_SHORT: 'TOO_SHORT',
    UNSTABLE: 'UNSTABLE', LOCKOUT: 'LOCKOUT',
    OUT_OF_REACQUIRE_RANGE: 'OUT_OF_REACQUIRE_RANGE', IMPLAUSIBLE_RAW: 'IMPLAUSIBLE_RAW',
};

function makeSensor() {
    return {
        offset: 0, coastActive: false, coastTicks: 0, acc: 0,
        candidate: 0, candidateStable: false, windowMin: 0, windowMax: 0,
        lockout: 0, calFault: false,
        reacquireCount: 0, lastCoastRest: 0,
        lastResult: RESULT.NONE, lastStep: 0,
        windowsStarted: 0, windowsCompleted: 0,
        corrections: 0, evaluations: 0,
        rejTooShort: 0, rejUnstable: 0, rejLockout: 0, rejRange: 0, rejImplausible: 0,
        noChange: 0,
    };
}

function coastUpdate(s, torqueCorrected, coastEligible, bikeMoving) {
    if (s.lockout > 0) s.lockout--;
    if (coastEligible) {
        if (!s.coastActive) {
            s.coastActive = true;
            s.coastTicks = 0;
            s.acc = torqueCorrected << 6;
            s.windowMin = torqueCorrected;
            s.windowMax = torqueCorrected;
            s.candidate = torqueCorrected;
            s.candidateStable = false;
            s.windowsStarted++;
            return;
        }
        if (s.coastTicks > SETTLE_TICKS) return; // candidate frozen
        s.acc -= s.acc >> 6;
        s.acc += torqueCorrected;
        if (torqueCorrected < s.windowMin) s.windowMin = torqueCorrected;
        if (torqueCorrected > s.windowMax) s.windowMax = torqueCorrected;
        s.coastTicks++;
        if (s.coastTicks > SETTLE_TICKS) {
            s.candidate = s.acc >> 6;
            s.candidateStable = (s.windowMax - s.windowMin) <= STABLE_MV;
        }
        return;
    }
    if (s.coastActive) {
        if (s.coastTicks > SETTLE_TICKS) {
            coastEvaluate(s, bikeMoving);
        } else {
            s.lastResult = RESULT.TOO_SHORT;
            s.rejTooShort++;
        }
    }
    s.coastActive = false;
}

function applyAndArm(s, diff, bikeMoving) {
    let step = diff;
    if (step > MAX_STEP) step = MAX_STEP;
    else if (step < -MAX_STEP) step = -MAX_STEP;
    s.offset += step;
    s.lastStep = step;
    s.evaluations++;
    if (step === 0) { s.lastResult = RESULT.NO_CHANGE; s.noChange++; return; }
    s.lastResult = RESULT.APPLIED;
    s.corrections++;
    if (bikeMoving) s.lockout = MIN_PERIOD_TICKS;
}

function driftConfirmed(s, rest) {
    const spread = Math.abs(rest - s.lastCoastRest);
    if (s.reacquireCount > 0 && spread <= REACQUIRE_TOL_MV) s.reacquireCount++;
    else s.reacquireCount = 1;
    s.lastCoastRest = rest;
    if (s.reacquireCount >= REACQUIRE_COASTS) { s.reacquireCount = 0; return true; }
    return false;
}

function coastEvaluate(s, bikeMoving) {
    const rest = s.candidate;
    const rawRest = rest - s.offset;
    s.windowsCompleted++;
    if (rawRest < REST_RAW_MIN || rawRest > REST_RAW_MAX) {
        s.calFault = true; s.lastResult = RESULT.IMPLAUSIBLE_RAW; s.rejImplausible++; return;
    }
    s.calFault = false;
    if (!s.candidateStable) { s.lastResult = RESULT.UNSTABLE; s.rejUnstable++; return; }
    if (bikeMoving && s.lockout > 0) { s.lastResult = RESULT.LOCKOUT; s.rejLockout++; return; }

    const diff = REST_TARGET - rest;
    const distance = Math.abs(diff);
    if (distance <= BAND_MV) {
        s.reacquireCount = 0;
        applyAndArm(s, diff, bikeMoving);
    } else if (distance <= REACQUIRE_MAX_MV) {
        if (driftConfirmed(s, rest)) applyAndArm(s, diff, bikeMoving);
        else { s.lastResult = RESULT.NO_CHANGE; s.noChange++; }
    } else {
        s.reacquireCount = 0;
        s.lastResult = RESULT.OUT_OF_REACQUIRE_RANGE;
        s.rejRange++;
    }
}

// main.c feeds torque_input the CORRECTED value (raw + offset_correction), so the
// harness must do the same — otherwise the zero would never converge and the test
// would measure something the firmware does not do.
const corrected = (s, rawValue) => rawValue + s.offset;

// One coast of the given length, then pedalling resumes (the first crank step ends it).
// opts.endBumpMv models the real failure mode: the rider starts pressing before the
// crank clicks a PAS step, so the last fraction of a second is already loaded.
// opts.noiseMv models a coast that is not quiet (rough road, chain slap).
function coast(s, seconds, rawRest, moving, opts = {}) {
    const ticks = Math.round(seconds * TICKS_PER_S);
    const bumpTicks = Math.round((opts.endBumpSeconds ?? 0.2) * TICKS_PER_S);
    for (let i = 0; i < ticks; i++) {
        let value = rawRest;
        if (opts.noiseMv) value += (i % 2 ? opts.noiseMv : -opts.noiseMv);
        if (opts.endBumpMv && i >= ticks - bumpTicks) value += opts.endBumpMv;
        coastUpdate(s, corrected(s, value), true, moving);
    }
    coastUpdate(s, corrected(s, rawRest + (opts.endBumpMv ?? 0)), false, moving);
}
function ride(s, seconds, moving) {
    const ticks = Math.round(seconds * TICKS_PER_S);
    for (let i = 0; i < ticks; i++) coastUpdate(s, corrected(s, REST_TARGET), false, moving);
}

console.log(`idle window ${IDLE_TICKS / TICKS_PER_S}s + settle ${SETTLE_TICKS / TICKS_PER_S}s, ` +
    `min period while moving ${MIN_PERIOD_TICKS / TICKS_PER_S}s, moving above ${MOVING_X100 / 100} km/h`);

check(IDLE_TICKS === 20000, `idle window must be 5 s (20000 ticks), got ${IDLE_TICKS}`);
check(MIN_PERIOD_TICKS === 240000, `min period must be 60 s (240000 ticks), got ${MIN_PERIOD_TICKS}`);
check(IDLE_TICKS < 64000, 'idle window must stay below the pas_idle_ticks saturation of 64000');

// The eligibility window itself is main.c's job; here we prove that a coast shorter
// than the settle time is never evaluated.
{
    const s = makeSensor();
    const drifted = REST_TARGET - 10;
    coast(s, SETTLE_TICKS / TICKS_PER_S / 2, drifted, true);
    check(s.corrections === 0, `a coast shorter than the settle window must not correct, got ${s.corrections}`);
}

// FW-059: the sample is frozen mid-coast, so pressing the pedal just before the
// crank clicks a PAS step must no longer drag the zero with it.
{
    const s = makeSensor();
    coast(s, 5, REST_TARGET, false, { endBumpMv: 20 });
    check(Math.abs(s.offset) <= 1,
        `end-of-coast pre-load must not move the zero, it moved ${s.offset} mV`);
}

// FW-059: a coast that is not quiet yields no calibration rather than a bad one.
{
    const s = makeSensor();
    coast(s, 5, REST_TARGET - 10, false, { noiseMv: STABLE_MV });
    check(s.offset === 0, `a noisy coast must not correct, it moved ${s.offset} mV`);
    check(s.rejUnstable === 1 && s.lastResult === RESULT.UNSTABLE,
        `a noisy coast must be reported as UNSTABLE, got ${s.lastResult} (${s.rejUnstable})`);
}

// A quiet coast with the same drift still corrects — the stability gate must not
// simply switch calibration off.
{
    const s = makeSensor();
    coast(s, 5, REST_TARGET - 10, false);
    check(s.offset > 0, 'a quiet coast must still correct the zero');
}

// FW-059: one correction can no longer exceed the assist engage threshold.
{
    const s = makeSensor();
    coast(s, 5, REST_TARGET - BAND_MV, false); // biggest in-band drift there is
    check(Math.abs(s.offset) === MAX_STEP,
        `a single correction must clamp to ${MAX_STEP} mV, got ${s.offset}`);
    check(MAX_STEP < 18,
        `a single correction (${MAX_STEP} mV) must stay below the 18 mV engage threshold`);
}

// While moving: two coasts 30 s apart -> only the first corrects.
{
    const s = makeSensor();
    const drifted = REST_TARGET - 10;
    coast(s, 1, drifted, true);
    check(s.corrections === 1, `first in-ride coast must correct, got ${s.corrections}`);
    ride(s, 30, true);
    coast(s, 1, drifted, true);
    check(s.corrections === 1, `second coast 30 s later must be blocked, got ${s.corrections} corrections`);
}

// While moving: two coasts 70 s apart -> both are allowed to act on the zero.
// (The second one finds it already on target, so it moves nothing — what matters
// is that it was not blocked.)
{
    const s = makeSensor();
    const drifted = REST_TARGET - 10;
    coast(s, 1, drifted, true);
    ride(s, 70, true);
    coast(s, 1, drifted, true);
    check(s.evaluations === 2, `two coasts 70 s apart must both be allowed, got ${s.evaluations}`);
}

// Standstill: back-to-back coasts stay unrestricted.
{
    const s = makeSensor();
    const drifted = REST_TARGET - 10;
    coast(s, 1, drifted, false);
    coast(s, 1, drifted, false);
    coast(s, 1, drifted, false);
    check(s.evaluations === 3, `standstill re-zero must stay unrestricted, got ${s.evaluations}`);
}

// A standstill correction must not arm the lockout for the ride that follows.
{
    const s = makeSensor();
    const drifted = REST_TARGET - 10;
    coast(s, 1, drifted, false);
    coast(s, 1, drifted, true);
    check(s.evaluations === 2, `standstill must not lock out the next in-ride coast, got ${s.evaluations}`);
}

// Sensor-fault detection must keep working while the lockout is blocking corrections.
{
    const s = makeSensor();
    coast(s, 1, REST_TARGET - 10, true);          // arms the lockout
    check(s.lockout > 0, 'lockout must be armed after an in-ride correction');
    coast(s, 1, REST_RAW_MAX + 500, true);        // implausible baseline during lockout
    check(s.calFault === true, 'implausible rest must still raise cal_fault while locked out');
}

// The lockout counter must not underflow or wrap over a long ride with no coasts.
{
    const s = makeSensor();
    coast(s, 1, REST_TARGET - 10, true);
    ride(s, 300, true);
    check(s.lockout === 0, `lockout must reach exactly 0 and stay there, got ${s.lockout}`);
    check(Number.isInteger(s.lockout) && s.lockout >= 0, 'lockout must never go negative');
}

// --- FW-061: port of the wheel-movement latch in src/main.c ---
// Classification must survive the whole no-pedalling episode. Sampling the speed
// at the end of a coast calls a coast that finished at a standstill a "standstill
// re-zero", and Speedx100 alone drops to zero between pulses below ~3 km/h.
const SPEED_STOP_TICKS = constant('SPEED_STOP_TICKS');
function makeLatch() { return { moved: 1, prevSpeedCounter: 0 }; }
function latchTick(l, { pasIdleTicks, speedCounter, speedX100 }) {
    if (pasIdleTicks === 0) l.moved = 0;
    if (speedCounter < l.prevSpeedCounter
        || speedX100 >= MOVING_X100
        || speedCounter < SPEED_STOP_TICKS) l.moved = 1;
    l.prevSpeedCounter = speedCounter;
    return l.moved;
}

// A coast that starts while riding and ends after the bike has stopped is still a
// ride: the zero must not be corrected as if it were a trustworthy standstill one.
{
    const l = makeLatch();
    latchTick(l, { pasIdleTicks: 0, speedCounter: 100, speedX100: 800 });   // pedalling
    for (let i = 0; i < 5; i++) {                                           // rolling, pulses arrive
        latchTick(l, { pasIdleTicks: 1000, speedCounter: 0, speedX100: 500 });
        latchTick(l, { pasIdleTicks: 1000, speedCounter: 900, speedX100: 500 });
    }
    // Rolled to a stop: no more pulses, speed reads zero, counter runs past the timeout.
    let moved = 1;
    for (let i = 0; i < 5; i++) {
        moved = latchTick(l, { pasIdleTicks: 30000, speedCounter: SPEED_STOP_TICKS + 5000, speedX100: 0 });
    }
    check(moved === 1, 'a coast that began while riding must stay classified as riding after stopping');
}

// Wheel pulses arriving while Speedx100 momentarily reads zero (below ~3 km/h).
{
    const l = makeLatch();
    const silent = SPEED_STOP_TICKS + 5000; // wheel has not pulsed for longer than the stop window
    latchTick(l, { pasIdleTicks: 0, speedCounter: silent, speedX100: 0 });  // pedalling -> reset
    check(l.moved === 0, 'latch must reset when pedalling resumes');
    latchTick(l, { pasIdleTicks: 100, speedCounter: silent + 100, speedX100: 0 });
    const moved = latchTick(l, { pasIdleTicks: 200, speedCounter: 0, speedX100: 0 }); // pulse! speed still 0
    check(moved === 1, 'a wheel pulse must count as movement even while Speedx100 reads zero');
}

// A genuine standstill still classifies as standstill.
{
    const l = makeLatch();
    latchTick(l, { pasIdleTicks: 0, speedCounter: 60000, speedX100: 0 });
    const moved = latchTick(l, { pasIdleTicks: 30000, speedCounter: 60000, speedX100: 0 });
    check(moved === 0, 'no pulses and no speed must classify as standstill');
}

// FW-061: an evaluation that changes nothing must not arm the lockout, or one
// pointless coast blocks the next correction that is actually needed.
{
    const s = makeSensor();
    coast(s, 5, REST_TARGET, true);                 // already on target -> diff 0
    check(s.lastResult === RESULT.NO_CHANGE, `expected NO_CHANGE, got ${s.lastResult}`);
    check(s.lockout === 0, 'a zero-sized step must not arm the lockout');
    coast(s, 5, REST_TARGET - 10, true);            // real drift right after
    check(s.lastResult === RESULT.APPLIED, `the next real correction must go through, got ${s.lastResult}`);
    check(s.lockout > 0, 'a real correction while moving must arm the lockout');
}

// A blocked coast must report LOCKOUT, not look like a stability problem.
{
    const s = makeSensor();
    coast(s, 5, REST_TARGET - 20, true);
    check(s.lastResult === RESULT.APPLIED, 'first in-ride coast applies');
    ride(s, 30, true);
    coast(s, 5, REST_TARGET - 20, true);
    check(s.lastResult === RESULT.LOCKOUT, `expected LOCKOUT, got ${s.lastResult}`);
    check(s.rejLockout === 1 && s.rejUnstable === 0, 'the block must be counted as lockout, not unstable');
}

// A window that never completes must be TOO_SHORT and must not touch UNSTABLE —
// otherwise "no coasts long enough" is indistinguishable from "coasts too noisy".
{
    const s = makeSensor();
    coast(s, 0.2, REST_TARGET - 10, true);
    check(s.lastResult === RESULT.TOO_SHORT, `expected TOO_SHORT, got ${s.lastResult}`);
    check(s.rejTooShort === 1 && s.rejUnstable === 0,
        'a short window must not be blamed on the stability gate');
    check(s.windowsStarted === 1 && s.windowsCompleted === 0,
        `started/completed must show the window never finished, got ${s.windowsStarted}/${s.windowsCompleted}`);
}

// Drift band boundaries: 30 in band, 31 and 40 need three consistent coasts,
// 41 is never corrected automatically but must be reported.
{
    const inBand = makeSensor();
    coast(inBand, 5, REST_TARGET - BAND_MV, false);
    check(inBand.lastResult === RESULT.APPLIED, `${BAND_MV} mV drift must correct immediately`);

    for (const drift of [BAND_MV + 1, REACQUIRE_MAX_MV]) {
        const s = makeSensor();
        coast(s, 5, REST_TARGET - drift, false);
        check(s.lastResult === RESULT.NO_CHANGE, `${drift} mV: first coast must wait for confirmation`);
        coast(s, 5, REST_TARGET - drift, false);
        check(s.lastResult === RESULT.NO_CHANGE, `${drift} mV: second coast must still wait`);
        coast(s, 5, REST_TARGET - drift, false);
        check(s.lastResult === RESULT.APPLIED,
            `${drift} mV: third consistent coast must correct, got ${s.lastResult}`);
        check(Math.abs(s.lastStep) === MAX_STEP, `${drift} mV: the step must still clamp to ${MAX_STEP} mV`);
    }

    const tooFar = makeSensor();
    coast(tooFar, 5, REST_TARGET - (REACQUIRE_MAX_MV + 1), false);
    check(tooFar.lastResult === RESULT.OUT_OF_REACQUIRE_RANGE,
        `${REACQUIRE_MAX_MV + 1} mV must report OUT_OF_REACQUIRE_RANGE, got ${tooFar.lastResult}`);
    check(tooFar.offset === 0, 'nothing beyond the reacquire range may be corrected automatically');
    check(tooFar.rejRange === 1, 'the out-of-range case must be counted, not silent');
}

// Counters are cumulative — a reading must never reset them.
{
    const s = makeSensor();
    coast(s, 5, REST_TARGET - 10, true);
    coast(s, 0.2, REST_TARGET, true);
    const snapshot = [s.windowsStarted, s.corrections, s.rejTooShort];
    check(snapshot[0] === 2 && snapshot[1] === 1 && snapshot[2] === 1,
        `cumulative counters expected 2/1/1, got ${snapshot.join('/')}`);
}

// The property this change exists for: while moving, the zero can never travel more
// than one step within any 60 s window. Simulated ride = a coast every 20 s where the
// captured rest alternates between "foot resting on the pedal" and "pedals free",
// which is exactly what walks the engage threshold around today.
{
    const run = (moving) => {
        const s = makeSensor();
        const samples = [];
        // Deterministic pseudo-random contamination of the captured rest: the sample
        // is taken at the end of the coast, just as the rider starts pressing, so how
        // much load it picks up differs every time. That is what walks the zero.
        let seed = 12345;
        const contamination = () => {
            seed = (seed * 1103515245 + 12345) & 0x7fffffff;
            return ((seed >> 16) % 41) - 20; // -20..+20 mV
        };
        for (let i = 0; i < 30; i++) {
            coast(s, 1, REST_TARGET + contamination(), moving);
            samples.push({ t: i * 21, offset: s.offset });
            ride(s, 20, moving);
        }
        let worstWindow = 0;
        for (let i = 0; i < samples.length; i++) {
            for (let j = i + 1; j < samples.length && samples[j].t - samples[i].t <= 60; j++) {
                const move = Math.abs(samples[j].offset - samples[i].offset);
                if (move > worstWindow) worstWindow = move;
            }
        }
        return { s, worstWindow };
    };
    const unrestricted = run(false);
    const limited = run(true);
    check(limited.worstWindow <= MAX_STEP,
        `while moving the zero must not travel more than ${MAX_STEP} mV per 60 s, got ${limited.worstWindow}`);
    check(limited.worstWindow < unrestricted.worstWindow,
        `the rate limit must reduce zero movement (unrestricted ${unrestricted.worstWindow} mV, ` +
        `limited ${limited.worstWindow} mV)`);
    console.log(`varying capture contamination, coast every 20 s: worst zero travel per 60 s = ` +
        `${unrestricted.worstWindow} mV unrestricted vs ${limited.worstWindow} mV while moving ` +
        `(engage threshold is 18 mV)`);
}

console.log(failures === 0 ? 'FW-058 coast re-zero: PASS' : `FW-058 coast re-zero: ${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
