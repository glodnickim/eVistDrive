// FW-091 host test: the speed limit a request gets must depend on the SOURCE of the
// request, never on a filtered cadence measurement.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw091_limit_source.js
//
// Ports map() from main.c, assist_limits_apply() from assist_limits.c and the latch/limit
// chain from ride_control.c. The whole point is that these three only misbehave TOGETHER,
// so they have to be tested together — layer-by-layer tests passed happily while the bike
// refused to re-engage.

'use strict';
const fs = require('fs');
const path = require('path');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

const P = (...p) => path.join(__dirname, '..', ...p);
const limitsSrc = fs.readFileSync(P('src', 'assist_limits.c'), 'utf8');
const rideSrc = fs.readFileSync(P('src', 'ride_control.c'), 'utf8');

// --- port of map() from src/main.c:2385 ---
const map = (x, inMin, inMax, outMin, outMax) => {
    if (x < inMin) return outMin;
    if (x > inMax) return outMax;
    if ((inMax - inMin) > (outMax - outMin)) {
        return Math.trunc((x - inMin) * (outMax - outMin + 1) / (inMax - inMin + 1)) + outMin;
    }
    return Math.trunc((x - inMin) * (outMax - outMin) / (inMax - inMin)) + outMin;
};

const PEDAL = 0, NON_PEDAL = 1;

// --- port of assist_limits_apply() ---
const applyLimits = (iq, i) => {
    let limited = map(i.voltageRaw, i.voltageMinRaw, i.voltageMinRaw + 176, 0, iq);
    limited = map(i.tempC, 75, 90, limited, 0);
    if (i.legal && !i.offroad && !i.walk) {
        limited = i.source === PEDAL
            ? map(i.speedX100, i.speedLimitX100, i.speedLimitX100 + 200, limited, 0)
            : map(i.speedX100, 500, 700, limited, 0);
    }
    return limited;
};

// Healthy battery and temperature unless a case says otherwise.
const env = (over = {}) => ({
    voltageRaw: 4000, voltageMinRaw: 1320, tempC: 30,
    speedX100: 1000, speedLimitX100: 2500,
    legal: true, offroad: false, walk: false, source: PEDAL, ...over,
});

// --- port of the ride_control chain (FW-091 shape) ---
// `latched` and `withoutRotation` are DERIVED from primitive rider state, never handed in.
// Taking them as inputs is what let the previous version of this test assert behaviour the
// firmware does not actually have.
const IQ_LIMIT = 700;
const STANDSTILL_STEPS = 4;      // tuning_config_start_steps() default
const STAND_CENTIKG = 70;        // ASSIST_MIN_PEDAL_LOAD_DEFAULT_CENTIKG
const ROLL_CENTIKG = 30;         // ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CENTIKG
const ROLLING_MIN_SPEED_X100 = 100;

function rideTick(s) {
    const {
        loadCentikg = 0, forwardSteps = 0, crankDirectionOk = true, cadenceRpm = 0,
        modeIq = null, throttleIq = 0, minIqPct = 2, safetyCut = false, level0 = false,
        withoutRotationEnabled = false, iqLimit = IQ_LIMIT, e = env(),
    } = s;

    // --- assist_modes: is the without-rotation branch raised? (assist_modes.c:563) ---
    // Note it never asks whether the cranks are turning — only cadence == 0 and load over
    // the STANDSTILL threshold. That is the trap FW-091 originally fell into.
    const withoutRotation =
        withoutRotationEnabled && cadenceRpm === 0 && loadCentikg > STAND_CENTIKG;

    // --- ride_control: the ride latch (ride_control.c:208-238) ---
    const rolling = e.speedX100 >= ROLLING_MIN_SPEED_X100;
    const requiredSteps = rolling ? Math.max(0, STANDSTILL_STEPS - 1) : STANDSTILL_STEPS;
    const crankMovingEnough = crankDirectionOk && forwardSteps >= requiredSteps;
    const assistOff = level0;
    const crankOk = !safetyCut && !assistOff && crankMovingEnough;
    const engageThreshold = (crankOk && rolling) ? ROLL_CENTIKG : STAND_CENTIKG;
    const latched = crankOk && loadCentikg >= engageThreshold;

    // Mode output: if nothing is given, assume the mode asked for nothing (the interesting
    // case, where only the latch floor can produce current).
    let iq = level0 ? 0 : (modeIq === null ? 0 : modeIq);
    if (!latched) iq = 0;                       // "no assist from a light touch"
    const minIq = Math.trunc((iqLimit * minIqPct + 99) / 100);
    if (latched && !safetyCut) iq = Math.max(iq, minIq);
    if (safetyCut) iq = 0;

    const pedalIq = applyLimits(iq, { ...e, source: latched ? PEDAL : NON_PEDAL });

    let throttle = 0;
    if (!safetyCut && throttleIq > 0) {
        throttle = applyLimits(throttleIq, { ...e, source: NON_PEDAL });
    }
    return {
        iq: Math.max(pedalIq, throttle), pedalIq, throttleIq: throttle,
        minIq, latched, withoutRotation,
    };
}

// Shorthand for "riding along, pedalling properly": rolling needs 3 forward steps.
const riding = (over = {}) => ({
    loadCentikg: 50, forwardSteps: 3, cadenceRpm: 0, e: env({ speedX100: 1000 }), ...over,
});

console.log('FW-091: limit source classification');

// 1. THE BUG: riding at 10 km/h with the cadence filter at zero, pedalling properly.
//    Under the old code the pedal request was classified non-pedal and zeroed outright.
{
    const r = rideTick(riding());
    check(r.latched, '1. three forward steps and load over the riding threshold latch');
    check(r.iq >= r.minIq, `1. and yield at least min_iq at 10 km/h (${r.iq} >= ${r.minIq})`);
    const old = applyLimits(r.minIq, env({ speedX100: 1000, source: NON_PEDAL }));
    check(old === 0, '1. the same demand judged non-pedal would still be zero (the old bug)');
}

// 2. Cadence must not enter into it at all — the value is now actually fed to the model.
{
    const results = [0, 5, 15, 16, 60].map((cadenceRpm) =>
        rideTick(riding({ cadenceRpm, modeIq: 120, e: env({ speedX100: 1500 }) })).iq);
    check(new Set(results).size === 1 && results[0] > 0,
        `2. the outcome does not vary with cadence (${results.join(',')})`);
    check(!/cadence_filtered_x8/.test(limitsSrc),
        '2. assist_limits no longer reads a cadence at all');
    check(/input->source == ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED/.test(limitsSrc),
        '2. it branches on the source instead');
}

// 3. Not latched (too few PAS steps, or load under the kg threshold) -> nothing.
{
    check(rideTick(riding({ forwardSteps: 2 })).iq === 0, '3. two forward steps at speed -> zero');
    check(rideTick(riding({ loadCentikg: 20 })).iq === 0, '3. load below the riding threshold -> zero');
    check(rideTick(riding({ crankDirectionOk: false })).iq === 0, '3. a reverse step disarms the latch');
}

// 4. Level 0 is off however hard you pedal.
{
    check(rideTick(riding({ modeIq: 400, loadCentikg: 300, level0: true })).iq === 0,
        '4. level 0 gives zero');
}

// 5. Safety cut (brake / backpedal / fault) beats everything, including the floor.
{
    check(rideTick(riding({ modeIq: 400, safetyCut: true })).iq === 0, '5. safety cut -> zero');
    check(rideTick(riding({ modeIq: 400, safetyCut: true, throttleIq: 400 })).iq === 0,
        '5. safety cut also kills throttle');
}

// 6. The real limiters still bite — the floor must never punch through them.
{
    check(rideTick(riding({ modeIq: 400, e: env({ speedX100: 1000, tempC: 95 }) })).iq === 0,
        '6. over-temperature -> zero despite the latch floor');
    check(rideTick(riding({ modeIq: 400, e: env({ speedX100: 1000, voltageRaw: 1300 }) })).iq === 0,
        '6. under-voltage -> zero despite the latch floor');
    check(rideTick(riding({ modeIq: 400, e: env({ speedX100: 2800 }) })).iq === 0,
        '6. above the legal speed limit -> zero despite the latch floor');
}

// 7. THE SAFETY CASE this refactor exists to protect: a latched rider must not hand the
//    throttle the pedal speed limit. Throttle is judged non-pedal whatever the latch says.
{
    const r = rideTick(riding({ throttleIq: 400 }));
    check(r.latched && r.throttleIq === 0,
        '7. throttle at 10 km/h stays zero even while the rider is latched');
    const slow = rideTick({ throttleIq: 400, e: env({ speedX100: 400 }) });
    check(slow.throttleIq > 0, '7. ...but throttle still works below 5 km/h');
    const cut = rideTick({ throttleIq: 400, e: env({ speedX100: 800 }) });
    check(cut.throttleIq === 0, '7. and is gone above 7 km/h');
}

// 8. THE REGRESSION THIS ROUND FIXED: with "assist without crank rotation" enabled, a firm
//    push mid-ride raises assist_without_rotation_active (cadence still reads 0 and the load
//    is over the STANDSTILL threshold) even though the cranks are demonstrably turning.
//    Classification must ignore that flag, or pushing HARDER would give LESS assist.
{
    const soft = rideTick(riding({ withoutRotationEnabled: true, loadCentikg: 50 }));   // < 0.70 kg
    const firm = rideTick(riding({ withoutRotationEnabled: true, loadCentikg: 150 }));  // > 0.70 kg
    check(firm.withoutRotation && !soft.withoutRotation,
        '8. a firm push does raise the without-rotation flag mid-ride');
    check(soft.iq > 0 && firm.iq > 0, '8. ...and BOTH still assist at 10 km/h');
    check(firm.iq >= soft.iq, `8. pushing harder never gives less (${soft.iq} -> ${firm.iq})`);
    check(/limits_input\.source = assist_latched \?/.test(rideSrc),
        '8. classification depends on the latch alone');
    check(!/assist_without_rotation_active\s*\)\s*\?/.test(rideSrc),
        '8. and not on the without-rotation flag');
}

// 8b. Honest record of what the firmware ACTUALLY does from a true standstill: the latch
//     cannot arm without forward crank steps, so "assist without crank rotation" produces
//     nothing in ride core. This test documents the gap rather than pretending otherwise —
//     the previous version asserted the opposite by hand-building an impossible state.
{
    const dead = rideTick({
        withoutRotationEnabled: true, loadCentikg: 300, forwardSteps: 0,
        e: env({ speedX100: 0 }),
    });
    check(dead.withoutRotation, '8b. the mode raises its flag from a dead stop');
    check(!dead.latched && dead.iq === 0,
        '8b. but the latch cannot arm without crank steps, so ride core delivers nothing');
}

// 9. Walk Assist bypasses the legal block entirely, as before.
{
    const r = rideTick({ throttleIq: 300, e: env({ walk: true, speedX100: 1000 }) });
    check(r.iq > 0, '9. walk assist is not touched by the legal speed limits');
}

// 10. The floor: 0% means off, and the rounding must actually change an outcome.
{
    const off = rideTick(riding({ minIqPct: 0 }));
    check(off.minIq === 0 && off.iq === 0, '10. Minimum Iq 0% forces no floor');
    // 1% of 700 is 7 either way — useless as evidence. A small limit is where truncation bit:
    // old (50*1)/100 = 0, new (50*1+99)/100 = 1.
    const small = rideTick(riding({ minIqPct: 1, iqLimit: 50 }));
    check(Math.trunc((50 * 1) / 100) === 0, '10. the old formula gave no floor at 1% of 50');
    check(small.minIq === 1, `10. rounding up now yields a real floor (${small.minIq})`);
}

// 11. Structural: pedal and throttle are limited by SEPARATE calls. Merging them first and
//     passing one source is what would let a latch leak the pedal limit to the throttle.
{
    const calls = rideSrc.match(/assist_limits_apply\(/g) || [];
    check(calls.length >= 2, `11. the limiter is called separately per source (${calls.length} calls)`);
    check(/throttle_iq = assist_limits_apply\(input->throttle_iq, &limits_input\)/.test(rideSrc),
        '11. throttle is limited from its own raw value');
    check(/limits_input\.source = ASSIST_LIMIT_SOURCE_NON_PEDAL;\s*\n\s*throttle_iq/.test(rideSrc),
        '11. and always as NON_PEDAL');
    check(/iq_target = \(throttle_iq > pedal_iq\) \? throttle_iq : pedal_iq;/.test(rideSrc),
        '11. the two are combined only after limiting');
}

// 12. FW-090 ships disabled — FW-091 is the fix; the fast attack waits for a ride first.
{
    const hdr = fs.readFileSync(P('inc', 'torque_input.h'), 'utf8');
    const steps = Number(hdr.match(/#define\s+TORQUE_RUN_ATTACK_STEPS\s+(\d+)U/)[1]);
    check(steps === 0, `12. the FW-090 fast attack is off by default (STEPS=${steps})`);
    const tq = fs.readFileSync(P('src', 'torque_input.c'), 'utf8');
    check(/if \(TORQUE_RUN_ATTACK_STEPS == 0U\) \{/.test(tq),
        '12. and is guarded explicitly, so STEPS=0 cannot fire on every step');
}

console.log(failures === 0 ? '\nAll FW-091 checks passed.' : `\n${failures} FW-091 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
