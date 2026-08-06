// FW-092 host test: Smooth Start must not treat a coasting bike as a standstill.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw092_smooth_start_rolling.js
//
// On a mid-drive the freewheel lets the motor stand still while the bike rolls, so
// "cadence == 0 && motor_erps == 0" is exactly what ordinary coasting looks like. Judging
// the standstill from those two alone armed the launch envelope on every mid-ride
// re-engage, which put a whole smooth-start duration between the third PAS step and the
// first milliamp — undoing FW-091's promise that re-engagement depends only on PAS steps.

'use strict';
const fs = require('fs');
const path = require('path');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

const P = (...p) => path.join(__dirname, '..', ...p);
const src = fs.readFileSync(P('src', 'assist_start.c'), 'utf8');
const hdr = fs.readFileSync(P('inc', 'assist_start.h'), 'utf8');
const ride = fs.readFileSync(P('src', 'ride_control.c'), 'utf8');

const TICKS_PER_MS = 4;
const MAX_PERMILLE = Number(src.match(/#define\s+SMOOTH_START_ENVELOPE_MAX_PERMILLE\s+(\d+)U/)[1]);

// --- port of assist_start_apply_smooth() ---
// `fix` selects the FW-092 behaviour; `disarm` selects the follow-up that cancels an armed
// but unspent envelope once the bike is rolling. Modelled separately so each half can be
// shown to be necessary on its own.
class SmoothStart {
    constructor(fix = true, disarm = true) {
        this.fix = fix; this.disarm = disarm;
        this.armed = false; this.stoppedSeen = false; this.elapsed = 0;
    }
    apply(iqTarget, { cadence = 0, motorErps = 0, rolling = false, safetyCut = false }, cfg) {
        const stopped = this.fix
            ? (cadence === 0 && motorErps === 0 && !rolling)
            : (cadence === 0 && motorErps === 0);
        if (stopped && !this.stoppedSeen) { this.armed = true; this.elapsed = 0; }
        this.stoppedSeen = stopped;

        if (this.disarm && rolling && iqTarget <= 0) { this.armed = false; this.elapsed = 0; }

        if (safetyCut || iqTarget <= 0) return 0;
        if (!cfg.enabled || cfg.durationMs === 0 || !this.armed) return iqTarget;

        const durationTicks = cfg.durationMs * TICKS_PER_MS;
        if (this.elapsed < durationTicks) this.elapsed++;
        let env = Math.trunc((this.elapsed * MAX_PERMILLE) / durationTicks);
        if (env >= MAX_PERMILLE) { env = MAX_PERMILLE; this.armed = false; }
        return Math.trunc((iqTarget * env) / MAX_PERMILLE);
    }
}

// Ticks until the output first becomes non-zero.
const ticksToCurrent = (ss, iq, state, cfg, limit = 4000) => {
    for (let t = 1; t <= limit; t++) if (ss.apply(iq, state, cfg) > 0) return t;
    return null;
};

const CFG = { enabled: true, durationMs: 300 };
const COASTING = { cadence: 0, motorErps: 0, rolling: true };   // freewheeling at speed
const STANDSTILL = { cadence: 0, motorErps: 0, rolling: false };

console.log(`smooth start: envelope max ${MAX_PERMILLE} permille`);

// 1. The defect: coasting used to arm the standstill envelope.
{
    const before = new SmoothStart(false, false);         // neither half of the fix
    before.apply(0, COASTING, CFG);                       // coasting -> armed under the old rule
    const n = ticksToCurrent(before, 14, COASTING, CFG);
    check(before.armed === true || n > 1, '1. coasting armed the launch envelope before the fix');
    check(n !== null && n >= 80,
        `1. and the first current arrived ${n} ticks (~${(n / TICKS_PER_MS).toFixed(1)} ms) late`);
}

// 2. With the fix, coasting is not a standstill: current flows on the very first tick.
{
    const after = new SmoothStart(true);
    after.apply(0, COASTING, CFG);
    check(after.armed === false, '2. coasting no longer arms the envelope');
    const n = ticksToCurrent(after, 14, COASTING, CFG);
    check(n === 1, `2. re-engagement delivers current immediately (tick ${n})`);
}

// 3. A genuine standstill still gets the soft launch — that is the whole point of it.
{
    const ss = new SmoothStart(true);
    ss.apply(0, STANDSTILL, CFG);
    check(ss.armed === true, '3. a real standstill still arms the envelope');
    const n = ticksToCurrent(ss, 14, STANDSTILL, CFG);
    check(n !== null && n > 1, `3. and the launch is still eased in (first current at tick ${n})`);
}

// 4. The audit's arithmetic, held as a regression guard: at Iq=14 over 300 ms the first
//    current lands near 22 ms, and at Iq=1 the output stays zero almost the whole time.
{
    const ss = new SmoothStart(true);
    ss.apply(0, STANDSTILL, CFG);
    const n14 = ticksToCurrent(ss, 14, STANDSTILL, CFG);
    check(Math.abs(n14 / TICKS_PER_MS - 22) <= 2, `4. Iq=14 -> first current ~22 ms (${(n14 / TICKS_PER_MS).toFixed(1)})`);

    const ss1 = new SmoothStart(true);
    ss1.apply(0, STANDSTILL, CFG);
    const n1 = ticksToCurrent(ss1, 1, STANDSTILL, CFG);
    check(n1 / TICKS_PER_MS > 250, `4. Iq=1 stays dark almost the full 300 ms (${(n1 / TICKS_PER_MS).toFixed(0)} ms)`);
}

// 5. Below the rolling threshold it is still a launch — pulling away from a near-stop
//    must keep its soft start.
{
    const ss = new SmoothStart(true);
    ss.apply(0, { cadence: 0, motorErps: 0, rolling: false }, CFG);
    check(ss.armed === true, '5. not rolling is still a launch');
}

// 5b. THE SEQUENCE THAT WAS MISSING: stand still, roll away without pedalling, then pedal.
//     Arming survives the standstill -> rolling transition unless it is explicitly
//     cancelled, so the first pedal stroke at speed used to inherit a launch ramp it never
//     earned. Test 2 could not catch this: it starts from a fresh object that never armed.
{
    const stale = new SmoothStart(true, false); // FW-092 without the disarm follow-up
    stale.apply(0, STANDSTILL, CFG);
    check(stale.armed === true, '5b. standing still arms the envelope');
    for (let i = 0; i < 200; i++) stale.apply(0, COASTING, CFG); // roll away, no pedalling
    check(stale.armed === true, '5b. ...and without the disarm it survives into the roll');
    const lateTick = ticksToCurrent(stale, 14, COASTING, CFG);
    check(lateTick > 1, `5b. so the first pedal stroke was still ramped (tick ${lateTick})`);

    const fixed = new SmoothStart(true, true);
    fixed.apply(0, STANDSTILL, CFG);
    check(fixed.armed === true, '5b. with the disarm, standing still still arms it');
    fixed.apply(0, COASTING, CFG);
    check(fixed.armed === false, '5b. rolling with no demand cancels the unspent envelope');
    check(ticksToCurrent(fixed, 14, COASTING, CFG) === 1,
        '5b. and the first pedal stroke at speed gets current immediately');
}

// 5c. An envelope already running under real demand must be allowed to finish — the
//     disarm only cancels UNSPENT arming, which is why it tests iq_target.
{
    const ss = new SmoothStart(true, true);
    ss.apply(0, STANDSTILL, CFG);
    ss.apply(14, STANDSTILL, CFG);            // launch begins, envelope starts running
    const midElapsed = ss.elapsed;
    ss.apply(14, { cadence: 20, motorErps: 50, rolling: true }, CFG); // now moving, still pedalling
    check(ss.armed === true && ss.elapsed > midElapsed,
        '5c. a launch under way keeps ramping once the bike starts moving');
}

// 6. Disabled smooth start is unaffected by any of this (the factory default).
{
    const ss = new SmoothStart(true);
    const off = { enabled: false, durationMs: 300 };
    check(ss.apply(14, STANDSTILL, off) === 14, '6. disabled smooth start passes the target through');
}

// 7. Structural: "rolling" has exactly ONE owner. assist_start must hold no speed threshold
//    of its own — a second constant with the same value only looks shared until someone
//    edits one of them.
{
    check(/bool bike_rolling;/.test(hdr), '7. the input carries a ready rolling flag');
    check(/!input->bike_rolling/.test(src), '7. the standstill test uses the flag');
    check(!/SMOOTH_START_ROLLING_SPEED_X100/.test(src),
        '7. assist_start no longer defines a speed threshold of its own');
    check(/\.bike_rolling =\s*\n?\s*input->speed_x100 >= RIDE_START_REDUCTION_MIN_SPEED_X100/.test(ride),
        '7. ride_control derives it from the single constant it already owns');
    check(/if \(input->bike_rolling && input->iq_target <= 0\)/.test(src),
        '7. the unspent-envelope disarm is present and gated on there being no demand');
}

console.log(failures === 0 ? '\nAll FW-092 checks passed.' : `\n${failures} FW-092 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
