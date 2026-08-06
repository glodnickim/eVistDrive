// FW-087 host test: the start phase is an explicit flag, not a fake cadence.
//
// Run from BAFANG_GD32F303RCT6/:  node tests/fw087_start_phase.js
//
// No host C toolchain here, so this ports the two gates the old fake 1 rpm used to
// slip through — the assist gate in assist_modes.c and forward_pedaling in main.c —
// and checks the real sources for the structural guarantees the refactor rests on.
//
// The risk this test exists for: the fake cadence was load-bearing in TWO places, so
// removing it while fixing only one gate would close the launch a second way and the
// bike would simply refuse to start assisting.

'use strict';
const fs = require('fs');
const path = require('path');

let failures = 0;
const check = (ok, label) => {
    if (!ok) { failures++; console.log(`  FAIL  ${label}`); }
    return ok;
};

const main = fs.readFileSync(path.join(__dirname, '..', 'src', 'main.c'), 'utf8');
const modes = fs.readFileSync(path.join(__dirname, '..', 'src', 'assist_modes.c'), 'utf8');
const cfg = fs.readFileSync(path.join(__dirname, '..', 'inc', 'config.h'), 'utf8');
const rider = fs.readFileSync(path.join(__dirname, '..', 'inc', 'rider_input.h'), 'utf8');

// --- port of the two gates ---

// main.c: forward_pedaling drives pedaling_active, which the assist gate also checks.
const forwardPedaling = (cadence, startPhase, backwards, idleTicks, stopTimeout) =>
    (cadence > 0 || startPhase) && backwards < 4 && idleTicks <= stopTimeout;

// assist_modes.c prepare_assist_input(): returns false = no assist this tick.
const assistBlocked = (i) =>
    !i.torqueValid || !i.pasValid ||
    (!i.pedalingActive && !i.withoutRotation) ||
    (i.cadence === 0 && !i.startPhase && !i.withoutRotation);

const riding = {
    torqueValid: true, pasValid: true, pedalingActive: true,
    withoutRotation: false, startPhase: false, cadence: 60,
};

console.log('FW-087: explicit start phase');

// 1. Start phase alone, with no cadence measured yet, must pass BOTH gates.
{
    const startPhase = true;
    check(forwardPedaling(0, startPhase, 0, 10, 800) === true,
        '1. forward_pedaling is true during the start phase at cadence 0');
    check(assistBlocked({ ...riding, cadence: 0, startPhase }) === false,
        '1. assist gate passes during the start phase at cadence 0');
}

// 2. The second gate is the one easy to miss. Had forward_pedaling not been taught about
//    the start phase, pedaling_active would be false and the launch would still be blocked.
{
    const pedalingActive = forwardPedaling(0, true, 0, 10, 800);
    check(assistBlocked({ ...riding, cadence: 0, startPhase: true, pedalingActive }) === false,
        '2. both gates agree during the start phase');
    const naive = (0 > 0); // what forward_pedaling would give without the start-phase term
    check(assistBlocked({ ...riding, cadence: 0, startPhase: true, pedalingActive: naive }) === true,
        '2. without the forward_pedaling fix the launch would still be blocked');
}

// 3. A zero cadence with NO start phase and NO without-rotation is still rejected —
//    the refactor must not turn the gate into a rubber stamp.
{
    check(assistBlocked({ ...riding, cadence: 0, startPhase: false }) === true,
        '3. zero cadence without any launch state is still rejected');
    check(assistBlocked({ ...riding, cadence: 0, startPhase: false, withoutRotation: true }) === false,
        '3. without-rotation launch still passes at cadence 0');
    check(assistBlocked({ ...riding, torqueValid: false, startPhase: true }) === true,
        '3. an invalid torque sensor still blocks, start phase or not');
    check(assistBlocked({ ...riding, pasValid: false, startPhase: true }) === true,
        '3. an invalid PAS sensor still blocks, start phase or not');
}

// 4. Normal riding is untouched by the refactor.
{
    check(assistBlocked(riding) === false, '4. ordinary riding still assists');
    check(forwardPedaling(60, false, 0, 10, 800) === true, '4. ordinary riding is forward pedalling');
    check(forwardPedaling(60, false, 4, 10, 800) === false, '4. backpedalling still blocks');
    check(forwardPedaling(60, false, 0, 900, 800) === false, '4. a stop still blocks');
}

// --- structural guarantees ---

// 5. The fake cadence is gone for good.
{
    check(!/START_CADENCE_SEED_RPM/.test(cfg + main + modes),
        '5. START_CADENCE_SEED_RPM no longer exists anywhere');
    check(/START_PHASE_STEPS/.test(cfg) && /START_PHASE_ENABLE/.test(cfg),
        '5. the start phase constants replaced it');
    check(/bool start_phase;/.test(rider) && !/cadence_seeded/.test(rider),
        '5. rider_input carries start_phase, not cadence_seeded');
}

// 6. MS.cadence must only ever be assigned a real measurement or a clean zero. Any
//    placeholder assignment would reintroduce exactly what this card removed.
{
    // (?!=) so a comparison `MS.cadence==0` is not mistaken for an assignment.
    const assignments = [...main.matchAll(/MS\.cadence\s*=(?!=)\s*([^;]+);/g)].map((m) => m[1].trim());
    const offending = assignments.filter((v) => !/^10000\/\w+$/.test(v) && v !== '0');
    check(offending.length === 0,
        `6. MS.cadence only gets measurements or 0 (offending: ${JSON.stringify(offending)})`);
}

// 7. The start-phase block sets the flag and nothing else — no cadence, no filtered
//    cadence, no p_human written from a placeholder.
{
    const i = main.indexOf('#if START_PHASE_ENABLE');
    const block = main.slice(i, main.indexOf('#endif', i));
    check(/start_phase=1;/.test(block), '7. the start block raises the flag');
    check(!/MS\.cadence\s*=(?!=)/.test(block), '7. the start block does not write MS.cadence');
    check(!/uint16_cadence_filtered=/.test(block), '7. the start block does not write the filtered cadence');
    check(!/MS\.p_human=/.test(block), '7. the start block does not write p_human');
}

// 8. The flag is cleared on a real measurement and on a stop — a flag that never falls
//    would keep the launch bypasses (power ceiling, cadence comp) on for the whole ride.
{
    check(/start_phase=0;\s*\/\/FW-087: a real measurement ends the start phase/.test(main),
        '8. a real cadence measurement ends the start phase');
    check(/pas_idle_ticks>pas_stop_timeout\)\{[^}]*start_phase=0;/.test(main),
        '8. a detected stop ends the start phase');
}

// 9. forward_pedaling and the auto-off timer both know about the start phase.
{
    check(/forward_pedaling\s*=\s*\(\(MS\.cadence>0\s*\|\|\s*start_phase\)/.test(main),
        '9. forward_pedaling honours the start phase');
    check(/MS\.cadence>0\s*\|\|\s*start_phase\s*\|\|\s*MS\.i_q_setpoint>0/.test(main),
        '9. the auto-off inactivity check honours the start phase');
}

// 10. The without-rotation branch no longer fabricates a cadence either.
{
    const i = modes.indexOf('config->assist_without_rotation');
    const branch = modes.slice(i, i + 1400);
    check(!/cadence_for_assist\s*=\s*1;/.test(branch),
        '10. the without-rotation branch no longer sets a synthetic cadence');
    check(/without_rotation_active\s*=\s*true;/.test(branch),
        '10. it still raises its own flag');
}

console.log(failures === 0 ? '\nAll FW-087 checks passed.' : `\n${failures} FW-087 check(s) FAILED.`);
process.exit(failures === 0 ? 0 : 1);
