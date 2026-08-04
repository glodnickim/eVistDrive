"use strict";

const assert = require("assert");
const fs = require("fs");
const path = require("path");

const root = path.resolve(__dirname, "..");
const controllerSource = fs.readFileSync(
  path.join(root, "src", "walk_speed_controller.c"),
  "utf8"
);
const motorSource = fs.readFileSync(
  path.join(root, "src", "walk_assist_motor.c"),
  "utf8"
);
const controllerHeader = fs.readFileSync(
  path.join(root, "inc", "walk_speed_controller.h"),
  "utf8"
);
const motorHeader = fs.readFileSync(
  path.join(root, "inc", "walk_assist_motor.h"),
  "utf8"
);
const dynamicsSource = fs.readFileSync(
  path.join(root, "src", "assist_dynamics.c"),
  "utf8"
);
const rideSource = fs.readFileSync(
  path.join(root, "src", "ride_control.c"),
  "utf8"
);
const mainSource = fs.readFileSync(path.join(root, "src", "main.c"), "utf8");
const configSource = fs.readFileSync(path.join(root, "inc", "config.h"), "utf8");

function constant(source, name) {
  const match = source.match(
    new RegExp(`^#define\\s+${name}\\s+(-?\\d+)`, "m")
  );
  assert(match, `missing ${name}`);
  return Number(match[1]);
}

const cc = (name) => constant(controllerSource, name);
const mc = (name) => constant(motorSource, name);

const C = {
  qShift: cc("WA_SPEED_IQ_Q_SHIFT"),
  controlDiv: cc("WA_SPEED_CONTROL_DIV"),
  deadband: cc("WA_SPEED_DEADBAND_ERPS"),
  errorClamp: cc("WA_SPEED_ERROR_CLAMP_ERPS"),
  kp: cc("WA_SPEED_KP_IQ_PER_ERPS"),
  kiQ: cc("WA_SPEED_KI_STEP_Q"),
  kiUnwindQ: cc("WA_SPEED_KI_UNWIND_STEP_Q"),
  integralMax: cc("WA_SPEED_INTEGRAL_MAX_IQ"),
  startIq: cc("WA_SPEED_START_IQ"),
  seedMaxIq: cc("WA_SPEED_START_SEED_MAX_IQ"),
  startDoneErps: cc("WA_SPEED_START_DONE_ERPS"),
  startRiseQ: cc("WA_SPEED_START_RISE_STEP_Q"),
  runRiseQ: cc("WA_SPEED_RUN_RISE_STEP_Q"),
  reacquireRiseQ: cc("WA_SPEED_REACQUIRE_RISE_STEP_Q"),
  fallQ: cc("WA_SPEED_FALL_STEP_Q"),
  trackMarginIq: cc("WA_SPEED_TRACK_MARGIN_IQ"),
  hallLossUnwindQ: cc("WA_SPEED_HALL_LOSS_UNWIND_Q"),
  absMaxIq: mc("WA_MOTOR_IQ_ABS_MAX"),
  safeLimitIq: mc("WA_MOTOR_SAFE_LIMIT_IQ"),
  startMaxIq: mc("WA_MOTOR_START_MAX_IQ"),
  reacquireIq: mc("WA_MOTOR_REACQUIRE_IQ"),
  reacquireTicks: mc("WA_MOTOR_REACQUIRE_TICKS"),
  coastRecoveryIq: mc("WA_MOTOR_COAST_RECOVERY_IQ"),
  coastRecoveryTicks: mc("WA_MOTOR_COAST_RECOVERY_TICKS"),
  coastExitIq: mc("WA_MOTOR_COAST_EXIT_IQ"),
  legacyTargetRpmDefault: constant(configSource, "WALK_ASSIST_RPM_DEFAULT"),
  hallLossDriveIq: mc("WA_MOTOR_HALL_LOSS_DRIVE_IQ"),
  jamCmdIq: mc("WA_MOTOR_JAM_CMD_IQ"),
  jamActualIq: mc("WA_MOTOR_JAM_ACTUAL_IQ"),
  targetRpmDefault: mc("WA_MOTOR_TARGET_RPM_DEFAULT"),
  targetRpmMin: mc("WA_MOTOR_TARGET_RPM_MIN"),
  targetRpmMax: mc("WA_MOTOR_TARGET_RPM_MAX"),
  erpsPerRpmNum: mc("WA_ERPS_PER_RPM_NUM"),
  erpsPerRpmDen: mc("WA_ERPS_PER_RPM_DEN"),
  runMaxIq: mc("WA_MOTOR_RUN_MAX_IQ"),
  runMinIq: mc("WA_MOTOR_RUN_MIN_IQ"),
};

const q = 1 << C.qShift;
const iqPerSecond = (stepQ) => (stepQ * 4000) / q;
const rpmToErps = (rpm) =>
  Math.trunc(
    (rpm * C.erpsPerRpmNum + Math.trunc(C.erpsPerRpmDen / 2)) /
      C.erpsPerRpmDen
  );
function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

class ControllerModel {
  constructor() {
    this.integralQ = 0;
    this.commandQ = 0;
    this.desiredIq = 0;
    this.ticks = 0;
    this.divider = 0;
    this.startupComplete = false;
    this.reacquireActive = false;
  }

  effectiveError(raw) {
    let error = 0;
    if (raw > C.deadband) error = raw - C.deadband;
    else if (raw < -C.deadband) error = raw + C.deadband;
    return clamp(error, -C.errorClamp, C.errorClamp);
  }

  startupFloor() {
    return this.startupComplete ? 0 : C.startIq;
  }

  update({
    targetErps = 67,
    measuredErps = 0,
    hallValid = false,
    reacquire = false,
    ceiling = C.absMaxIq,
    runMin = C.runMinIq,
    runMax = C.runMaxIq,
    downstream,
  } = {}) {
    const finalDownstream =
      downstream === undefined
        ? Math.trunc((this.commandQ + q / 2) / q)
        : clamp(downstream, 0, ceiling);
    if (this.ticks === 0) this.commandQ = finalDownstream * q;
    this.ticks++;

    const reacquireEntered = reacquire && !this.reacquireActive;
    this.reacquireActive = reacquire;
    if (reacquireEntered) {
      this.integralQ = 0;
      this.desiredIq = 0;
      this.divider = C.controlDiv - 1;
    }

    let currentIq = Math.trunc((this.commandQ + q / 2) / q);
    const downstreamLimited =
      finalDownstream + C.trackMarginIq < currentIq;
    if (downstreamLimited) {
      this.commandQ =
        (finalDownstream + C.trackMarginIq) * q;
      currentIq = finalDownstream + C.trackMarginIq;
    }

    const rawError = targetErps - measuredErps;
    const effective = this.effectiveError(rawError);
    const pIq = effective * C.kp;
    const doneErps = C.startDoneErps;
    let justCompleted = false;
    if (!this.startupComplete && hallValid && measuredErps >= doneErps) {
      this.startupComplete = true;
      justCompleted = true;
      this.integralQ = clamp(
        (currentIq - pIq) * q,
        0,
        C.seedMaxIq * q
      );
    }

    this.divider++;
    if (this.divider >= C.controlDiv) {
      this.divider = 0;
      if (!this.startupComplete) {
        this.integralQ = 0;
      } else if (!hallValid) {
        if (!reacquire) {
          this.integralQ = Math.max(
            0,
            this.integralQ - C.hallLossUnwindQ
          );
        }
      } else if (!justCompleted) {
        const integralIq = Math.trunc(this.integralQ / q);
        const unsaturated = pIq + integralIq;
        const integrationCeiling = Math.min(ceiling, runMax);
        const highSaturated =
          unsaturated >= integrationCeiling && effective > 0;
        const lowSaturated =
          this.integralQ === 0 && effective < 0;
        if (!highSaturated && !lowSaturated) {
          this.integralQ +=
            effective * (effective < 0 ? C.kiUnwindQ : C.kiQ);
        }
        this.integralQ = clamp(
          this.integralQ,
          0,
          C.integralMax * q
        );
      }

      let integralIq = Math.trunc(this.integralQ / q);
      if (hallValid) {
        const trackedMax = Math.max(
          0,
          finalDownstream + C.trackMarginIq - pIq
        );
        if (this.integralQ > trackedMax * q && downstreamLimited) {
          this.integralQ = trackedMax * q;
          integralIq = trackedMax;
        }
        this.desiredIq = pIq + integralIq;
      } else if (reacquire) {
        this.desiredIq = ceiling;
      } else if (this.startupComplete) {
        this.desiredIq = 0;
      } else {
        this.desiredIq = pIq;
      }
    }

    const startupIq = this.startupFloor();
    let desiredBeforeLimit = Math.max(
      this.desiredIq,
      startupIq
    );
    if (this.startupComplete && hallValid && !reacquire) {
      desiredBeforeLimit = Math.max(desiredBeforeLimit, runMin);
    }
    const desiredCeiling = this.startupComplete
      ? Math.min(ceiling, runMax)
      : ceiling;
    const desired = clamp(desiredBeforeLimit, 0, desiredCeiling);
    const targetQ = desired * q;
    const riseQ = !this.startupComplete
      ? C.startRiseQ
      : reacquire
        ? C.reacquireRiseQ
        : C.runRiseQ;
    if (this.commandQ < targetQ) {
      this.commandQ += Math.min(targetQ - this.commandQ, riseQ);
    } else if (this.commandQ > targetQ) {
      this.commandQ -= Math.min(this.commandQ - targetQ, C.fallQ);
    }
    this.commandQ = clamp(this.commandQ, 0, ceiling * q);
    currentIq = Math.trunc((this.commandQ + q / 2) / q);

    return {
      iq: currentIq,
      integralIq: Math.trunc(this.integralQ / q),
      startupIq,
      startupActive: !this.startupComplete,
      effective,
      aboveTarget: rawError < 0,
    };
  }
}

// Architecture and integration invariants.
assert(motorHeader.includes("WA_STATE_REGULATE"));
assert(motorHeader.includes("WA_STATE_LIMIT"));
assert(motorHeader.includes("WA_STATE_STALL"));
assert(!motorHeader.includes("WA_STATE_START"));
assert(!motorHeader.includes("WA_STATE_CLOSED_LOOP"));
assert(!controllerHeader.includes("force_coast"));
assert(controllerHeader.includes("int32_t run_iq_max"));
assert(controllerHeader.includes("int32_t run_iq_min"));
assert(!controllerHeader.includes("iq_floor"));
assert(!motorSource.includes("wa_overspeed_coast"));
assert(!motorSource.includes("WA_MOTOR_ZERO_IQ_OFFSET_RPM"));
assert(!motorSource.includes("WA_MOTOR_COAST_HYSTERESIS_RPM"));
assert(!controllerSource.includes("coast_requested"));
assert(!motorSource.includes("WA_MOTOR_ANTISTALL_IQ"));
assert(!motorSource.includes("regulate_iq_floor"));
assert(motorSource.includes("wa_coast_expected || wa_coast_recovery_active"));
assert(motorSource.includes("wa_controller.desired_iq <= 0"));
assert(
  motorSource.indexOf("wa_coast_expected || wa_coast_recovery_active") <
    motorSource.indexOf("wa_reacquire_active || !drive_without_hall")
);
assert(!mainSource.includes(".walk_current_pct"));
assert(dynamicsSource.includes("WA owns its complete Iq trajectory"));
assert(dynamicsSource.includes("if (input->immediate_cut)"));
assert(rideSource.includes("walk_was_active && !input->walk_active"));
assert(mainSource.includes("bank_toggle_pending"));

// Binding FW-082 values.
assert.strictEqual(C.runMinIq, 2);
assert.strictEqual(C.runMaxIq, 40);
assert.strictEqual(C.startIq, 40);
assert.strictEqual(C.startMaxIq, 40);
assert.strictEqual(C.startDoneErps, 8);
assert.strictEqual(C.reacquireIq, 24);
assert.strictEqual(C.coastRecoveryIq, C.reacquireIq);
assert(C.jamCmdIq <= C.startMaxIq);
assert(C.jamActualIq <= C.startMaxIq);
assert.strictEqual(C.targetRpmDefault, 20);
assert.strictEqual(C.targetRpmMin, 20);
assert.strictEqual(C.legacyTargetRpmDefault, 20);
assert(C.reacquireIq < C.hallLossDriveIq);
assert.strictEqual(iqPerSecond(C.startRiseQ), 93.75);
assert.strictEqual(C.kiQ, 2);
assert.strictEqual(iqPerSecond(C.runRiseQ), 31.25);
assert.strictEqual(iqPerSecond(C.reacquireRiseQ), 31.25);
assert.strictEqual(iqPerSecond(C.fallQ), 31.25);

// One-shot START is a ramped 40 Iq motion pulse, never the former 80 Iq surge.
const start = new ControllerModel();
let startResult;
let startIqAt200ms = 0;
let firstFullStartTick = -1;
let previousIq = 0;
let maxStartTickRise = 0;
for (let tick = 0; tick < 2400; tick++) {
  startResult = start.update({ ceiling: C.startMaxIq });
  maxStartTickRise = Math.max(
    maxStartTickRise,
    startResult.iq - previousIq
  );
  previousIq = startResult.iq;
  if (tick === 799) startIqAt200ms = startResult.iq;
  if (firstFullStartTick < 0 && startResult.iq >= C.startIq) {
    firstFullStartTick = tick + 1;
  }
}
assert(
  startIqAt200ms >= 18 && startIqAt200ms <= 20,
  `START reached ${startIqAt200ms} Iq at 0.20 s`
);
assert(
  firstFullStartTick >= 1680 && firstFullStartTick <= 1700,
  `full START arrived at tick ${firstFullStartTick}`
);
assert.strictEqual(startResult.iq, C.startIq);
assert(maxStartTickRise <= 1);
assert(startResult.startupActive);

// Stable motion at 8 ERPS (about 6 chainring rpm) ends START once.
const handover = new ControllerModel();
handover.commandQ = 25 * q;
handover.desiredIq = 30;
handover.ticks = 1000;
const handoverFirst = handover.update({
  measuredErps: C.startDoneErps,
  hallValid: true,
  downstream: 25,
});
assert(!handoverFirst.startupActive);
assert.strictEqual(handoverFirst.integralIq, C.seedMaxIq);
assert(
  handoverFirst.iq >= 24 && handoverFirst.iq <= C.startMaxIq,
  `START/RUN handover produced ${handoverFirst.iq} Iq`
);
let handoverResult = handoverFirst;
for (let tick = 0; tick < 5000; tick++) {
  handoverResult = handover.update({
    measuredErps: 50,
    hallValid: true,
  });
}
assert(handoverResult.iq <= C.runMaxIq);
assert(!handoverResult.startupActive);

// RUN increases only inside 2..40 Iq with the agreed doubled response.
const runRise = new ControllerModel();
runRise.startupComplete = true;
runRise.commandQ = 0;
runRise.desiredIq = 0;
runRise.ticks = 1;
let runIqAt1s = 0;
let runFullTick = -1;
let runResult;
for (let tick = 0; tick < 12000; tick++) {
  runResult = runRise.update({
    measuredErps: 0,
    hallValid: true,
  });
  if (tick === 3999) runIqAt1s = runResult.iq;
  if (runFullTick < 0 && runResult.iq >= C.runMaxIq) {
    runFullTick = tick + 1;
  }
  assert(runResult.iq <= C.runMaxIq);
}
assert(
  runIqAt1s >= 29 && runIqAt1s <= 31,
  `RUN rose to ${runIqAt1s} Iq in one second`
);
assert(
  runFullTick >= 6000 && runFullTick <= 6100,
  `RUN span completed at tick ${runFullTick}`
);

// Normal RUN reduction is smooth but stops at the 2 Iq Hall keepalive.
const runFall = new ControllerModel();
runFall.startupComplete = true;
runFall.commandQ = C.runMaxIq * q;
runFall.desiredIq = C.runMaxIq;
runFall.integralQ = C.runMaxIq * q;
runFall.ticks = 1;
let runFallTick = -1;
let runFallResult;
for (let tick = 0; tick < 8000; tick++) {
  runFallResult = runFall.update({
    measuredErps: 90,
    hallValid: true,
  });
  if (runFallTick < 0 && runFallResult.iq === C.runMinIq) {
    runFallTick = tick + 1;
  }
}
assert.strictEqual(runFallResult.iq, C.runMinIq);
assert(
  runFallTick >= 4780 && runFallTick <= 4860,
  `RUN reduction reached keepalive at tick ${runFallTick}`
);

// Far above target normal RUN keeps exactly 2 Iq instead of losing Hall at zero.
const softOverspeed = new ControllerModel();
softOverspeed.startupComplete = true;
softOverspeed.commandQ = 20 * q;
softOverspeed.desiredIq = 20;
softOverspeed.integralQ = 20 * q;
softOverspeed.ticks = 1;
let softOverspeedResult;
for (let tick = 0; tick < 8000; tick++) {
  softOverspeedResult = softOverspeed.update({
    measuredErps: 160,
    hallValid: true,
  });
}
assert.strictEqual(softOverspeedResult.iq, C.runMinIq);

// Safety callers disable the keepalive explicitly and can still demand true zero.
const safetyZero = new ControllerModel();
safetyZero.startupComplete = true;
safetyZero.commandQ = 20 * q;
safetyZero.desiredIq = 20;
safetyZero.integralQ = 20 * q;
safetyZero.ticks = 1;
let safetyZeroResult;
for (let tick = 0; tick < 8000; tick++) {
  safetyZeroResult = safetyZero.update({
    measuredErps: 160,
    hallValid: true,
    runMin: 0,
  });
}
assert.strictEqual(safetyZeroResult.iq, 0);

// Reacquire is slow, bounded, does not integrate and never re-arms START.
const reacquire = new ControllerModel();
reacquire.startupComplete = true;
reacquire.ticks = 1;
let reacquireResult;
let reacquireIqAt200ms = 0;
let reacquireFullTick = -1;
for (let tick = 0; tick < C.reacquireTicks; tick++) {
  reacquireResult = reacquire.update({
    hallValid: false,
    reacquire: true,
    ceiling: C.reacquireIq,
  });
  if (tick === 799) reacquireIqAt200ms = reacquireResult.iq;
  if (reacquireFullTick < 0 && reacquireResult.iq >= C.reacquireIq) {
    reacquireFullTick = tick + 1;
  }
}
assert(
  reacquireIqAt200ms >= 5 && reacquireIqAt200ms <= 7,
  `reacquire jumped to ${reacquireIqAt200ms} Iq after 200 ms`
);
assert(
  reacquireResult.iq >= C.reacquireIq - 1 &&
    reacquireResult.iq <= C.reacquireIq
);
assert.strictEqual(reacquireResult.integralIq, 0);
assert(!reacquireResult.startupActive);
assert(
  reacquireFullTick > 0 &&
    C.reacquireTicks - reacquireFullTick >= 2000,
  `24 Iq reacquire has only ${C.reacquireTicks - reacquireFullTick} dwell ticks`
);

// If even the 2 Iq keepalive cannot preserve Hall, recovery remains capped at
// 24 Iq. It must reach that ceiling well before timeout and never re-arm START.
const coastRecovery = new ControllerModel();
coastRecovery.startupComplete = true;
coastRecovery.ticks = 1;
let coastRecoveryResult;
let coastRecoveryFullTick = -1;
for (let tick = 0; tick < C.coastRecoveryTicks; tick++) {
  coastRecoveryResult = coastRecovery.update({
    hallValid: false,
    reacquire: true,
    ceiling: C.coastRecoveryIq,
  });
  if (
    coastRecoveryFullTick < 0 &&
    coastRecoveryResult.iq >= C.coastRecoveryIq
  ) {
    coastRecoveryFullTick = tick + 1;
  }
}
assert.strictEqual(coastRecoveryResult.iq, C.coastRecoveryIq);
assert.strictEqual(coastRecoveryResult.integralIq, 0);
assert(!coastRecoveryResult.startupActive);
assert(
  coastRecoveryFullTick > 0 &&
    C.coastRecoveryTicks - coastRecoveryFullTick >= 4000,
  `24 Iq coast recovery has only ` +
    `${C.coastRecoveryTicks - coastRecoveryFullTick} dwell ticks`
);

// Repeated Hall loss/return cycles must keep START disarmed and normal RUN at
// 2 Iq rather than falling back to zero between recovery attempts.
const repeatedRecovery = new ControllerModel();
repeatedRecovery.startupComplete = true;
repeatedRecovery.commandQ = C.runMinIq * q;
repeatedRecovery.desiredIq = 0;
repeatedRecovery.ticks = 1;
for (let cycle = 0; cycle < 5; cycle++) {
  let cycleResult;
  for (let tick = 0; tick < 1000; tick++) {
    cycleResult = repeatedRecovery.update({
      hallValid: false,
      reacquire: true,
      ceiling: C.reacquireIq,
      runMin: 0,
    });
    assert(cycleResult.iq >= C.runMinIq);
    assert(cycleResult.iq <= C.reacquireIq);
    assert(!cycleResult.startupActive);
  }
  for (let tick = 0; tick < 2500; tick++) {
    cycleResult = repeatedRecovery.update({
      measuredErps: 160,
      hallValid: true,
    });
    assert(cycleResult.iq >= C.runMinIq);
    assert(!cycleResult.startupActive);
  }
  assert.strictEqual(cycleResult.iq, C.runMinIq);
}

// Quiet measurement noise inside the deadband cannot pump the bounded current.
const quiet = new ControllerModel();
quiet.startupComplete = true;
quiet.commandQ = 15 * q;
quiet.desiredIq = 15;
quiet.integralQ = 15 * q;
quiet.ticks = 1;
for (let tick = 0; tick < 4000; tick++) {
  const result = quiet.update({
    measuredErps: tick % 2 === 0 ? 65 : 69,
    hallValid: true,
  });
  assert.strictEqual(result.iq, 15);
  assert.strictEqual(result.integralIq, 15);
}

// A severe sudden obstruction may stop the model, but cannot demand a surge.
const obstruction = new ControllerModel();
obstruction.startupComplete = true;
obstruction.commandQ = 0;
obstruction.desiredIq = 0;
obstruction.ticks = 1;
let obstructionSpeed = 67;
let obstructionMaxIq = 0;
let obstructionIqAt1s = 0;
for (let tick = 0; tick < 12000; tick++) {
  const result = obstruction.update({
    measuredErps: Math.round(obstructionSpeed),
    hallValid: obstructionSpeed >= 0.5,
  });
  obstructionMaxIq = Math.max(obstructionMaxIq, result.iq);
  if (tick === 3999) obstructionIqAt1s = result.iq;
  const resistingIq = 42 + 0.2 * obstructionSpeed;
  obstructionSpeed = Math.max(
    0,
    obstructionSpeed + (result.iq - resistingIq) / 0.6 / 4000
  );
}
assert(
  obstructionIqAt1s >= 28 && obstructionIqAt1s <= 32,
  `obstruction response reached ${obstructionIqAt1s} Iq after 1 s`
);
assert(obstructionMaxIq <= C.runMaxIq);

// A realistic constant load still starts and settles without a current cycle.
const loaded = new ControllerModel();
let loadedSpeed = 0;
let loadedTailMin = Infinity;
let loadedTailMax = -Infinity;
let loadedTailIqMin = Infinity;
let loadedTailIqMax = -Infinity;
for (let tick = 0; tick < 120000; tick++) {
  const hallValid = loadedSpeed >= 0.5;
  const result = loaded.update({
    measuredErps: Math.round(loadedSpeed),
    hallValid,
  });
  const resistingIq = 9 + 0.28 * loadedSpeed;
  loadedSpeed = Math.max(
    0,
    loadedSpeed + (result.iq - resistingIq) / 0.9 / 4000
  );
  if (tick >= 100000) {
    loadedTailMin = Math.min(loadedTailMin, loadedSpeed);
    loadedTailMax = Math.max(loadedTailMax, loadedSpeed);
    loadedTailIqMin = Math.min(loadedTailIqMin, result.iq);
    loadedTailIqMax = Math.max(loadedTailIqMax, result.iq);
  }
}
assert(
  loadedSpeed >= 55 && loadedSpeed <= 78,
  `loaded drive settled at ${loadedSpeed.toFixed(2)} erps`
);
assert(
  loadedTailMax - loadedTailMin < 4,
  `loaded speed pumps by ${(loadedTailMax - loadedTailMin).toFixed(2)} erps`
);
assert(loadedTailIqMin >= C.runMinIq);
assert(loadedTailIqMax <= C.runMaxIq);

// A light drivetrain must settle around each requested speed. In FW-074 both
// 30 and 50 rpm collapsed to the same ~92 ERPS equilibrium because the forced
// 5 Iq floor remained active even above target. Each run below starts from
// rest and includes the full one-shot START trajectory.
function simulateLightTarget(targetRpm) {
  const targetErps = rpmToErps(targetRpm);
  const controller = new ControllerModel();
  let speed = 0;
  let minIq = Infinity;
  let maxTickDelta = 0;
  let previousIq = 0;
  let tailMin = Infinity;
  let tailMax = -Infinity;
  let tailIqMin = Infinity;
  for (let tick = 0; tick < 240000; tick++) {
    const result = controller.update({
      targetErps,
      measuredErps: Math.round(speed),
      hallValid: speed >= 0.5,
    });
    minIq = Math.min(minIq, result.iq);
    maxTickDelta = Math.max(
      maxTickDelta,
      Math.abs(result.iq - previousIq)
    );
    previousIq = result.iq;
    // Even a very light real drivetrain has speed-dependent iron/bearing drag.
    // Keeping it flat at exactly 2 Iq would make the new 2 Iq keepalive cancel
    // the model load at every speed and create an unphysical neutral equilibrium.
    const resistingIq = 2 + speed * 0.02;
    speed = Math.max(
      0,
      speed + (result.iq - resistingIq) / 0.5 / 4000
    );
    if (tick >= 220000) {
      tailMin = Math.min(tailMin, speed);
      tailMax = Math.max(tailMax, speed);
      tailIqMin = Math.min(tailIqMin, result.iq);
    }
  }
  return {
    targetErps,
    speed,
    minIq,
    maxTickDelta,
    tailMin,
    tailMax,
    tailIqMin,
  };
}

const light30 = simulateLightTarget(30);
const light50 = simulateLightTarget(50);
for (const lightResult of [light30, light50]) {
  assert(
    Math.abs(lightResult.speed - lightResult.targetErps) <= C.deadband + 2,
    `light drive target ${lightResult.targetErps} settled at ` +
      `${lightResult.speed.toFixed(2)} erps`
  );
  assert(
    lightResult.tailMax - lightResult.tailMin < 6,
    `light drive target ${lightResult.targetErps} pumps by ` +
      `${(lightResult.tailMax - lightResult.tailMin).toFixed(2)} erps`
  );
  assert(lightResult.tailIqMin >= C.runMinIq);
  assert(lightResult.maxTickDelta <= 1);
}
assert(
  light50.speed - light30.speed >= 20,
  `30/50 rpm targets collapsed to ${light30.speed.toFixed(2)}/` +
    `${light50.speed.toFixed(2)} erps`
);

console.log(
  `FW-082 faster WA load response: PASS ` +
    `(START=${iqPerSecond(C.startRiseQ)} Iq/s to ${C.startIq}, ` +
    `RUN=${C.runMinIq}..${C.runMaxIq} Iq, ` +
    `rise=${iqPerSecond(C.runRiseQ)} Iq/s, ` +
    `recovery=${iqPerSecond(C.reacquireRiseQ)} Iq/s, ` +
    `fall=${iqPerSecond(C.fallQ)} Iq/s, ` +
    `30/50 rpm=${light30.speed.toFixed(1)}/${light50.speed.toFixed(1)} ERPS)`
);
