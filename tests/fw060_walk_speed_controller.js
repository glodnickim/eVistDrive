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
  preloadIq: cc("WA_SPEED_PRELOAD_IQ"),
  preloadTicks: cc("WA_SPEED_PRELOAD_TICKS"),
  breakawayIq: cc("WA_SPEED_BREAKAWAY_IQ"),
  breakawayTicks: cc("WA_SPEED_BREAKAWAY_TICKS"),
  handoverIq: cc("WA_SPEED_START_HANDOVER_IQ"),
  seedMaxIq: cc("WA_SPEED_START_SEED_MAX_IQ"),
  startFullPct: cc("WA_SPEED_START_FULL_PCT"),
  startDonePct: cc("WA_SPEED_START_DONE_PCT"),
  startRiseQ: cc("WA_SPEED_START_RISE_STEP_Q"),
  runRiseQ: cc("WA_SPEED_RUN_RISE_STEP_Q"),
  fallQ: cc("WA_SPEED_FALL_STEP_Q"),
  trackMarginIq: cc("WA_SPEED_TRACK_MARGIN_IQ"),
  hallLossUnwindQ: cc("WA_SPEED_HALL_LOSS_UNWIND_Q"),
  absMaxIq: mc("WA_MOTOR_IQ_ABS_MAX"),
  safeLimitIq: mc("WA_MOTOR_SAFE_LIMIT_IQ"),
  reacquireIq: mc("WA_MOTOR_REACQUIRE_IQ"),
  reacquireTicks: mc("WA_MOTOR_REACQUIRE_TICKS"),
  hallLossDriveIq: mc("WA_MOTOR_HALL_LOSS_DRIVE_IQ"),
  targetRpmDefault: mc("WA_MOTOR_TARGET_RPM_DEFAULT"),
  targetRpmMin: mc("WA_MOTOR_TARGET_RPM_MIN"),
  targetRpmMax: mc("WA_MOTOR_TARGET_RPM_MAX"),
  runMinIq: mc("WA_MOTOR_RUN_MIN_IQ"),
  runMaxIq: mc("WA_MOTOR_RUN_MAX_IQ"),
  zeroIqOffsetRpm: mc("WA_MOTOR_ZERO_IQ_OFFSET_RPM"),
  coastHysteresisRpm: mc("WA_MOTOR_COAST_HYSTERESIS_RPM"),
  coastNoHallTicks: mc("WA_MOTOR_COAST_NO_HALL_TICKS"),
};

const q = 1 << C.qShift;
const iqPerSecond = (stepQ) => (stepQ * 4000) / q;
const rpmToErps = (rpm) => Math.round((rpm * 4) / 3);

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function mapClamped(value, inLow, inHigh, outLow, outHigh) {
  if (value <= inLow) return outLow;
  if (value >= inHigh) return outHigh;
  return (
    outLow +
    Math.trunc(((value - inLow) * (outHigh - outLow)) / (inHigh - inLow))
  );
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

  startupFloor(targetErps, measuredErps, hallValid) {
    if (this.startupComplete) return 0;
    const timeFloor =
      this.ticks <= C.preloadTicks
        ? C.preloadIq
        : mapClamped(
            this.ticks,
            C.preloadTicks,
            C.breakawayTicks,
            C.preloadIq,
            C.breakawayIq
          );
    const speedFull = Math.max(
      2,
      Math.trunc((targetErps * C.startFullPct) / 100)
    );
    const speedDone = Math.max(
      speedFull + 1,
      Math.trunc((targetErps * C.startDonePct) / 100)
    );
    if (!hallValid || measuredErps <= speedFull) return timeFloor;
    return mapClamped(
      measuredErps,
      speedFull,
      speedDone,
      timeFloor,
      C.handoverIq
    );
  }

  update({
    targetErps = 67,
    measuredErps = 0,
    hallValid = false,
    reacquire = false,
    forceCoast = false,
    ceiling = C.absMaxIq,
    runMax = C.runMaxIq,
    iqFloor,
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
    const doneErps = Math.max(
      2,
      Math.trunc((targetErps * C.startDonePct) / 100)
    );
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
      if (forceCoast) {
        this.integralQ = 0;
        this.desiredIq = 0;
      } else if (!this.startupComplete) {
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

    if (forceCoast) {
      this.integralQ = 0;
      this.desiredIq = 0;
    }

    const startupIq = forceCoast
      ? 0
      : this.startupFloor(targetErps, measuredErps, hallValid);
    const finalFloor =
      iqFloor === undefined
        ? this.startupComplete && !reacquire && !forceCoast
          ? C.runMinIq
          : 0
        : iqFloor;
    let desiredBeforeLimit = forceCoast ? 0 : this.desiredIq;
    if (!forceCoast) {
      desiredBeforeLimit = Math.max(
        desiredBeforeLimit,
        startupIq,
        finalFloor
      );
    }
    const desiredCeiling = this.startupComplete
      ? Math.min(ceiling, runMax)
      : ceiling;
    const desired = clamp(desiredBeforeLimit, 0, desiredCeiling);
    const targetQ = desired * q;
    const riseQ = this.startupComplete ? C.runRiseQ : C.startRiseQ;
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
      coastRequested: forceCoast,
      floorIq: finalFloor,
    };
  }
}

class GovernorModel {
  constructor(targetRpm = C.targetRpmDefault) {
    assert(targetRpm >= C.targetRpmMin && targetRpm <= C.targetRpmMax);
    this.zeroIqRpm = targetRpm + C.zeroIqOffsetRpm;
    this.coastResumeRpm =
      this.zeroIqRpm - C.coastHysteresisRpm;
    this.zeroIqErps = rpmToErps(this.zeroIqRpm);
    this.coastResumeErps = rpmToErps(this.coastResumeRpm);
    this.coast = false;
    this.noHallTicks = 0;
    this.reacquire = false;
  }

  update(hallValid, erps) {
    this.reacquire = false;
    if (hallValid && erps >= this.zeroIqErps) {
      this.coast = true;
      this.noHallTicks = 0;
    }
    if (this.coast) {
      if (hallValid) {
        this.noHallTicks = 0;
        if (erps <= this.coastResumeErps) this.coast = false;
      } else {
        this.noHallTicks++;
        if (this.noHallTicks >= C.coastNoHallTicks) {
          this.coast = false;
          this.noHallTicks = 0;
          this.reacquire = true;
        }
      }
    }
    return { coast: this.coast, reacquire: this.reacquire };
  }
}

// Architecture and integration invariants.
assert(motorHeader.includes("WA_STATE_REGULATE"));
assert(motorHeader.includes("WA_STATE_LIMIT"));
assert(motorHeader.includes("WA_STATE_STALL"));
assert(!motorHeader.includes("WA_STATE_START"));
assert(!motorHeader.includes("WA_STATE_CLOSED_LOOP"));
assert(controllerHeader.includes("bool force_coast"));
assert(controllerHeader.includes("int32_t run_iq_max"));
assert(motorSource.includes("wa_overspeed_coast"));
assert(motorSource.includes("WA_MOTOR_COAST_NO_HALL_TICKS"));
assert(motorSource.includes("target_rpm + WA_MOTOR_ZERO_IQ_OFFSET_RPM"));
assert(motorSource.includes("zero_iq_rpm - WA_MOTOR_COAST_HYSTERESIS_RPM"));
assert(!motorSource.includes("WA_MOTOR_ANTISTALL_IQ"));
assert(!motorSource.includes("regulate_iq_floor"));
assert(!mainSource.includes(".walk_current_pct"));
assert(dynamicsSource.includes("WA owns its complete Iq trajectory"));
assert(dynamicsSource.includes("if (input->immediate_cut)"));
assert(rideSource.includes("walk_was_active && !input->walk_active"));
assert(mainSource.includes("bank_toggle_pending"));

// Binding FW-067 values.
assert.strictEqual(C.runMinIq, 5);
assert.strictEqual(C.runMaxIq, 36);
assert(C.runMinIq > 0 && C.runMinIq < C.runMaxIq);
assert.strictEqual(C.breakawayIq, 80);
assert.strictEqual(C.reacquireIq, 24);
assert(C.reacquireIq < C.hallLossDriveIq);
assert.strictEqual(C.zeroIqOffsetRpm, 20);
assert.strictEqual(C.coastHysteresisRpm, 15);
assert(C.coastHysteresisRpm < C.zeroIqOffsetRpm);
assert.strictEqual(iqPerSecond(C.startRiseQ), 93.75);
assert.strictEqual(iqPerSecond(C.runRiseQ), 15.625);
assert.strictEqual(iqPerSecond(C.fallQ), 31.25);

// Every valid bank target produces target+20 coast and target+5 resume.
for (const targetRpm of [
  C.targetRpmMin,
  40,
  C.targetRpmDefault,
  C.targetRpmMax,
]) {
  const targetGovernor = new GovernorModel(targetRpm);
  assert.strictEqual(
    targetGovernor.zeroIqRpm,
    targetRpm + C.zeroIqOffsetRpm
  );
  assert.strictEqual(
    targetGovernor.coastResumeRpm,
    targetRpm +
      C.zeroIqOffsetRpm -
      C.coastHysteresisRpm
  );
  assert(targetGovernor.coastResumeErps < targetGovernor.zeroIqErps);
}

// One-shot START: energetic but still ramped, reaching 80 Iq around 0.85 s.
const start = new ControllerModel();
let startResult;
let startIqAt450ms = 0;
let firstFullStartTick = -1;
let previousIq = 0;
let maxStartTickRise = 0;
for (let tick = 0; tick < 5000; tick++) {
  startResult = start.update();
  maxStartTickRise = Math.max(
    maxStartTickRise,
    startResult.iq - previousIq
  );
  previousIq = startResult.iq;
  if (tick === 1799) startIqAt450ms = startResult.iq;
  if (firstFullStartTick < 0 && startResult.iq >= C.breakawayIq) {
    firstFullStartTick = tick + 1;
  }
}
assert(
  startIqAt450ms >= 40 && startIqAt450ms <= 44,
  `START reached ${startIqAt450ms} Iq at 0.45 s`
);
assert(
  firstFullStartTick >= 3300 && firstFullStartTick <= 3500,
  `full START arrived at tick ${firstFullStartTick}`
);
assert.strictEqual(startResult.iq, C.breakawayIq);
assert(maxStartTickRise <= 1);
assert(startResult.startupActive);

// Handover above 30% target ends START once and descends through the RUN slew.
const handover = new ControllerModel();
handover.commandQ = 70 * q;
handover.desiredIq = 80;
handover.ticks = 1000;
const handoverFirst = handover.update({
  measuredErps: 25,
  hallValid: true,
  downstream: 70,
});
assert(!handoverFirst.startupActive);
assert.strictEqual(handoverFirst.integralIq, C.seedMaxIq);
assert(
  handoverFirst.iq >= 69,
  `RUN ceiling hard-clamped handover to ${handoverFirst.iq} Iq`
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

// RUN increases only inside 5..36 Iq and needs about two seconds for the span.
const runRise = new ControllerModel();
runRise.startupComplete = true;
runRise.commandQ = C.runMinIq * q;
runRise.desiredIq = C.runMinIq;
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
  runIqAt1s >= 20 && runIqAt1s <= 22,
  `RUN rose to ${runIqAt1s} Iq in one second`
);
assert(
  runFullTick >= 10000 && runFullTick <= 10800,
  `RUN span completed at tick ${runFullTick}`
);

// Normal reduction is smooth: roughly one second from 36 to the 5 Iq floor.
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
  if (runFallTick < 0 && runFallResult.iq <= C.runMinIq) {
    runFallTick = tick + 1;
  }
}
assert.strictEqual(runFallResult.iq, C.runMinIq);
assert(
  runFallTick >= 3900 && runFallTick <= 4100,
  `RUN reduction reached its floor at tick ${runFallTick}`
);
assert(!runFallResult.coastRequested);

// Mild overspeed below target+20 rpm is a soft error, never a zero-Iq coast.
const softOverspeed = new ControllerModel();
softOverspeed.startupComplete = true;
softOverspeed.commandQ = 20 * q;
softOverspeed.desiredIq = 20;
softOverspeed.integralQ = 20 * q;
softOverspeed.ticks = 1;
let softOverspeedResult;
for (let tick = 0; tick < 8000; tick++) {
  softOverspeedResult = softOverspeed.update({
    measuredErps: 90,
    hallValid: true,
  });
}
assert.strictEqual(softOverspeedResult.iq, C.runMinIq);
assert(!softOverspeedResult.coastRequested);

// At the dynamic target+20 rpm governor, coast resets I and slews to zero.
const defaultGovernor = new GovernorModel();
const hardCoast = new ControllerModel();
hardCoast.startupComplete = true;
hardCoast.commandQ = C.runMaxIq * q;
hardCoast.desiredIq = C.runMaxIq;
hardCoast.integralQ = C.runMaxIq * q;
hardCoast.ticks = 1;
const firstCoast = hardCoast.update({
  measuredErps: defaultGovernor.zeroIqErps,
  hallValid: true,
  forceCoast: true,
  iqFloor: 0,
});
assert(firstCoast.iq >= C.runMaxIq - 1, "hard governor cut current instantly");
assert.strictEqual(firstCoast.integralIq, 0);
assert(firstCoast.coastRequested);
let hardCoastResult = firstCoast;
for (let tick = 0; tick < 5000; tick++) {
  hardCoastResult = hardCoast.update({
    measuredErps: defaultGovernor.zeroIqErps,
    hallValid: true,
    forceCoast: true,
    iqFloor: 0,
  });
}
assert.strictEqual(hardCoastResult.iq, 0);
assert(!hardCoastResult.startupActive);

// Dynamic +20/+5 rpm thresholds and no-Hall handoff to gentle reacquire.
const governor = new GovernorModel();
assert(!governor.update(true, governor.zeroIqErps - 1).coast);
assert(governor.update(true, governor.zeroIqErps).coast);
assert(governor.update(true, governor.zeroIqErps - 10).coast);
assert(governor.update(true, governor.coastResumeErps + 1).coast);
assert(!governor.update(true, governor.coastResumeErps).coast);
assert(governor.update(true, governor.zeroIqErps).coast);
let governorState;
for (let tick = 0; tick < C.coastNoHallTicks - 1; tick++) {
  governorState = governor.update(false, 0);
  assert(governorState.coast);
  assert(!governorState.reacquire);
}
governorState = governor.update(false, 0);
assert(!governorState.coast);
assert(governorState.reacquire);

// Reacquire is slow, bounded, does not integrate and never re-arms START.
const reacquire = new ControllerModel();
reacquire.startupComplete = true;
reacquire.ticks = 1;
let reacquireResult;
for (let tick = 0; tick < C.reacquireTicks; tick++) {
  reacquireResult = reacquire.update({
    hallValid: false,
    reacquire: true,
    ceiling: C.reacquireIq,
    iqFloor: 0,
  });
}
assert(
  reacquireResult.iq >= C.reacquireIq - 1 &&
    reacquireResult.iq <= C.reacquireIq
);
assert.strictEqual(reacquireResult.integralIq, 0);
assert(!reacquireResult.startupActive);

// Quiet measurement noise inside the deadband cannot pump the bounded current.
const quiet = new ControllerModel();
quiet.startupComplete = true;
quiet.commandQ = 15 * q;
quiet.desiredIq = 15;
quiet.integralQ = 15 * q;
quiet.ticks = 1;
for (let tick = 0; tick < 4000; tick++) {
  const result = quiet.update({
    measuredErps: tick % 2 === 0 ? 61 : 73,
    hallValid: true,
  });
  assert.strictEqual(result.iq, 15);
  assert.strictEqual(result.integralIq, 15);
}

// A severe sudden obstruction may stop the model, but cannot demand a surge.
const obstruction = new ControllerModel();
obstruction.startupComplete = true;
obstruction.commandQ = C.runMinIq * q;
obstruction.desiredIq = C.runMinIq;
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
assert(obstructionIqAt1s <= 22);
assert(obstructionMaxIq <= C.runMaxIq);

// A realistic constant load still starts and settles without a current cycle.
const loaded = new ControllerModel();
const loadedGovernor = new GovernorModel();
let loadedSpeed = 0;
let loadedTailMin = Infinity;
let loadedTailMax = -Infinity;
let loadedTailIqMin = Infinity;
let loadedTailIqMax = -Infinity;
for (let tick = 0; tick < 120000; tick++) {
  const hallValid = loadedSpeed >= 0.5;
  const state = loadedGovernor.update(hallValid, Math.round(loadedSpeed));
  const result = loaded.update({
    measuredErps: Math.round(loadedSpeed),
    hallValid,
    forceCoast: state.coast,
    reacquire: state.reacquire,
    ceiling: state.reacquire ? C.reacquireIq : C.absMaxIq,
    iqFloor:
      loaded.startupComplete && !state.coast && !state.reacquire
        ? C.runMinIq
        : 0,
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

// On a very light drivetrain, positive Iq_min may accelerate above the soft
// target, but the dynamic +20/+5 rpm governor must bound it without switching.
const light = new ControllerModel();
light.startupComplete = true;
light.commandQ = C.runMinIq * q;
light.desiredIq = C.runMinIq;
light.ticks = 1;
const lightGovernor = new GovernorModel();
let lightSpeed = 67;
let lightMaxSpeed = lightSpeed;
let lightCoastEntries = 0;
let lightWasCoasting = false;
for (let tick = 0; tick < 240000; tick++) {
  const state = lightGovernor.update(true, Math.round(lightSpeed));
  if (state.coast && !lightWasCoasting) lightCoastEntries++;
  lightWasCoasting = state.coast;
  const result = light.update({
    measuredErps: Math.round(lightSpeed),
    hallValid: true,
    forceCoast: state.coast,
    iqFloor: state.coast ? 0 : C.runMinIq,
  });
  const resistingIq = 2;
  lightSpeed = Math.max(
    0,
    lightSpeed + (result.iq - resistingIq) / 0.5 / 4000
  );
  lightMaxSpeed = Math.max(lightMaxSpeed, lightSpeed);
}
assert(
  lightMaxSpeed <= lightGovernor.zeroIqErps + 2,
  `light drive exceeded governor at ${lightMaxSpeed.toFixed(2)} erps`
);
assert(
  lightCoastEntries >= 1 && lightCoastEntries <= 8,
  `light drive made ${lightCoastEntries} coast entries`
);

console.log(
  `FW-067 bounded WA controller: PASS ` +
    `(START=${iqPerSecond(C.startRiseQ)} Iq/s to ${C.breakawayIq}, ` +
    `RUN=${C.runMinIq}..${C.runMaxIq} Iq, ` +
    `rise=${iqPerSecond(C.runRiseQ)} Iq/s, ` +
    `fall=${iqPerSecond(C.fallQ)} Iq/s, ` +
    `coast=target+${C.zeroIqOffsetRpm}/+` +
    `${C.zeroIqOffsetRpm - C.coastHysteresisRpm} rpm)`
);
