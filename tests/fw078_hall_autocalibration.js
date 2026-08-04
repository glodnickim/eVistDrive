// FW-078 host regression: Hall calibration must re-arm PWM between phases.
// Run from BAFANG_GD32F303RCT6/: node tests/fw078_hall_autocalibration.js

"use strict";

const assert = require("assert");
const fs = require("fs");
const path = require("path");

const root = path.resolve(__dirname, "..");
const mainSource = fs.readFileSync(path.join(root, "src", "main.c"), "utf8");

const autodetectStart = mainSource.indexOf("void autodetect(void)");
const autodetectEnd = mainSource.indexOf("void ADC0_1_IRQHandler", autodetectStart);
assert(autodetectStart >= 0 && autodetectEnd > autodetectStart,
  "autodetect function not found");
const autodetect = mainSource.slice(autodetectStart, autodetectEnd);

const phase1Disable = autodetect.lastIndexOf(
  "timer_primary_output_config(TIMER0,DISABLE)"
);
const phase2Select = autodetect.indexOf("MS.hall_angle_detect_flag = 2");
assert(phase1Disable >= 0 && phase2Select > phase1Disable,
  "phase-1 disable must precede phase-2 selection");

const transition = autodetect.slice(phase1Disable, phase2Select);
for (const required of [
  "ui_8_PWM_ON_Flag=0",
  "pwm_cutoff_active=0",
  "pwm_cutoff_tick=0",
  "uint16_half_rotation_counter=0",
  "temp6=0",
  "p=0",
]) {
  assert(transition.includes(required),
    `phase transition must reset ${required}`);
}

const phase2Start = mainSource.indexOf("if(MS.hall_angle_detect_flag>1)");
assert(phase2Start >= 0, "phase-2 calibration block not found");
const phase2End = mainSource.indexOf("return MS.i_q_setpoint_temp", phase2Start);
assert(phase2End > phase2Start, "phase-2 calibration block end not found");
const phase2 = mainSource.slice(phase2Start, phase2End);

assert(/if\s*\(p>70\s*&&\s*ui_8_PWM_ON_Flag\)/.test(phase2),
  "phase 2 must not converge or persist while PWM is marked off");

const completionDisable = phase2.indexOf(
  "timer_primary_output_config(TIMER0,DISABLE)"
);
const persist = phase2.indexOf("write_virtual_eeprom()", completionDisable);
assert(completionDisable >= 0 && persist > completionDisable,
  "phase-2 completion disable/persist sequence not found");
const completion = phase2.slice(completionDisable, persist);
for (const required of [
  "ui_8_PWM_ON_Flag=0",
  "pwm_cutoff_active=0",
  "pwm_cutoff_tick=0",
  "uint16_half_rotation_counter=0",
]) {
  assert(completion.includes(required),
    `phase-2 completion must reset ${required} before persistence`);
}

// The old behaviour accidentally depended on the rotor-stop timeout being
// shorter than phase-2 convergence. The explicit resets above are the contract;
// this test intentionally remains valid if that unrelated timeout changes.

console.log("FW-078 Hall autocalibration phase handoff: PASS");
