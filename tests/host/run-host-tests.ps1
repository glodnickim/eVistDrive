# Host tests - compile and RUN the shipped C modules on this PC.
#
#   powershell -File tests/host/run-host-tests.ps1
#
# Why a script: the point of these tests is that they exercise the real modules, so they need
# a compiler that produces something this machine can execute. If only the ARM cross-compiler
# is available the tests cannot RUN - a cross compile + link still proves the harness is valid
# C and that the module needs no stubs, so that is done instead and reported as SKIPPED, never
# as passed.
#
# FW-101 note: a portable toolchain now lives in C:\Projekty\tools\w64devkit, which is what
# makes these actually run. It was added because the episode recorder shipped with three
# defects that its own output could not reveal, and nothing could have caught them: every JS
# test in this repo reads C as TEXT rather than executing it.

$ErrorActionPreference = 'Stop'

# Anything a compiler writes to stderr - every gcc warning, and the whole pile of "_close is
# not implemented" notes from the nosys stubs - is turned into a terminating error by 'Stop'.
# Native tools are therefore always invoked through this, which reports the EXIT CODE and
# nothing else. build_firmware.ps1 carries the same note for the same reason.
function Invoke-Native {
    param([string]$Exe, [string[]]$Arguments, [switch]$Quiet)
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $output = & $Exe @Arguments 2>&1
        if (-not $Quiet -or $LASTEXITCODE -ne 0) {
            $output | ForEach-Object { Write-Host $_ }
        }
        return $LASTEXITCODE
    } finally { $ErrorActionPreference = $previous }
}

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$inc = Join-Path $root 'inc'
# Forward slashes so this can go straight into a -D value with no embedded quoting/escaping -
# see main_startup_wiring_host.c's own STRINGIZE() comment for why that matters here.
$mainCPathForward = (Join-Path $root 'src\main.c') -replace '\\', '/'
$mainHPathForward = (Join-Path $root 'inc\main.h') -replace '\\', '/'
$canDisplayCPathForward = (Join-Path $root 'src\CAN_Display.c') -replace '\\', '/'
$currentCalCPathForward = (Join-Path $root 'src\current_cal.c') -replace '\\', '/'
$rideControlCPathForward = (Join-Path $root 'src\ride_control.c') -replace '\\', '/'
$assistPipelineCPathForward = (Join-Path $root 'src\assist_pipeline.c') -replace '\\', '/'
$ap2LimitsCPathForward = (Join-Path $root 'src\ap2_limits.c') -replace '\\', '/'
$ap2PasStateCPathForward = (Join-Path $root 'src\ap2_pas_state.c') -replace '\\', '/'
$ap2RiderDemandCPathForward = (Join-Path $root 'src\ap2_rider_demand.c') -replace '\\', '/'
$motorCoreCPathForward = (Join-Path $root 'src\motor_core.c') -replace '\\', '/'
$focCPathForward = (Join-Path $root 'src\FOC.c') -replace '\\', '/'
$focCurrentLoopCPathForward = (Join-Path $root 'src\foc_current_loop.c') -replace '\\', '/'
$sampleWindowCPathForward = (Join-Path $root 'src\sample_window.c') -replace '\\', '/'
$armMathHPathForward = (Join-Path $root 'Firmware\CMSIS\arm_math.h') -replace '\\', '/'
$configHPathForward = (Join-Path $root 'inc\config.h') -replace '\\', '/'
$batteryCurrentCPathForward = (Join-Path $root 'src\battery_current.c') -replace '\\', '/'
$walkAssistMotorCPathForward = (Join-Path $root 'src\walk_assist_motor.c') -replace '\\', '/'

# Every harness and the module(s) it links. Add new ones here.
$suites = @(
    @{ Name = 'STOP-CLICK Hall confidence and continuous stop/restart (real modules)'
       Harness = Join-Path $PSScriptRoot 'stop_click_regression_host.c'
       Modules = @((Join-Path $root 'src\rotor_angle.c'), (Join-Path $root 'src\quiet_zero.c')) },
    @{ Name = 'STOP-TRACE passive stop capture and confirmed indexed replay'
       Harness = Join-Path $PSScriptRoot 'stop_trace_host.c'
       Modules = @(Join-Path $root 'src\stop_trace.c')
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=0') },
    @{ Name = 'FW-113.1 Walk Assist RUN minimum Iq (real walk_assist_motor + walk_speed_controller)'
       Harness = Join-Path $PSScriptRoot 'walk_assist_run_min_host.c'
       Modules = @((Join-Path $root 'src\walk_assist_motor.c'),
                   (Join-Path $root 'src\walk_speed_controller.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-113.2 Walk Assist no hold timeout + reason bits (real walk_assist_motor + walk_speed_controller)'
       Harness = Join-Path $PSScriptRoot 'walk_assist_diag_host.c'
       Modules = @((Join-Path $root 'src\walk_assist_motor.c'),
                   (Join-Path $root 'src\walk_speed_controller.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-130 Walk Assist governor: G532 ramp, centred gear band, wheel fuse (real modules)'
       # Reports SKIPPED when built with WALK_GOVERNOR_ENABLE=0 rather than passing vacuously,
       # so an A-side build cannot look green for behaviour it does not implement.
       Harness = Join-Path $PSScriptRoot 'fw130_walk_governor_host.c'
       Modules = @((Join-Path $root 'src\walk_assist_motor.c'),
                   (Join-Path $root 'src\walk_speed_controller.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-131 canonical rotor angle: continuity across the formula handover (real rotor_angle.c)'
       # L0 models the LEGACY pair of formulas alongside the new module, so the improvement is a
       # measured number: the old code stepped 24 deg even exactly on a Hall edge and up to 36 deg
       # mid-sector. Everything after it asserts the step is gone.
       Harness = Join-Path $PSScriptRoot 'fw131_rotor_angle_host.c'
       Modules = @(Join-Path $root 'src\rotor_angle.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },

    @{ Name = 'FW-137 edge-age speed ceiling (estimator replica + production wiring guards)'
       Harness = Join-Path $PSScriptRoot 'fw137_erps_edge_age_host.c'
       # No modules linked - the estimator lives in main.c's Hall ISR and 4 kHz path. The replica
       # reproduces both halves; the guards prove the ceiling is in the PERIODIC path (an ISR that
       # is not firing cannot fix a value that is stale because it is not firing) and that
       # walk_assist_motor.c never touches ui16_erps at all.
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward",
                   "-DWALK_ASSIST_MOTOR_C_PATH=$walkAssistMotorCPathForward") },

    @{ Name = 'FW-136.0 click-zone measurement (wiring guards + gating replica)'
       Harness = Join-Path $PSScriptRoot 'fw1360_click_measurement_host.c'
       # No modules linked - the measurement is inline in runPIcontrol() (main.c, the ARM entry
       # point) and the responder is in the CAN receive path, so the wiring is proven against the
       # production source and the gating against a replica pinned to it. The guards are aimed at
       # the ways a diagnostic can silently measure NOTHING, which is how FW-121.0 and FW-106 each
       # cost a ride.
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DCAN_DISPLAY_C_PATH=$canDisplayCPathForward") },

    @{ Name = 'FW-135 update-session power hold (production wiring guards over main.c/CAN_Display.c)'
       Harness = Join-Path $PSScriptRoot 'fw135_update_power_hold_host.c'
       # No modules linked - main.c is the ARM entry point and CAN_Display.c the CAN receive path
       # (same reasoning as armed_zero_lifecycle_host.c). The guards prove that a DISPLAY firmware
       # update can no longer make the controller cut its own display's supply, and - just as
       # important - that the hold which achieves that gates the SILENCE power-off ONLY, never the
       # on/off button and never the inactivity timer.
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DCAN_DISPLAY_C_PATH=$canDisplayCPathForward",
                   "-DCONFIG_H_PATH=$configHPathForward") },
    @{ Name = 'FW-101 episode recorder'
       Harness = Join-Path $PSScriptRoot 'fw101_episode_host.c'
       Modules = @(Join-Path $root 'src\ride_episode.c') },
    @{ Name = 'FW-102 pas_trace'
       Harness = Join-Path $PSScriptRoot 'fw102_pas_trace_host.c'
       Modules = @(Join-Path $root 'src\pas_trace.c') },
    @{ Name = 'FW-106 recorders'
       Harness = Join-Path $PSScriptRoot 'fw106_recorder_host.c'
       Modules = @((Join-Path $root 'src\ride_episode.c'),
                   (Join-Path $root 'src\pas_trace.c'),
                   (Join-Path $root 'src\pas_raw.c'))
       # PAS_RAW_COPY_HOOK: a test-only seam inside pas_raw.c (compiled out of the firmware
       # entirely - see the #ifdef there) that lets the D4 block below call the real ISR entry
       # point FROM INSIDE pas_raw_freeze()'s real copy loop, which a single-threaded test
       # cannot otherwise reach.
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1', '-DPAS_RAW_COPY_HOOK') },
    @{ Name = 'FW-106 session and dump'
       Harness = Join-Path $PSScriptRoot 'fw106_session_host.c'
       Modules = @(Join-Path $root 'src\diag_session.c')
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1') },
    @{ Name = 'FW-117.1 bridge lifecycle trace, TRIGGER_EVENT=START (real fw117_trace.c)'
       Harness = Join-Path $PSScriptRoot 'fw117_trace_host.c'
       Modules = @(Join-Path $root 'src\fw117_trace.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       # FW117_TRACE_TRIGGER_EVENT is left at its default (0 = START) here; the STOP-triggered
       # image is the separate suite entry right below, from the SAME harness source.
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1', '-DFW117_TRACE_ENABLE=1', '-DROLLING_NO_ASSIST_DIAG_ENABLE=0') },
    @{ Name = 'FW-117.1 bridge lifecycle trace, TRIGGER_EVENT=STOP (real fw117_trace.c)'
       Harness = Join-Path $PSScriptRoot 'fw117_trace_host.c'
       Modules = @(Join-Path $root 'src\fw117_trace.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1', '-DFW117_TRACE_TRIGGER_EVENT=1', '-DFW117_TRACE_ENABLE=1', '-DROLLING_NO_ASSIST_DIAG_ENABLE=0') },
    @{ Name = 'FW-117.1 TRIGGER_EVENT selector rejects an invalid value (must FAIL to build)'
       Harness = Join-Path $PSScriptRoot 'fw117_trace_bad_selector_host.c'
       Modules = @()
       Defines = @('-DFW117_TRACE_TRIGGER_EVENT=2')
       # This probe is SUPPOSED to fail to compile (inc/fw117_trace.h's #error) - see the
       # harness's own file header. ExpectBuildFailure inverts the pass/fail check below: a
       # build that succeeds here is the test FAILING, not passing.
       ExpectBuildFailure = $true },
    @{ Name = 'FW-106 integration (real modules together)'
       Harness = Join-Path $PSScriptRoot 'fw106_integration_host.c'
       Modules = @((Join-Path $root 'src\diag_session.c'),
                   (Join-Path $root 'src\pas_trace.c'),
                   (Join-Path $root 'src\pas_raw.c'),
                   (Join-Path $root 'src\ride_episode.c'))
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1') },
    @{ Name = 'FW-109 v2 pas_quadrature raw decoder (all 16 pairs)'
       Harness = Join-Path $PSScriptRoot 'pas_quadrature_host.c'
       Modules = @(Join-Path $root 'src\pas_quadrature.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'PAS sampler physical-glitch filter (real sampler + direction safety)'
       Harness = Join-Path $PSScriptRoot 'pas_sampler_glitch_filter_host.c'
       Modules = @((Join-Path $root 'src\pas_sampler.c'),
                   (Join-Path $root 'src\pas_quadrature.c'),
                   (Join-Path $root 'src\pas_direction.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-109 v2 pas_direction automaton (exhaustive + property proof)'
       Harness = Join-Path $PSScriptRoot 'pas_direction_host.c'
       Modules = @(Join-Path $root 'src\pas_direction.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-112.4 ordinary-RUN asymmetric filter (real torque_input.c, S1-S8 host comparison)'
       Harness = Join-Path $PSScriptRoot 'torque\torque_run_asym_host.c'
       # Real torque_input.c only - no rearm/session chain needed, this card's filter is scoped
       # to recovery_state == IDLE (ordinary RUN). Compares the shipped module's real ARUN
       # output against an in-harness replica of the pre-card 48-step plain moving average fed
       # the SAME real afilt stream, across S1-S8 trajectories at 20/40/60/80 rpm, plus a direct
       # cold-arm/rolling-rearm seed-parity check. See the harness's own file header.
       Modules = @((Join-Path $root 'src\torque_input.c'),
                   (Join-Path $PSScriptRoot 'common\crank_model.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @('-Wno-type-limits') },
    @{ Name = 'FW-110 can_tx_queue non-blocking TX queue'
       Harness = Join-Path $PSScriptRoot 'can_tx_queue_host.c'
       Modules = @(Join-Path $root 'src\can_tx_queue.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
@{ Name = 'FW-110 can_multiframe stop-and-wait multiframe producer'
        Harness = Join-Path $PSScriptRoot 'can_multiframe_host.c'
        # Linked TOGETHER with the real can_tx_queue.c it feeds - can_multiframe.c calls
        # can_tx_queue_enqueue_tracked() directly rather than through an injected ops table, so its
        # own correctness cannot be shown without the real queue underneath it.
        Modules = @((Join-Path $root 'src\can_multiframe.c'), (Join-Path $root 'src\can_tx_queue.c'))
        IncludeDirs = @(Join-Path $PSScriptRoot 'common')
        # CANMF_REFUSAL_HOOK: a test-only seam inside can_multiframe.c (compiled out of the
        # firmware entirely - see the #ifdef there) that lets the harness force the producer's
        # defensive enqueue-refusal abort path, which a single-threaded test cannot otherwise
        # reach because of the free-slot pre-check.
        Defines = @('-DCANMF_REFUSAL_HOOK') },
    @{ Name = 'FW-110 v4 can_reply_effects deferred 0x6029 peak reset'
       Harness = Join-Path $PSScriptRoot 'can_reply_effects_host.c'
       # Linked TOGETHER with the real can_multiframe.c (whose transfer-id/state it resolves
       # against) and the real can_tx_queue.c underneath it.
       Modules = @((Join-Path $root 'src\can_reply_effects.c'),
                   (Join-Path $root 'src\can_multiframe.c'),
                   (Join-Path $root 'src\can_tx_queue.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-109 v2 main.c startup wiring guard (pas_direction_init)'
       Harness = Join-Path $PSScriptRoot 'main_startup_wiring_host.c'
       # No modules to link - this harness reads src/main.c's own SOURCE TEXT at runtime (see
       # its file header for exactly what that does and does not prove) rather than linking it,
       # which is not feasible here: main.c is the ARM entry point, wired directly to GD32 CMSIS
       # registers and real hardware throughout.
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward") },
    @{ Name = 'FW-110 CAN blocking-wait guard (main.c / CAN_Display.c)'
       Harness = Join-Path $PSScriptRoot 'fw110_can_blocking_guard_host.c'
       # Same reasoning as the pas_direction_init wiring guard above: neither file can be linked
       # here, so this reads their SOURCE TEXT instead (see the harness's own file header).
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DCAN_DISPLAY_C_PATH=$canDisplayCPathForward") },
    @{ Name = 'EVD-WALK 0x3202 bit0 gate guard (CAN_Display.c source-text check)'
       Harness = Join-Path $PSScriptRoot 'walk3202_bit0_guard_host.c'
       # Source-text, same reasoning as the FW-110 sibling: CAN_Display.c cannot be linked here
       # (ARM entry wired to GD32 CMSIS); the regression guards the gated bit0 semantics live in
       # the active source, so a revert to unconditional 0x00 (or a set on Walk-selected-only,
       # or on motion without Walk) is caught structurally. See the harness's own file header.
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DCAN_DISPLAY_C_PATH=$canDisplayCPathForward") },
    @{ Name = 'STEP 2A neutral-dwell wiring guard (main.c source-text check)'
       Harness = Join-Path $PSScriptRoot 'step2a_neutral_dwell_wiring_host.c'
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward") },
    @{ Name = 'Persistent ARMED_ZERO lifecycle (host model + production wiring guards)'
       Harness = Join-Path $PSScriptRoot 'armed_zero_lifecycle_host.c'
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DMAIN_H_PATH=$mainHPathForward", "-DFOC_C_PATH=$focCPathForward", "-DFOC_CURRENT_LOOP_C_PATH=$focCurrentLoopCPathForward") },
    @{ Name = 'FOC-AW1 D/Q voltage tracking anti-windup (real PI_control()+limiter replica + production wiring guards)'
       Harness = Join-Path $PSScriptRoot 'focaw1_tracking_aw_host.c'
       # No modules linked - main.c/FOC.c are the ARM entry point/ISR core (same reasoning as
       # stopclick_c1_pi_integral_host.c below, whose replica this suite extends). The behaviour
       # checks run against a byte-faithful replica of PI_control() AND of runPIcontrol()'s
       # circle limiter; the wiring checks are source-text guards over main.c/FOC.c/main.h.
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DMAIN_H_PATH=$mainHPathForward", "-DFOC_C_PATH=$focCPathForward", "-DFOC_CURRENT_LOOP_C_PATH=$focCurrentLoopCPathForward") },
    @{ Name = 'FOC current-loop extraction parity (real shared module vs legacy block, randomized)'
       Harness = Join-Path $PSScriptRoot 'foc_current_loop_parity_host.c'
       Modules = @(Join-Path $root 'src\foc_current_loop.c')
       IncludeDirs = @((Join-Path $PSScriptRoot 'foc_current_loop_shim'),
                       (Join-Path $PSScriptRoot 'common')) },
    @{ Name = 'STOP-CLICK-C1 PI D/Q integrator continuity (real PI_control() replica + production wiring guards)'
       Harness = Join-Path $PSScriptRoot 'stopclick_c1_pi_integral_host.c'
       # No modules linked - main.c/FOC.c are the ARM entry point/ISR core (same reasoning as
       # armed_zero_lifecycle_host.c above); T1/T4-T10 are proven against their source text,
       # T2/T3/boundedness against a byte-faithful replica of FOC.c's PI_control() pinned to the
       # real body by its own model-matches-production source guard.
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DFOC_C_PATH=$focCPathForward") },
    @{ Name = 'FW-139 startup has one current-trajectory owner (no Hall-gated preload)'
       Harness = Join-Path $PSScriptRoot 'fw139_start_trajectory_host.c'
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DRIDE_CONTROL_C_PATH=$rideControlCPathForward") },
    @{ Name = 'FW-140 conditioned control cadence (real filter + main wiring guard)'
       Harness = Join-Path $PSScriptRoot 'fw140_control_cadence_host.c'
       Modules = @(Join-Path $root 'src\cadence_filter.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward") },
    @{ Name = 'FW-141 torque filters use elapsed hardware time (real torque_input.c + main wiring)'
       Harness = Join-Path $PSScriptRoot 'fw141_torque_elapsed_time_host.c'
       Modules = @(Join-Path $root 'src\torque_input.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @('-Wno-type-limits') },
    @{ Name = 'QZERO Quiet Zero PI integral fade at Iq_ref=0 (real quiet_zero.c + real 16 kHz slew owner + PI_control replica + wiring guards)'
       Harness = Join-Path $PSScriptRoot 'qzero_quiet_zero_host.c'
       # Links the REAL state machine and the REAL 16 kHz slew owner, so the entry edge is the
       # production definition of "Iq_ref first became exactly 0" rather than a hand-written one.
       # The effect on the regulators is measured against a byte-faithful PI_control() replica
       # (main.c/FOC.c are the ARM entry point/ISR core - same reasoning as
       # stopclick_c1_pi_integral_host.c, whose replica this suite reuses); the wiring is proven
       # by source-text guards over main.c and ride_control.c.
       Modules = @((Join-Path $root 'src\quiet_zero.c'),
                   (Join-Path $root 'src\fast_iq_slew.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
        Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DFOC_C_PATH=$focCPathForward",
                   "-DFOC_CURRENT_LOOP_C_PATH=$focCurrentLoopCPathForward",
                   "-DRIDE_CONTROL_C_PATH=$rideControlCPathForward",
                   "-DASSIST_PIPELINE_C_PATH=$assistPipelineCPathForward") },
    @{ Name = 'QS-3D 16 kHz final Iq slew parity (real 16 kHz owner + real 4 kHz owner, lockstep)'
       Harness = Join-Path $PSScriptRoot 'qs3d_16khz_slew_host.c'
       # Links the REAL 16 kHz slew owner (fast_iq_slew.c) and the REAL legacy 4 kHz owner
       # (assist_dynamics.c) and drives both in lockstep so the boundary outputs and total
       # rise/fall/release timing are parity-proven against the shipped Q8 round-half-up ramp.
       Modules = @((Join-Path $root 'src\fast_iq_slew.c'),
                   (Join-Path $PSScriptRoot 'common\map_adapter.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @('-DFAST_IQ_SLEW_TEST_HOOKS', "-DMAIN_C_PATH=$mainCPathForward",
                   "-DRIDE_CONTROL_C_PATH=$rideControlCPathForward",
                   "-DMOTOR_CORE_C_PATH=$motorCoreCPathForward", "-DFOC_C_PATH=$focCPathForward",
                   "-DASSIST_PIPELINE_C_PATH=$assistPipelineCPathForward") },
    @{ Name = 'QS-3C battery-current limiter upstream ownership (real battery_iq_cap.c + main.c/ride_control.c ownership guard)'
       Harness = Join-Path $PSScriptRoot 'qs3c_battery_cap_host.c'
       Modules = @(Join-Path $root 'src\battery_iq_cap.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DRIDE_CONTROL_C_PATH=$rideControlCPathForward", "-DAP2_LIMITS_C_PATH=$ap2LimitsCPathForward", "-DFOC_CURRENT_LOOP_C_PATH=$focCurrentLoopCPathForward") },
    @{ Name = 'FW-127A applied PWM geometry clamp (real pwm_geometry.c)'
       Harness = Join-Path $PSScriptRoot 'fw127a_pwm_geometry_host.c'
       Modules = @(Join-Path $root 'src\pwm_geometry.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-127B atomic sample context (real current_sample_ctx.c)'
       Harness = Join-Path $PSScriptRoot 'fw127b_sample_ctx_host.c'
       Modules = @(Join-Path $root 'src\current_sample_ctx.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-127C sampling window from applied geometry (real sample_window.c)'
       Harness = Join-Path $PSScriptRoot 'fw127c_sample_window_host.c'
       Modules = @((Join-Path $root 'src\sample_window.c'),
                   (Join-Path $root 'src\pwm_geometry.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-127C acquisition wiring guard (main.c source-text check)'
       Harness = Join-Path $PSScriptRoot 'fw127c_wiring_host.c'
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward") },
    @{ Name = 'FW-127D current feedback validity (real current_feedback.c)'
       Harness = Join-Path $PSScriptRoot 'fw127d_current_feedback_host.c'
       Modules = @(Join-Path $root 'src\current_feedback.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-128A canonical q-current ownership (real iq_chain.c + source guard)'
       Harness = Join-Path $PSScriptRoot 'fw128a_iq_chain_host.c'
       Modules = @(Join-Path $root 'src\iq_chain.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DRIDE_CONTROL_C_PATH=$rideControlCPathForward", "-DAP2_LIMITS_C_PATH=$ap2LimitsCPathForward", "-DFOC_CURRENT_LOOP_C_PATH=$focCurrentLoopCPathForward") },
    @{ Name = 'PRE-FW128 PAS timebase (real pas_sampler.c + pas_cadence.c + pas_quadrature.c)'
       Harness = Join-Path $PSScriptRoot 'pre128_pas_timebase_host.c'
       Modules = @((Join-Path $root 'src\pas_sampler.c'), (Join-Path $root 'src\pas_cadence.c'), (Join-Path $root 'src\pas_quadrature.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-128B1 battery current timebase (real battery_current.c + main.c source guard)'
       Harness = Join-Path $PSScriptRoot 'fw128b1_battery_timebase_host.c'
       Modules = @(Join-Path $root 'src\battery_current.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward") },
    @{ Name = 'FW-128B0 battery current scale (model + main.c/config.h source guard)'
       Harness = Join-Path $PSScriptRoot 'fw128b0_battery_scale_host.c'
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DCONFIG_H_PATH=$configHPathForward", "-DBATTERY_CURRENT_C_PATH=$batteryCurrentCPathForward", "-DRIDE_CONTROL_C_PATH=$rideControlCPathForward", "-DAP2_LIMITS_C_PATH=$ap2LimitsCPathForward") },
    @{ Name = 'FW-128C0 physical current scale (numeric model + arm_math/FOC/main source guard)'
       Harness = Join-Path $PSScriptRoot 'fw128c0_current_scale_host.c'
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DARM_MATH_H_PATH=$armMathHPathForward", "-DFOC_C_PATH=$focCPathForward", "-DSAMPLE_WINDOW_C_PATH=$sampleWindowCPathForward", "-DMAIN_C_PATH=$mainCPathForward") },
    @{ Name = 'FW-126.7 production current calibration (real current_cal.c)'
       Harness = Join-Path $PSScriptRoot 'fw1267_current_cal_host.c'
       Modules = @(Join-Path $root 'src\current_cal.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-126.7 calibration wiring guard (main.c source-text check)'
       Harness = Join-Path $PSScriptRoot 'fw1267_cal_wiring_host.c'
       # main.c is the ARM entry point and cannot be linked here - same reasoning as the other
       # source-text guards. The module's own behaviour is covered by the harness above.
       Modules = @()
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DCURRENT_CAL_C_PATH=$currentCalCPathForward") },
    @{ Name = 'Assist Pipeline V2 behavioural scenarios (real ap2_*/assist_pipeline/limits chain)'
       # Drives the SHIPPED assist chain with a pulsating pedal model and checks the rider-facing
       # invariants: the sustained term survives the dead spot, a reverse step zeroes the request
       # in the same tick, the profiles are ordered and differ in character, AUTO moves
       # continuously, and every limiter both binds and releases. No exact Iq value is pinned -
       # those are ride-feel settings meant to be tuned on the bike.
       Harness = Join-Path $PSScriptRoot 'ap2_pipeline_scenarios_host.c'
       Modules = @((Join-Path $root 'src\ap2_pas_state.c'),
                   (Join-Path $root 'src\ap2_rider_demand.c'),
                   (Join-Path $root 'src\ap2_estimators.c'),
                   (Join-Path $root 'src\ap2_profiles.c'),
                   (Join-Path $root 'src\ap2_limits.c'),
                   (Join-Path $root 'src\assist_pipeline.c'),
                   (Join-Path $root 'src\assist_modes.c'),
                   (Join-Path $root 'src\torque_input.c'),
                   (Join-Path $root 'src\tuning_config.c'),
                   (Join-Path $root 'src\battery_iq_cap.c'),
                   (Join-Path $root 'src\fast_iq_slew.c'))
       IncludeDirs = @((Join-Path $PSScriptRoot 'common\host_stubs'), (Join-Path $PSScriptRoot 'common'))
       Defines = @('-Wno-type-limits') },
    @{ Name = 'FW-144 SOC core exact-production parity + randomized 1 Hz transitions'
       Harness = Join-Path $PSScriptRoot 'fw144_soc_core_parity_host.c'
       Modules = @(Join-Path $root 'src\soc_core.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-145 continuous Level-4 ride telemetry (real module pacing, priority and wire schema)'
       Harness = Join-Path $PSScriptRoot 'ride_telemetry_host.c'
       Modules = @((Join-Path $root 'src\ride_telemetry.c'))
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1', '-DCAN_RIDE_TELEMETRY_ENABLE=1') }
)

function Find-HostCompiler {
    # The portable toolchain first: it is the one that is actually present on this machine.
    # Its bin directory must be on PATH for the run, or gcc cannot find `as` and `ld`.
    $portable = 'C:\Projekty\tools\w64devkit\bin\gcc.exe'
    if (Test-Path $portable) { return @{ Kind = 'gcc'; Path = $portable; Bin = (Split-Path -Parent $portable) } }
    foreach ($name in @('gcc.exe', 'clang.exe', 'cc.exe')) {
        $found = Get-Command $name -ErrorAction SilentlyContinue
        if ($found) { return @{ Kind = 'gcc'; Path = $found.Source; Bin = (Split-Path -Parent $found.Source) } }
    }
    $cl = Get-Command 'cl.exe' -ErrorAction SilentlyContinue
    if ($cl) { return @{ Kind = 'msvc'; Path = $cl.Source; Bin = (Split-Path -Parent $cl.Source) } }
    return $null
}

function Find-CrossCompiler {
    $found = Get-Command 'arm-none-eabi-gcc.exe' -ErrorAction SilentlyContinue
    if ($found) { return $found.Source }
    foreach ($candidate in @(
            'C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin\arm-none-eabi-gcc.exe',
            'C:\Program Files (x86)\GNU Tools Arm Embedded\7 2018-q2-update\bin\arm-none-eabi-gcc.exe')) {
        if (Test-Path $candidate) { return $candidate }
    }
    return $null
}

$host_cc = Find-HostCompiler
if ($host_cc) {
    Write-Host "Host compiler: $($host_cc.Path)"
    # gcc shells out to `as` and `ld` from its own bin directory.
    $env:PATH = "$($host_cc.Bin);$env:PATH"
    $failed = 0
    foreach ($s in $suites) {
        $exe = Join-Path $env:TEMP ("host_" + ($s.Name -replace '[^A-Za-z0-9]', '_') + ".exe")
        # FW-106 sizes the diagnostic buffers off CAN_DIAGNOSTICS_ENABLE, so a harness that
        # exercises them has to be built the way the diagnostic firmware is built.
        $defs = @()
        if ($s.ContainsKey('Defines')) { $defs = $s.Defines }
        # A harness that needs stub headers ahead of the real inc/ (e.g. host_stubs/gd32f30x.h
        # standing in for the real GD32 CMSIS vendor header) lists them here, in order, before
        # -I$inc - see tests/host/pipeline/ride_control_pipeline_host.c for why that harness
        # needs this and tests/host/common/host_stubs for exactly what is substituted.
        $includeDirs = @()
        if ($s.ContainsKey('IncludeDirs')) { $includeDirs = $s.IncludeDirs }
        $includeFlags = @($includeDirs | ForEach-Object { "-I$_" }) + @("-I$inc")
        if ($host_cc.Kind -eq 'gcc') {
            $built = Invoke-Native $host_cc.Path (@(
                '-std=c11', '-Wall', '-Wextra', '-Werror') + $defs + $includeFlags + @('-o', $exe, $s.Harness) + $s.Modules)
        } else {
            $msvcIncludeFlags = @($includeDirs | ForEach-Object { "/I$_" }) + @("/I$inc")
            Push-Location $env:TEMP
            try {
                $built = Invoke-Native $host_cc.Path (@(
                    '/nologo', '/W4', '/WX') + $msvcIncludeFlags + @("/Fe:$exe", $s.Harness) + $s.Modules)
            } finally { Pop-Location }
        }
        # FW-117.1: a suite that is SUPPOSED to fail to build (proving a compile-time #error
        # actually fires - see fw117_trace_bad_selector_host.c) inverts the usual pass/fail
        # check: succeeding here is the failure.
        if ($s.ContainsKey('ExpectBuildFailure') -and $s.ExpectBuildFailure) {
            if ($built -eq 0) {
                Write-Host "$($s.Name): unexpectedly BUILT (this probe must fail to compile)"
                Remove-Item $exe -ErrorAction SilentlyContinue
                $failed++
            } else {
                Write-Host "$($s.Name): correctly refused to build"
            }
            continue
        }
        if ($built -ne 0) { Write-Host "$($s.Name): harness failed to BUILD"; $failed++; continue }
        $transportFixture = $null
        $transportCsv = $null
        if ($s.ContainsKey('TransportDecoder') -and $s.TransportDecoder) {
            $transportFixture = Join-Path $env:TEMP 'rolling_no_assist_transport_host_capture.log'
            $transportCsv = Join-Path $env:TEMP 'rolling_no_assist_transport_host_capture.csv'
            Remove-Item -LiteralPath $transportFixture,$transportCsv -Force -ErrorAction SilentlyContinue
            $env:RNA_CAPTURE_FILE = $transportFixture
        }
        # A suite may pass arguments to its harness. FW-129 uses this to run in --quiet mode
        # here (checks only) while still printing its BEFORE/AFTER table when run by hand.
        $harnessArgs = if ($s.ContainsKey('Arguments') -and $s.Arguments) { $s.Arguments } else { @() }
        $code = Invoke-Native $exe $harnessArgs
        if ($null -ne $transportFixture) { Remove-Item Env:RNA_CAPTURE_FILE -ErrorAction SilentlyContinue }
        Remove-Item $exe -ErrorAction SilentlyContinue
        if ($code -ne 0) {
            $failed++
        } elseif ($null -ne $transportFixture) {
            $decoder = Join-Path $root 'tools\decode_rolling_no_assist.ps1'
            $decoded = Invoke-Native 'powershell.exe' @(
                '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $decoder,
                '-InputFile', $transportFixture, '-OutputFile', $transportCsv,
                '-RequireCompleteCapture')
            Remove-Item -LiteralPath $transportFixture,$transportCsv -Force -ErrorAction SilentlyContinue
            if ($decoded -ne 0) { $failed++ }
        }
    }
    if ($failed -ne 0) { throw "$failed host suite(s) FAILED" }
    Write-Host 'All host suites: PASS'
    exit 0
}

$cross = Find-CrossCompiler
if (-not $cross) {
    Write-Warning 'No C compiler found at all. Host tests were NOT run.'
    exit 2
}

Write-Warning 'No host C compiler on this machine - the host tests were NOT RUN.'
Write-Host "Falling back to a cross compile + link with $cross (syntax and linkage only)."
foreach ($s in $suites) {
    $elf = Join-Path $env:TEMP 'host_tests.elf'
    $built = Invoke-Native $cross (@(
        '-std=c11', '-Wall', '-Wextra', '-Werror', "-I$inc", '-mcpu=cortex-m4', '-mthumb',
        '--specs=nosys.specs', '--specs=nano.specs', '-o', $elf, $s.Harness) + $s.Modules) -Quiet
    if ($s.ContainsKey('ExpectBuildFailure') -and $s.ExpectBuildFailure) {
        if ($built -eq 0) {
            Remove-Item $elf -ErrorAction SilentlyContinue
            throw "$($s.Name): unexpectedly compiled (this probe must fail to compile)"
        }
        Write-Host "$($s.Name): correctly refused to build."
        continue
    }
    if ($built -ne 0) { throw "$($s.Name): harness does not compile" }
    Remove-Item $elf -ErrorAction SilentlyContinue
    Write-Host "$($s.Name): COMPILES AND LINKS (behaviour NOT verified - SKIPPED)."
}
exit 2
