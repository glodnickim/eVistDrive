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
$canDisplayCPathForward = (Join-Path $root 'src\CAN_Display.c') -replace '\\', '/'

# Every harness and the module(s) it links. Add new ones here.
$suites = @(
    @{ Name = 'FW-100 Extended Boost'
       Harness = Join-Path $PSScriptRoot 'fw100_extended_boost_host.c'
       Modules = @(Join-Path $root 'src\assist_extended_boost.c') },
    @{ Name = 'FW-113.1 Walk Assist RUN minimum Iq (real walk_assist_motor + walk_speed_controller)'
       Harness = Join-Path $PSScriptRoot 'walk_assist_run_min_host.c'
       Modules = @((Join-Path $root 'src\walk_assist_motor.c'),
                   (Join-Path $root 'src\walk_speed_controller.c')) },
    @{ Name = 'FW-113.2 Walk Assist no hold timeout + reason bits (real walk_assist_motor + walk_speed_controller)'
       Harness = Join-Path $PSScriptRoot 'walk_assist_diag_host.c'
       Modules = @((Join-Path $root 'src\walk_assist_motor.c'),
                   (Join-Path $root 'src\walk_speed_controller.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
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
    @{ Name = 'FW-111 delayed-rearm recorder'
       Harness = Join-Path $PSScriptRoot 'rearm_delay_diag_host.c'
       Modules = @(Join-Path $root 'src\rearm_delay_diag.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       # REARM_UID_SEAM_TEST: a test-only seam inside rearm_delay_diag.c (compiled out of the
       # firmware entirely - see the #ifdef there) that lets the harness drive the internal
       # record_uid generator against its uint32_t wrap, which a real ride could take decades to
       # reach. Same pattern as PAS_RAW_COPY_HOOK and CANMF_REFUSAL_HOOK below.
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1', '-DREARM_UID_SEAM_TEST') },
    @{ Name = 'FW-112-DIAG whole-chain event recorder (real fw112_diag.c)'
       Harness = Join-Path $PSScriptRoot 'fw112_diag_host.c'
       Modules = @(Join-Path $root 'src\fw112_diag.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       # The module's whole input surface is fw112_diag_input_t, so no other module is linked;
       # the harness drives every event/snapshot field directly (see the harness's file header).
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1') },
    @{ Name = 'FW-111 Bug 1 main.c standstill wiring (source-text guard)'
       Harness = Join-Path $PSScriptRoot 'main_rearm_wiring_host.c'
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @("-DMAIN_C_PATH=$mainCPathForward") },
    @{ Name = 'FW-111 v5/v5.1 full-chain integration (quadrature + direction + session + rearm + pas_trace + pas_raw)'
       Harness = Join-Path $PSScriptRoot 'rearm_trace_raw_integration_host.c'
       # v4: unlike v3's file of this name, this links the REAL direction/session automatons too -
       # not just the three recorders - so session-state transitions come from real reverse/
       # forward/invalid quadrature inputs, never from hand-pinning session_state (see the
       # harness's own file header for why the v3 file's proof was invalid).
       Modules = @((Join-Path $root 'src\pas_quadrature.c'),
                   (Join-Path $root 'src\pas_direction.c'),
                   (Join-Path $root 'src\ride_session.c'),
                   (Join-Path $root 'src\rearm_delay_diag.c'),
                   (Join-Path $root 'src\pas_trace.c'),
                   (Join-Path $root 'src\pas_raw.c'))
       IncludeDirs = @(Join-Path $PSScriptRoot 'common')
       Defines = @('-DCAN_DIAGNOSTICS_ENABLE=1') },
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
    @{ Name = 'FW-109 v2 pas_direction automaton (exhaustive + property proof)'
       Harness = Join-Path $PSScriptRoot 'pas_direction_host.c'
       Modules = @(Join-Path $root 'src\pas_direction.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-109 v2 ride_session automaton (exhaustive)'
       Harness = Join-Path $PSScriptRoot 'ride_session_host.c'
       Modules = @(Join-Path $root 'src\ride_session.c')
       IncludeDirs = @(Join-Path $PSScriptRoot 'common') },
    @{ Name = 'FW-109 v2 ride control / direction / session integration'
       Harness = Join-Path $PSScriptRoot 'ride_control_rearm_host.c'
       # Same module list and host_stubs directory as
       # tests/host/pipeline/ride_control_pipeline_host.c (see tests/host/run_regression.ps1's
       # Build-Harness -UseStubs): motor_core.h pulls in inc/main.h, which pulls in the real
       # GD32 CMSIS vendor headers - host_stubs/gd32f30x.h + arm_math.h stand in for those only.
       Modules = @((Join-Path $root 'src\torque_input.c'), (Join-Path $root 'src\rider_input.c'),
                   (Join-Path $root 'src\assist_modes.c'), (Join-Path $root 'src\cadence_comp.c'),
                   (Join-Path $root 'src\power_curve.c'), (Join-Path $root 'src\assist_start.c'),
                   (Join-Path $root 'src\assist_extended_boost.c'), (Join-Path $root 'src\tuning_config.c'),
                   (Join-Path $root 'src\ride_control.c'), (Join-Path $root 'src\ride_session.c'),
                   (Join-Path $root 'src\assist_dynamics.c'),
                   (Join-Path $root 'src\assist_limits.c'), (Join-Path $root 'src\motor_core.c'),
                   (Join-Path $root 'src\pas_quadrature.c'), (Join-Path $root 'src\pas_direction.c'),
                   (Join-Path $PSScriptRoot 'common\map_adapter.c'),
                   (Join-Path $PSScriptRoot 'common\motor_service_stub.c'))
       IncludeDirs = @((Join-Path $PSScriptRoot 'common\host_stubs'), (Join-Path $PSScriptRoot 'common'))
       # torque_input.c and assist_modes.c carry a pre-existing tautological-comparison warning
       # under this gcc - the same documented, targeted exception run_regression.ps1 already
       # applies (see its $typeLimitsException). -Werror stays on for every other module.
        Defines = @('-Wno-type-limits') },
    @{ Name = 'FW-112 RUN estimator rearm-recovery lifecycle (real estimator + rearm chain)'
       Harness = Join-Path $PSScriptRoot 'fw112_run_rearm_recovery_host.c'
       # Same real-module chain as the FW-109 v2 ride control suite: the RUN estimator
       # (torque_input.c) is driven by the real pas_direction/ride_session automatons, so the
       # harness exercises the actual FW-112 fast-track lifecycle around a real rearm.
       Modules = @((Join-Path $root 'src\torque_input.c'), (Join-Path $root 'src\rider_input.c'),
                   (Join-Path $root 'src\assist_modes.c'), (Join-Path $root 'src\cadence_comp.c'),
                   (Join-Path $root 'src\power_curve.c'), (Join-Path $root 'src\assist_start.c'),
                   (Join-Path $root 'src\assist_extended_boost.c'), (Join-Path $root 'src\tuning_config.c'),
                   (Join-Path $root 'src\ride_control.c'), (Join-Path $root 'src\ride_session.c'),
                   (Join-Path $root 'src\assist_dynamics.c'),
                   (Join-Path $root 'src\assist_limits.c'), (Join-Path $root 'src\motor_core.c'),
                   (Join-Path $root 'src\pas_quadrature.c'), (Join-Path $root 'src\pas_direction.c'),
                   (Join-Path $PSScriptRoot 'common\map_adapter.c'),
                   (Join-Path $PSScriptRoot 'common\motor_service_stub.c'))
IncludeDirs = @((Join-Path $PSScriptRoot 'common\host_stubs'), (Join-Path $PSScriptRoot 'common'))
        Defines = @('-Wno-type-limits') },
    @{ Name = 'FW-112.1 REAL_STOP liveness separation (real liveness + rearm chain)'
       Harness = Join-Path $PSScriptRoot 'fw112_1_realstop_host.c'
       # Same real-module chain as the FW-109 v2 ride control suite PLUS the FW-112.1 liveness
       # module: real_stop is COMPUTED from the real pas_liveness module fed by real transitions,
       # exactly as main.c wires it (see the harness file header for the S1-S11 scenario map).
       Modules = @((Join-Path $root 'src\torque_input.c'), (Join-Path $root 'src\rider_input.c'),
                   (Join-Path $root 'src\assist_modes.c'), (Join-Path $root 'src\cadence_comp.c'),
                   (Join-Path $root 'src\power_curve.c'), (Join-Path $root 'src\assist_start.c'),
                   (Join-Path $root 'src\assist_extended_boost.c'), (Join-Path $root 'src\tuning_config.c'),
                   (Join-Path $root 'src\ride_control.c'), (Join-Path $root 'src\ride_session.c'),
                   (Join-Path $root 'src\assist_dynamics.c'),
                   (Join-Path $root 'src\assist_limits.c'), (Join-Path $root 'src\motor_core.c'),
                   (Join-Path $root 'src\pas_quadrature.c'), (Join-Path $root 'src\pas_direction.c'),
                   (Join-Path $root 'src\pas_liveness.c'),
                   (Join-Path $PSScriptRoot 'common\map_adapter.c'),
                   (Join-Path $PSScriptRoot 'common\motor_service_stub.c'))
        IncludeDirs = @((Join-Path $PSScriptRoot 'common\host_stubs'), (Join-Path $PSScriptRoot 'common'))
        Defines = @('-Wno-type-limits') },
    @{ Name = 'FW-112.2 REAL_STOP vs ROLLING COAST (real liveness + wheel model + rearm chain)'
       Harness = Join-Path $PSScriptRoot 'fw112_2_rolling_coast_host.c'
       # Same real-module chain as the FW-112.1 suite PLUS the FW-112.2 wheel-freshness helper:
       # real_stop is COMPUTED from the real pas_liveness module and rider_input.wheel_valid from
       # the real ride_wheel_valid() (driven by the harness's wheel pulse model), so the qualified
       # terminal and the forward_pedaling coast rearm are the exact production decisions.
       Modules = @((Join-Path $root 'src\torque_input.c'), (Join-Path $root 'src\rider_input.c'),
                   (Join-Path $root 'src\assist_modes.c'), (Join-Path $root 'src\cadence_comp.c'),
                   (Join-Path $root 'src\power_curve.c'), (Join-Path $root 'src\assist_start.c'),
                   (Join-Path $root 'src\assist_extended_boost.c'), (Join-Path $root 'src\tuning_config.c'),
                   (Join-Path $root 'src\ride_control.c'), (Join-Path $root 'src\ride_session.c'),
                   (Join-Path $root 'src\assist_dynamics.c'),
                   (Join-Path $root 'src\assist_limits.c'), (Join-Path $root 'src\motor_core.c'),
                   (Join-Path $root 'src\pas_quadrature.c'), (Join-Path $root 'src\pas_direction.c'),
                   (Join-Path $root 'src\pas_liveness.c'), (Join-Path $root 'src\ride_wheel.c'),
                   (Join-Path $PSScriptRoot 'common\map_adapter.c'),
                   (Join-Path $PSScriptRoot 'common\motor_service_stub.c'))
       IncludeDirs = @((Join-Path $PSScriptRoot 'common\host_stubs'), (Join-Path $PSScriptRoot 'common'))
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
       Defines = @("-DMAIN_C_PATH=$mainCPathForward", "-DCAN_DISPLAY_C_PATH=$canDisplayCPathForward") }
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
        if ($built -ne 0) { Write-Host "$($s.Name): harness failed to BUILD"; $failed++; continue }
        $code = Invoke-Native $exe @()
        Remove-Item $exe -ErrorAction SilentlyContinue
        if ($code -ne 0) { $failed++ }
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
    if ($built -ne 0) { throw "$($s.Name): harness does not compile" }
    Remove-Item $elf -ErrorAction SilentlyContinue
    Write-Host "$($s.Name): COMPILES AND LINKS (behaviour NOT verified - SKIPPED)."
}
exit 2
