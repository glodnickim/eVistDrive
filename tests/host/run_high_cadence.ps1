# TEST-002: HIGH-CADENCE BENCHMARK QUALITY - single entry point.
#
#   powershell -File tests/host/run_high_cadence.ps1
#
# Builds torque_revolution_bench_host (TEST A) and power_revolution_bench_host (TEST B)
# against the SHIPPED src/*.c modules, runs the sweep matrix documented in
# documentation/TEST_002_HIGH_CADENCE_BENCHMARK_REPORT_PL.md section 13, writes CSVs to
# tests/host/out/high_cadence/, and runs the post-processing (normalized output vs
# cadence, pumping/symmetry summaries) via tools/HighCadenceTools.ps1.
#
# Does not touch or replace tests/host/run-host-tests.ps1 (FW-100/101/102) or
# tests/host/run_regression.ps1 (TEST-001) - both remain separate and unchanged.

$ErrorActionPreference = 'Stop'
[System.Threading.Thread]::CurrentThread.CurrentCulture = [System.Globalization.CultureInfo]::InvariantCulture

function Invoke-Native {
    param([string]$Exe, [string[]]$Arguments)
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $output = & $Exe @Arguments 2>&1
        $output | ForEach-Object { Write-Host $_ }
        return $LASTEXITCODE
    } finally { $ErrorActionPreference = $previous }
}

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$inc = Join-Path $root 'inc'
$src = Join-Path $root 'src'
$testHost = $PSScriptRoot
$common = Join-Path $testHost 'common'
$stubs = Join-Path $common 'host_stubs'
$outDir = Join-Path $testHost 'out\high_cadence'
$objDir = Join-Path $env:TEMP 'evistdrive_high_cadence_obj'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
New-Item -ItemType Directory -Force -Path $objDir | Out-Null

. (Join-Path $testHost 'tools\HighCadenceTools.ps1')

function Find-HostCompiler {
    $portable = 'C:\Projekty\tools\w64devkit\bin\gcc.exe'
    if (Test-Path $portable) { return @{ Path = $portable; Bin = (Split-Path -Parent $portable) } }
    $found = Get-Command 'gcc.exe' -ErrorAction SilentlyContinue
    if ($found) { return @{ Path = $found.Source; Bin = (Split-Path -Parent $found.Source) } }
    return $null
}
$hostCc = Find-HostCompiler
if (-not $hostCc) {
    Write-Warning 'No host C compiler found. TEST-002 was NOT run.'
    exit 2
}
Write-Host "Host compiler: $($hostCc.Path)"
$env:PATH = "$($hostCc.Bin);$env:PATH"

# Same documented exception as TEST-001 (run_regression.ps1) - a pre-existing
# tautological-comparison warning in these two files under -Wall -Wextra -Werror on
# gcc 14.2.0, unrelated to this card. See documentation/TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md
# findings F-tl-1/F-tl-2.
$typeLimitsException = @('torque_input.c', 'assist_modes.c')

function Build-Object {
    param([string]$SourcePath, [string]$ObjPath, [string[]]$IncludeDirs, [switch]$AllowTypeLimits)
    if (Test-Path $ObjPath) { return } # object cache: many harnesses share the same production modules
    $ArgList = @('-std=c11', '-Wall', '-Wextra', '-Werror')
    if ($AllowTypeLimits) { $ArgList += '-Wno-type-limits' }
    foreach ($i in $IncludeDirs) { $ArgList += "-I$i" }
    $ArgList += @('-c', $SourcePath, '-o', $ObjPath)
    $code = Invoke-Native $hostCc.Path $ArgList
    if ($code -ne 0) { throw "compile failed: $SourcePath" }
}

function Obj($name) { return (Join-Path $objDir "$name.o") }

Write-Host '--- compiling shared objects ---'
Build-Object -SourcePath (Join-Path $common 'crank_model.c') -ObjPath (Obj 'crank_model') -IncludeDirs @($common)
Build-Object -SourcePath (Join-Path $common 'signal_stats.c') -ObjPath (Obj 'signal_stats') -IncludeDirs @($common)
Build-Object -SourcePath (Join-Path $common 'map_adapter.c') -ObjPath (Obj 'map_adapter') -IncludeDirs @($common)
Build-Object -SourcePath (Join-Path $common 'motor_service_stub.c') -ObjPath (Obj 'motor_service_stub') -IncludeDirs @($inc)
Build-Object -SourcePath (Join-Path $src 'torque_input.c') -ObjPath (Obj 'torque_input') -IncludeDirs @($inc) -AllowTypeLimits
Build-Object -SourcePath (Join-Path $src 'rider_input.c') -ObjPath (Obj 'rider_input') -IncludeDirs @($inc)
Build-Object -SourcePath (Join-Path $src 'assist_modes.c') -ObjPath (Obj 'assist_modes') -IncludeDirs @($inc) -AllowTypeLimits
Build-Object -SourcePath (Join-Path $src 'cadence_comp.c') -ObjPath (Obj 'cadence_comp') -IncludeDirs @($inc)
Build-Object -SourcePath (Join-Path $src 'power_curve.c') -ObjPath (Obj 'power_curve') -IncludeDirs @($inc)
Build-Object -SourcePath (Join-Path $src 'assist_start.c') -ObjPath (Obj 'assist_start') -IncludeDirs @($inc)
Build-Object -SourcePath (Join-Path $src 'assist_extended_boost.c') -ObjPath (Obj 'assist_extended_boost') -IncludeDirs @($inc)
Build-Object -SourcePath (Join-Path $src 'tuning_config.c') -ObjPath (Obj 'tuning_config') -IncludeDirs @($inc)
Build-Object -SourcePath (Join-Path $src 'ride_control.c') -ObjPath (Obj 'ride_control') -IncludeDirs @($stubs, $inc)
Build-Object -SourcePath (Join-Path $src 'motor_core.c') -ObjPath (Obj 'motor_core') -IncludeDirs @($stubs, $inc)
Build-Object -SourcePath (Join-Path $src 'assist_dynamics.c') -ObjPath (Obj 'assist_dynamics') -IncludeDirs @($inc)
Build-Object -SourcePath (Join-Path $src 'assist_limits.c') -ObjPath (Obj 'assist_limits') -IncludeDirs @($inc)

Write-Host '--- building TEST A (torque_revolution_bench_host) ---'
$testAObj = Obj 'test_a_harness'
Build-Object -SourcePath (Join-Path $testHost 'torque\torque_revolution_bench_host.c') -ObjPath $testAObj -IncludeDirs @($common, $inc)
$testAExe = Join-Path $objDir 'test_a.exe'
$code = Invoke-Native $hostCc.Path (@('-o', $testAExe, $testAObj, (Obj 'crank_model'), (Obj 'signal_stats'), (Obj 'torque_input')))
if ($code -ne 0) { throw 'TEST A link failed' }

Write-Host '--- building TEST B (power_revolution_bench_host) ---'
$testBObj = Obj 'test_b_harness'
Build-Object -SourcePath (Join-Path $testHost 'pipeline\power_revolution_bench_host.c') -ObjPath $testBObj -IncludeDirs @($stubs, $common, $inc)
$testBExe = Join-Path $objDir 'test_b.exe'
$testBModules = @('crank_model', 'signal_stats', 'torque_input', 'rider_input', 'assist_modes',
    'cadence_comp', 'power_curve', 'assist_start', 'assist_extended_boost', 'tuning_config',
    'ride_control', 'motor_core', 'assist_dynamics', 'assist_limits', 'map_adapter', 'motor_service_stub')
$code = Invoke-Native $hostCc.Path (@('-o', $testBExe, $testBObj) + ($testBModules | ForEach-Object { Obj $_ }))
if ($code -ne 0) { throw 'TEST B link failed' }

Write-Host "`nHarnesses built. Running TEST A sweeps..."

# Clear shared summary files so re-runs don't append onto stale data from a previous session.
Remove-Item (Join-Path $outDir 'torque_summary.csv') -ErrorAction SilentlyContinue
Remove-Item (Join-Path $outDir 'power_summary.csv') -ErrorAction SilentlyContinue

$cadenceFull = @(60, 80, 90, 100, 110, 120)
$cadenceReduced = @(60, 80, 100, 120)
$cadenceHighFocus = @(80, 100, 110, 120)

function Run-TestA {
    param([double]$Cadence, [string]$Profile, [string]$Mode, [string]$Tag, [switch]$FullTrace)
    $cliArgs = @($Cadence, $Profile, $Mode, $outDir, $Tag)
    if ($FullTrace) { $cliArgs += '--full-trace' }
    $code = Invoke-Native $testAExe $cliArgs
    if ($code -ne 0) { throw "TEST A run failed: $Tag" }
}
function Run-TestB {
    param([double]$Cadence, [string]$Profile, [int]$VoltageMv, [int]$Mvu, [string]$Tag, [switch]$PerRev)
    $cliArgs = @($Cadence, $Profile, $VoltageMv, $Mvu, $outDir, $Tag)
    if ($PerRev) { $cliArgs += '--per-rev' }
    $code = Invoke-Native $testBExe $cliArgs
    if ($code -ne 0) { throw "TEST B run failed: $Tag" }
}

# --- A1: baseline cadence sweep, REV20 (this card's methodology) -------------------
foreach ($c in $cadenceFull) {
    $full = ($c -eq 80 -or $c -eq 120)
    Run-TestA -Cadence $c -Profile 'BASELINE' -Mode 'REV20' -Tag "A1_rev20_c$c" -FullTrace:$full
}
# --- A2: baseline cadence sweep, TIME6S (TEST-001's methodology, for direct comparison) --
foreach ($c in $cadenceFull) {
    Run-TestA -Cadence $c -Profile 'BASELINE' -Mode 'TIME6S' -Tag "A2_time6s_c$c"
}
# --- A3: symmetry sweep (BASELINE already covers the asymmetric case) --------------
foreach ($c in $cadenceReduced) {
    foreach ($p in @('SYMMETRIC', 'DEADSPOT')) {
        $full = ($c -eq 80)
        Run-TestA -Cadence $c -Profile $p -Mode 'REV20' -Tag "A3_${p}_c$c" -FullTrace:$full
    }
}
# --- A4: load sweep (BASELINE = medium already covered) ----------------------------
foreach ($c in $cadenceReduced) {
    foreach ($p in @('LOAD_LOW', 'LOAD_HIGH')) {
        Run-TestA -Cadence $c -Profile $p -Mode 'REV20' -Tag "A4_${p}_c$c"
    }
}
# --- Warm-up scan (card section 5) --------------------------------------------------
Run-TestA -Cadence 60 -Profile 'BASELINE' -Mode 'WARMUP_SCAN' -Tag 'warmup_scan_c60'
Run-TestA -Cadence 120 -Profile 'BASELINE' -Mode 'WARMUP_SCAN' -Tag 'warmup_scan_c120'
Run-TestA -Cadence 120 -Profile 'LOAD_HIGH' -Mode 'WARMUP_SCAN' -Tag 'warmup_scan_loadhigh_c120'

Write-Host "`nRunning TEST B sweeps..."

# --- B1: motor_voltage_utilization sweep at every cadence, fixed voltage/load ------
foreach ($c in $cadenceFull) {
    foreach ($m in @(0, 800, 1200, 1400, 1600, 1800, 1900)) {
        $perRev = ($m -eq 0 -or $m -eq 1900)
        Run-TestB -Cadence $c -Profile 'BASELINE' -VoltageMv 42000 -Mvu $m -Tag "B1_c${c}_mvu$m" -PerRev:$perRev
    }
}
# --- B2: load sweep, high-cadence focused, two representative mvu points ----------
foreach ($c in $cadenceReduced) {
    foreach ($p in @('LOAD_LOW', 'LOAD_HIGH')) {
        foreach ($m in @(0, 1600)) {
            Run-TestB -Cadence $c -Profile $p -VoltageMv 42000 -Mvu $m -Tag "B2_${p}_c${c}_mvu$m"
        }
    }
}
# --- B3: battery voltage sweep, high-cadence focused, two representative mvu points --
foreach ($c in $cadenceHighFocus) {
    foreach ($v in @(42000, 40000, 38000, 36000)) {
        foreach ($m in @(0, 1600)) {
            Run-TestB -Cadence $c -Profile 'BASELINE' -VoltageMv $v -Mvu $m -Tag "B3_c${c}_v${v}_mvu$m"
        }
    }
}
# --- B4: clamp-boundary probe (see report section 10/13 for the math this checks) --
# The power/voltage-utilization cross-check in assist_modes.c's finish_power_request()
# gets MORE restrictive as battery voltage RISES (less current needed for the same
# power), the opposite direction from B3's sweep - probe upward too, at the highest
# tested load, to see whether/where it ever becomes the binding constraint.
foreach ($v in @(42000, 48000, 54000, 59000)) {
    foreach ($m in @(0, 2048)) {
        Run-TestB -Cadence 120 -Profile 'LOAD_HIGH' -VoltageMv $v -Mvu $m -Tag "B4_clampprobe_v${v}_mvu$m"
    }
}

Write-Host "`nAll sweeps complete. Post-processing..."
Build-NormalizedOutputReport -OutDir $outDir
Build-PumpingSymmetryReport -OutDir $outDir
Build-ClampProbeReport -OutDir $outDir

Write-Host "`nDone. Outputs in $outDir"
exit 0
