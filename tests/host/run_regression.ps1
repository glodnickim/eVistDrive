# EVistDrive regression test infrastructure - single entry point.
#
#   powershell -File tests/host/run_regression.ps1
#
# Builds every regression harness against the SHIPPED src/*.c modules, runs the P0
# scenarios (RUN_60/80/100/110/120, CADENCE_RAMP_50_120, MISSED_TICK_BURST), writes one
# CSV trace per run to tests/host/out/, computes metrics, runs a determinism smoke-test
# through the first-divergence comparator, and writes
# tests/host/out/REGRESSION_RESULTS.md.
#
# This does NOT run the existing FW-100/101/102 pass/fail suites - that is still
# tests/host/run-host-tests.ps1, unchanged, and this script does not replace it.
#
# No ARM toolchain is required or used (card section 18). See
# documentation/TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md for the full report this
# script's own output feeds.

$ErrorActionPreference = 'Stop'

# All harness trace CSVs are written by C (fprintf, "C" locale -> always a period decimal
# point). Force this script's own number formatting (Export-Csv, string interpolation) to
# match, regardless of the machine's regional settings - otherwise metrics_summary.csv
# would mix a comma-decimal culture with period-decimal source data, which is exactly the
# kind of silent, locale-dependent bug this test infrastructure exists to not have.
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
$outDir = Join-Path $testHost 'out'
$objDir = Join-Path $env:TEMP 'evistdrive_regression_obj'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
New-Item -ItemType Directory -Force -Path $objDir | Out-Null

. (Join-Path $testHost 'tools\RegressionTools.ps1')

function Find-HostCompiler {
    $portable = 'C:\Projekty\tools\w64devkit\bin\gcc.exe'
    if (Test-Path $portable) { return @{ Path = $portable; Bin = (Split-Path -Parent $portable) } }
    $found = Get-Command 'gcc.exe' -ErrorAction SilentlyContinue
    if ($found) { return @{ Path = $found.Source; Bin = (Split-Path -Parent $found.Source) } }
    return $null
}

$hostCc = Find-HostCompiler
if (-not $hostCc) {
    Write-Warning 'No host C compiler found (looked for w64devkit and gcc on PATH). Regression tests were NOT run.'
    exit 2
}
Write-Host "Host compiler: $($hostCc.Path)"
$env:PATH = "$($hostCc.Bin);$env:PATH"

# Production files with a PRE-EXISTING tautological-comparison warning under -Wall
# -Wextra -Werror on this gcc (14.2.0) - see documentation/TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md
# findings F-tl-1/F-tl-2. Not touched (out of scope for this card); this list is the one
# targeted, documented exception to an otherwise strict build.
$typeLimitsException = @('torque_input.c', 'assist_modes.c')

function Build-Object {
    param([string]$SourcePath, [string]$ObjPath, [string[]]$IncludeDirs, [switch]$AllowTypeLimits)
    $args = @('-std=c11', '-Wall', '-Wextra', '-Werror')
    if ($AllowTypeLimits) { $args += '-Wno-type-limits' }
    foreach ($i in $IncludeDirs) { $args += "-I$i" }
    $args += @('-c', $SourcePath, '-o', $ObjPath)
    $code = Invoke-Native $hostCc.Path $args
    if ($code -ne 0) { throw "compile failed: $SourcePath" }
}

function Build-Harness {
    param([string]$Name, [string]$HarnessRel, [string[]]$CommonFiles, [string[]]$Modules, [switch]$UseStubs)
    Write-Host "--- building $Name ---"
    $includeDirs = @($common)
    if ($UseStubs) { $includeDirs = @($stubs) + $includeDirs }
    $includeDirs += $inc

    $objs = New-Object System.Collections.Generic.List[string]

    $harnessObj = Join-Path $objDir "$Name.harness.o"
    Build-Object -SourcePath (Join-Path $testHost $HarnessRel) -ObjPath $harnessObj -IncludeDirs $includeDirs
    $objs.Add($harnessObj)

    foreach ($cf in $CommonFiles) {
        $cfObj = Join-Path $objDir "$Name.common.$cf.o"
        Build-Object -SourcePath (Join-Path $common $cf) -ObjPath $cfObj -IncludeDirs $includeDirs
        $objs.Add($cfObj)
    }

    foreach ($m in $Modules) {
        $mObj = Join-Path $objDir "$Name.src.$m.o"
        $allow = $typeLimitsException -contains $m
        Build-Object -SourcePath (Join-Path $src $m) -ObjPath $mObj -IncludeDirs $includeDirs -AllowTypeLimits:$allow
        $objs.Add($mObj)
    }

    $exe = Join-Path $objDir "$Name.exe"
    $code = Invoke-Native $hostCc.Path (@('-o', $exe) + $objs)
    if ($code -ne 0) { throw "link failed: $Name" }
    return $exe
}

$scenarios = @('RUN_60', 'RUN_80', 'RUN_100', 'RUN_110', 'RUN_120', 'CADENCE_RAMP_50_120')

$powerPipelineModules = @('torque_input.c', 'rider_input.c', 'assist_modes.c', 'cadence_comp.c',
    'power_curve.c', 'assist_start.c', 'assist_extended_boost.c', 'tuning_config.c')
$rideControlModules = $powerPipelineModules + @('ride_control.c', 'ride_session.c', 'assist_dynamics.c', 'assist_limits.c', 'motor_core.c')

$torqueExe = Build-Harness -Name 'torque_trace' -HarnessRel 'torque\torque_trace_host.c' `
    -CommonFiles @('crank_model.c') -Modules @('torque_input.c')
$powerExe = Build-Harness -Name 'power_pipeline' -HarnessRel 'pipeline\power_pipeline_host.c' `
    -CommonFiles @('crank_model.c') -Modules $powerPipelineModules
$rideExe = Build-Harness -Name 'ride_control_pipeline' -HarnessRel 'pipeline\ride_control_pipeline_host.c' `
    -CommonFiles @('crank_model.c', 'map_adapter.c', 'motor_service_stub.c') -Modules $rideControlModules -UseStubs
$burstExe = Build-Harness -Name 'missed_tick_burst' -HarnessRel 'scenarios\missed_tick_burst_host.c' `
    -CommonFiles @('crank_model.c') -Modules @('torque_input.c', 'ride_episode.c')

Write-Host "`nAll harnesses built. Running scenarios..."

$traceFiles = @{}
foreach ($sc in $scenarios) {
    foreach ($pair in @(@{ Exe = $torqueExe; Tag = 'torque' }, @{ Exe = $powerExe; Tag = 'power' }, @{ Exe = $rideExe; Tag = 'ride' })) {
        $csv = Join-Path $outDir "$($sc)_$($pair.Tag).csv"
        $code = Invoke-Native $pair.Exe @($sc, $csv)
        if ($code -ne 0) { throw "$($pair.Tag) harness failed on $sc" }
        $traceFiles["$sc.$($pair.Tag)"] = $csv
    }
}

$burstCsv = Join-Path $outDir 'missed_tick_burst_summary.csv'
$code = Invoke-Native $burstExe @($burstCsv)
if ($code -ne 0) { throw 'missed_tick_burst harness failed' }

Write-Host "`nComputing metrics..."

$powerColumns = @('torque_raw', 'torque_corrected', 'torque_fast', 'torque_run',
    'human_power_w', 'motor_power_raw_w', 'motor_power_w', 'iq_request')
$rideColumns = @('torque_fast', 'torque_run', 'iq_request', 'iq_final')

$metricsReport = New-Object System.Collections.Generic.List[object]
foreach ($sc in $scenarios) {
    $m = Get-TraceMetrics -CsvPath $traceFiles["$sc.power"] -Columns $powerColumns
    foreach ($row in $m) {
        $metricsReport.Add([pscustomobject]@{ Scenario = $sc; Layer = 'power_pipeline'; Column = $row.Column
            Mean = $row.Mean; Min = $row.Min; Max = $row.Max; P5 = $row.P5; P95 = $row.P95
            PeakToPeak = $row.PeakToPeak; StdDev = $row.StdDev; Ripple = $row.Ripple })
    }
    $rm = Get-TraceMetrics -CsvPath $traceFiles["$sc.ride"] -Columns $rideColumns
    foreach ($row in $rm) {
        $metricsReport.Add([pscustomobject]@{ Scenario = $sc; Layer = 'ride_control_pipeline'; Column = $row.Column
            Mean = $row.Mean; Min = $row.Min; Max = $row.Max; P5 = $row.P5; P95 = $row.P95
            PeakToPeak = $row.PeakToPeak; StdDev = $row.StdDev; Ripple = $row.Ripple })
    }
}
$metricsCsvPath = Join-Path $outDir 'metrics_summary.csv'
$metricsReport | Export-Csv -Path $metricsCsvPath -NoTypeInformation -Encoding UTF8

Write-Host "`nDeterminism smoke-test (first-divergence comparator self-check)..."
# The comparator is only trustworthy if it reports SAME between two runs of the
# IDENTICAL scenario against the IDENTICAL binary - this is what "deterministic
# regression measurement" in the card's success criterion (section 22) actually rests
# on. Re-run RUN_100 into a second file and compare it against the first.
$repeatCsv = Join-Path $outDir 'RUN_100_power_repeat.csv'
Invoke-Native $powerExe @('RUN_100', $repeatCsv) | Out-Null
$layers = @(
    @{ Column = 'torque_raw'; Tolerance = 0.5 }
    @{ Column = 'torque_corrected'; Tolerance = 0.5 }
    @{ Column = 'torque_fast'; Tolerance = 0.5 }
    @{ Column = 'torque_run'; Tolerance = 0.5 }
    @{ Column = 'human_power_w'; Tolerance = 0.5 }
    @{ Column = 'motor_power_w'; Tolerance = 0.5 }
    @{ Column = 'iq_request'; Tolerance = 0.5 }
)
$determinism = Compare-Traces -BaselineCsv $traceFiles['RUN_100.power'] -CurrentCsv $repeatCsv -Layers $layers
$determinismOk = ($null -eq $determinism.FirstDivergence)

Write-Host "`nCandidate golden baselines (NOT auto-approved - see documentation/testing/TEST_ARCHITECTURE.md)..."
$goldenDir = Join-Path $testHost 'golden\candidates'
New-Item -ItemType Directory -Force -Path $goldenDir | Out-Null
Copy-Item -Path $metricsCsvPath -Destination (Join-Path $goldenDir 'metrics_summary.csv') -Force

# ---- Markdown result report -------------------------------------------------------
$md = New-Object System.Collections.Generic.List[string]
$md.Add('# Regression run results')
$md.Add('')
$md.Add("Generated: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
$md.Add('')
$md.Add('## Build')
$md.Add('')
$md.Add('All harnesses built and linked with -Wall -Wextra -Werror (host gcc), with the')
$md.Add('documented -Wno-type-limits exception for torque_input.c and assist_modes.c only.')
$md.Add('')
$md.Add('## Determinism smoke-test (RUN_100, power_pipeline, two independent runs)')
$md.Add('')
$md.Add("Result: **$(if ($determinismOk) { 'PASS - identical' } else { "FAIL - first divergence at $($determinism.FirstDivergence)" })**")
$md.Add('')
$md.Add('## Metrics summary')
$md.Add('')
$md.Add('See metrics_summary.csv in this directory for the full table (all scenarios x')
$md.Add('layers x columns). No PASS/FAIL thresholds are declared yet (card section 12) -')
$md.Add('these are baseline measurements of the CURRENT firmware behaviour.')
$md.Add('')
$md.Add('## MISSED_TICK_BURST')
$md.Add('')
$md.Add('See missed_tick_burst_summary.csv and the console output above for the three')
$md.Add('category demonstrations (A elapsed-time / B control-update / C lost sample).')
$md.Add('')
($md -join "`n") | Out-File -FilePath (Join-Path $outDir 'REGRESSION_RESULTS.md') -Encoding UTF8

Write-Host "`nDone. Results: $outDir\REGRESSION_RESULTS.md"
Write-Host "Determinism smoke-test: $(if ($determinismOk) { 'PASS' } else { 'FAIL' })"
if (-not $determinismOk) { exit 1 }
exit 0
