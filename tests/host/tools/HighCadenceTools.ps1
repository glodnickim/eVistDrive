# TEST-002 post-processing helpers. Reads the CSVs run_high_cadence.ps1's harnesses
# wrote, never re-derives firmware behaviour itself - every number here is copied or
# arithmetically combined (ratio, difference) from values the C harnesses already
# computed against the real modules.

function ConvertTo-InvariantDouble($text) {
    return [double]::Parse($text, [System.Globalization.CultureInfo]::InvariantCulture)
}

<#
.SYNOPSIS
Card section 13: normalized_output(cadence) = mean_output_at_cadence / mean_output_at_80rpm,
for torque_RUN (TEST A, REV20, BASELINE) and motor_power/iq_request/iq_final (TEST B,
BASELINE, per motor_voltage_utilization). 80 rpm is this card's DECLARED reference (not
claimed to be universally "ideal" - card section 13).
#>
function Build-NormalizedOutputReport {
    param([string]$OutDir)

    $rows = New-Object System.Collections.Generic.List[object]

    $torqueSummary = Import-Csv (Join-Path $OutDir 'torque_summary.csv') |
        Where-Object { $_.mode -eq 'REV20' -and $_.profile -eq 'BASELINE' -and $_.run_tag -like 'A1_*' }
    $ref = $torqueSummary | Where-Object { $_.cadence_rpm -eq '80.00' } | Select-Object -First 1
    if ($ref) {
        $refVal = ConvertTo-InvariantDouble $ref.per_rev_mean_run_avg
        foreach ($r in $torqueSummary) {
            $val = ConvertTo-InvariantDouble $r.per_rev_mean_run_avg
            $rows.Add([pscustomobject]@{
                metric = 'torque_run_mean'; series = 'TEST_A_BASELINE'; cadence_rpm = $r.cadence_rpm
                absolute_value = $val; normalized_vs_80rpm = ($val / $refVal)
            })
        }
    }

    $powerSummary = Import-Csv (Join-Path $OutDir 'power_summary.csv') |
        Where-Object { $_.profile -eq 'BASELINE' -and $_.battery_voltage_mv -eq '42000' -and $_.run_tag -like 'B1_*' }
    foreach ($mvu in ($powerSummary | Select-Object -ExpandProperty motor_voltage_utilization -Unique)) {
        $series = $powerSummary | Where-Object { $_.motor_voltage_utilization -eq $mvu }
        $refRow = $series | Where-Object { $_.cadence_rpm -eq '80.00' } | Select-Object -First 1
        if (-not $refRow) { continue }
        foreach ($metricName in @('motor_power_w_mean', 'iq_request_mean', 'iq_final_mean')) {
            $refVal = ConvertTo-InvariantDouble $refRow.$metricName
            foreach ($r in $series) {
                $val = ConvertTo-InvariantDouble $r.$metricName
                $norm = if ([Math]::Abs($refVal) -gt 0.001) { $val / $refVal } else { $null }
                $rows.Add([pscustomobject]@{
                    metric = $metricName; series = "TEST_B_mvu$mvu"; cadence_rpm = $r.cadence_rpm
                    absolute_value = $val; normalized_vs_80rpm = $norm
                })
            }
        }
    }

    $rows | Export-Csv -Path (Join-Path $OutDir 'normalized_output_vs_cadence.csv') -NoTypeInformation -Encoding UTF8
    Write-Host "normalized_output_vs_cadence.csv: $($rows.Count) rows"
}

<#
.SYNOPSIS
Card section 14 (pumping) + section 15 (left/right asymmetry). Pumping: per-scenario
ripple_run summary straight from each TEST A per-revolution CSV (already computed by the
C harness - this just collects one row per scenario, card section 14: "preferuj proste
metryki"). Symmetry: mean(FAST/RUN) over phase bins 0-47 (0-180 deg, "left") vs 48-95
(180-360 deg, "right") from each TEST A phase-binned CSV.
#>
function Build-PumpingSymmetryReport {
    param([string]$OutDir)

    $pumpingRows = New-Object System.Collections.Generic.List[object]
    $symmetryRows = New-Object System.Collections.Generic.List[object]

    Get-ChildItem (Join-Path $OutDir 'torque_per_revolution_*.csv') | ForEach-Object {
        $tag = $_.BaseName -replace '^torque_per_revolution_', ''
        $rows = Import-Csv $_.FullName
        if ($rows.Count -eq 0) { return }
        $rippleVals = $rows | ForEach-Object { ConvertTo-InvariantDouble $_.ripple_run }
        $ptpVals = $rows | ForEach-Object { ConvertTo-InvariantDouble $_.ptp_run }
        $pumpingRows.Add([pscustomobject]@{
            run_tag = $tag; n_revolutions = $rows.Count
            ripple_run_mean = ($rippleVals | Measure-Object -Average).Average
            ripple_run_min = ($rippleVals | Measure-Object -Minimum).Minimum
            ripple_run_max = ($rippleVals | Measure-Object -Maximum).Maximum
            ptp_run_mean = ($ptpVals | Measure-Object -Average).Average
        })
    }
    $pumpingRows | Export-Csv -Path (Join-Path $OutDir 'pumping_metrics_summary.csv') -NoTypeInformation -Encoding UTF8
    Write-Host "pumping_metrics_summary.csv: $($pumpingRows.Count) rows"

    Get-ChildItem (Join-Path $OutDir 'torque_phase_binned_*.csv') | ForEach-Object {
        $tag = $_.BaseName -replace '^torque_phase_binned_', ''
        $rows = Import-Csv $_.FullName
        if ($rows.Count -lt 96) { return }
        $left = $rows[0..47]
        $right = $rows[48..95]
        $leftFast = ($left | ForEach-Object { ConvertTo-InvariantDouble $_.mean_fast } | Measure-Object -Average).Average
        $rightFast = ($right | ForEach-Object { ConvertTo-InvariantDouble $_.mean_fast } | Measure-Object -Average).Average
        $leftRun = ($left | ForEach-Object { ConvertTo-InvariantDouble $_.mean_run } | Measure-Object -Average).Average
        $rightRun = ($right | ForEach-Object { ConvertTo-InvariantDouble $_.mean_run } | Measure-Object -Average).Average
        $symmetryRows.Add([pscustomobject]@{
            run_tag = $tag
            left_fast_mean = $leftFast; right_fast_mean = $rightFast
            fast_left_right_diff_pct = (($rightFast - $leftFast) / (($leftFast + $rightFast) / 2)) * 100
            left_run_mean = $leftRun; right_run_mean = $rightRun
            run_left_right_diff_pct = (($rightRun - $leftRun) / (($leftRun + $rightRun) / 2)) * 100
        })
    }
    $symmetryRows | Export-Csv -Path (Join-Path $OutDir 'symmetry_analysis.csv') -NoTypeInformation -Encoding UTF8
    Write-Host "symmetry_analysis.csv: $($symmetryRows.Count) rows"
}

<#
.SYNOPSIS
Card section 7 "OBSERVABILITY GAP" workaround: no exported flag says whether the
power/voltage-utilization cross-check inside assist_modes.c clamped this tick, so this
compares PAIRED runs (same cadence/profile/voltage, mvu=0 vs mvu=X) from power_summary.csv
and reports the difference. A non-zero delta is direct evidence the clamp bound; a zero
delta means it did not, for that combination - honestly inferred from two independent
process runs, not a claim read from a firmware-exported state (see
documentation/testing/TEST_INTERFACES.md and the final report's OBSERVABILITY GAP list).
#>
function Build-ClampProbeReport {
    param([string]$OutDir)

    $summary = Import-Csv (Join-Path $OutDir 'power_summary.csv')
    $rows = New-Object System.Collections.Generic.List[object]

    $groups = $summary | Group-Object { "$($_.cadence_rpm)|$($_.profile)|$($_.battery_voltage_mv)" }
    foreach ($g in $groups) {
        $atZero = $g.Group | Where-Object { $_.motor_voltage_utilization -eq '0' } | Select-Object -First 1
        $others = $g.Group | Where-Object { $_.motor_voltage_utilization -ne '0' }
        if (-not $atZero) { continue }
        $refIq = ConvertTo-InvariantDouble $atZero.iq_request_mean
        foreach ($o in $others) {
            $iq = ConvertTo-InvariantDouble $o.iq_request_mean
            $rows.Add([pscustomobject]@{
                cadence_rpm = $atZero.cadence_rpm; profile = $atZero.profile
                battery_voltage_mv = $atZero.battery_voltage_mv
                motor_voltage_utilization = $o.motor_voltage_utilization
                iq_request_at_mvu0 = $refIq; iq_request_at_this_mvu = $iq
                delta = ($iq - $refIq); clamp_engaged = [bool]([Math]::Abs($iq - $refIq) -gt 0.01)
            })
        }
    }
    $rows | Export-Csv -Path (Join-Path $OutDir 'clamp_probe_summary.csv') -NoTypeInformation -Encoding UTF8
    $engaged = ($rows | Where-Object { $_.clamp_engaged }).Count
    Write-Host "clamp_probe_summary.csv: $($rows.Count) paired comparisons, $engaged show the clamp engaged"
}
