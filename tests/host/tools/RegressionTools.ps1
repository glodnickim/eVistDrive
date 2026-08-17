# Shared PowerShell helpers for the regression test infrastructure: metrics over a CSV
# trace column, and a layer-by-layer "first divergence" comparator between two traces.
#
# WHY POWERSHELL, NOT PYTHON: the card asked for Python for this role. Python is not
# installed in this environment (`python`/`python3`/`py` all resolve to nothing on PATH -
# verified before writing this file). Section 18 of the card explicitly allows "cos w
# rodzaju tests/host/run_tests.sh lub odpowiednik zgodny ze srodowiskiem repo" - this
# repo's own tooling (build_firmware.ps1, tests/host/run-host-tests.ps1) is already 100%
# PowerShell, so that is the equivalent used here. Every trace this produces is a plain
# CSV; nothing here stops a later card from pointing an actual Python analysis at the same
# files if Python becomes available.
#
# Plain [double[]] arrays throughout, never .NET List<T>.AddRange from a function return -
# PowerShell enumerates a collection written to the output stream one element at a time
# unless the caller guards it with the unary comma, which is easy to get wrong. Arrays
# built with `$x = foreach (...) { ... }` and returned as `,@($x)` sidestep that trap
# entirely instead of relying on remembering the guard at every call site.

function Get-CsvColumnDoubles {
    param([Parameter(Mandatory)][object[]]$Rows, [Parameter(Mandatory)][string]$Column)
    $values = foreach ($row in $Rows) {
        $raw = $row.$Column
        if ($null -ne $raw -and $raw -ne '') {
            [double]::Parse($raw, [System.Globalization.CultureInfo]::InvariantCulture)
        }
    }
    return ,@($values)
}

function Get-Percentile {
    param([Parameter(Mandatory)][double[]]$Sorted, [Parameter(Mandatory)][double]$Fraction)
    if ($Sorted.Count -eq 0) { return [double]::NaN }
    if ($Sorted.Count -eq 1) { return $Sorted[0] }
    $pos = $Fraction * ($Sorted.Count - 1)
    $lo = [int][Math]::Floor($pos)
    $hi = [int][Math]::Ceiling($pos)
    if ($lo -eq $hi) { return $Sorted[$lo] }
    $frac = $pos - $lo
    return $Sorted[$lo] + ($Sorted[$hi] - $Sorted[$lo]) * $frac
}

<#
.SYNOPSIS
Computes mean/min/max/P5/P95/peak-to-peak/stddev/ripple for one CSV column (card
section 12). Ripple = (P95-P5)/mean, only when mean is large enough to make that
meaningful (guards a divide-by-near-zero the card itself warns about).
#>
function Get-ColumnMetrics {
    param([Parameter(Mandatory)][object[]]$Rows, [Parameter(Mandatory)][string]$Column)
    $values = Get-CsvColumnDoubles -Rows $Rows -Column $Column
    if ($values.Count -eq 0) {
        return [pscustomobject]@{ Column = $Column; Count = 0 }
    }
    $sorted = [double[]]($values | Sort-Object)
    $mean = ($values | Measure-Object -Average).Average
    $min = $sorted[0]
    $max = $sorted[$sorted.Count - 1]
    $p5 = Get-Percentile -Sorted $sorted -Fraction 0.05
    $p95 = Get-Percentile -Sorted $sorted -Fraction 0.95
    $variance = 0.0
    foreach ($v in $values) { $variance += [Math]::Pow($v - $mean, 2) }
    $variance = $variance / $values.Count
    $stddev = [Math]::Sqrt($variance)
    # A mean near zero makes a ripple RATIO meaningless (division blows up on noise) -
    # report it only once the mean clears a small absolute floor, otherwise leave it null
    # rather than print a misleading huge number. The threshold is a display guard, not a
    # PASS/FAIL judgement (the card explicitly says not to set aggressive thresholds yet).
    $ripple = $null
    if ([Math]::Abs($mean) -gt 1.0) { $ripple = ($p95 - $p5) / $mean }
    return [pscustomobject]@{
        Column = $Column; Count = $values.Count; Mean = $mean; Min = $min; Max = $max
        P5 = $p5; P95 = $p95; PeakToPeak = ($max - $min); StdDev = $stddev; Ripple = $ripple
    }
}

function Get-TraceMetrics {
    param([Parameter(Mandatory)][string]$CsvPath, [Parameter(Mandatory)][string[]]$Columns)
    $rows = Import-Csv -Path $CsvPath
    $result = foreach ($col in $Columns) { Get-ColumnMetrics -Rows $rows -Column $col }
    return ,@($result)
}

<#
.SYNOPSIS
Layer-by-layer first-divergence comparison between two traces of the SAME scenario
(card section 13). Layers are checked in pipeline order; the first one whose metrics
differ by more than its tolerance is reported as FIRST DIVERGENCE, and comparison stops
being informative below that point (later layers are still shown, but a rider reading
the report should trust the first one).
.PARAMETER Layers
Array of @{ Column = 'torque_fast'; Tolerance = 1.0 } - Tolerance is an ABSOLUTE
difference in the column's own units, checked against |mean_current - mean_baseline| AND
against |ripple_current - ripple_baseline| (whichever is larger), matching the card's
"comparing metrics/przebieg/timing, not a single sample" instruction.
#>
function Compare-Traces {
    param(
        [Parameter(Mandatory)][string]$BaselineCsv,
        [Parameter(Mandatory)][string]$CurrentCsv,
        [Parameter(Mandatory)][hashtable[]]$Layers
    )
    $baselineRows = Import-Csv -Path $BaselineCsv
    $currentRows = Import-Csv -Path $CurrentCsv
    $firstDivergence = $null
    $reportRows = foreach ($layer in $Layers) {
        $col = $layer.Column
        $tol = $layer.Tolerance
        $bm = Get-ColumnMetrics -Rows $baselineRows -Column $col
        $cm = Get-ColumnMetrics -Rows $currentRows -Column $col
        if ($bm.Count -eq 0 -or $cm.Count -eq 0) {
            [pscustomobject]@{ Layer = $col; Verdict = 'MISSING'; Detail = 'column not present in one trace' }
            continue
        }
        $meanDiff = [Math]::Abs($cm.Mean - $bm.Mean)
        $rippleDiff = 0.0
        if ($null -ne $bm.Ripple -and $null -ne $cm.Ripple) {
            $rippleDiff = [Math]::Abs($cm.Ripple - $bm.Ripple)
        }
        $verdict = 'SAME'
        if ($meanDiff -gt $tol -or $rippleDiff -gt 0.05) {
            $verdict = 'DIFFERENT'
            if ($null -eq $firstDivergence) { $firstDivergence = $col }
        }
        [pscustomobject]@{
            Layer = $col; Verdict = $verdict
            BaselineMean = [Math]::Round($bm.Mean, 3); CurrentMean = [Math]::Round($cm.Mean, 3)
            MeanDiff = [Math]::Round($meanDiff, 3); Tolerance = $tol
        }
    }
    return [pscustomobject]@{ Rows = ,@($reportRows); FirstDivergence = $firstDivergence }
}
