$ErrorActionPreference = "Stop"

$zeroMv = 740
$lowDeltaMv = 146
$lowCentiKg = 600
$highDeltaMv = 1580
$highCentiKg = 8400
$spanMv = 1139
$filterMs = 35
$ticksPerMs = 4
$qOne = 256
$iqLimit = 157 # 15 A install build / 95 mA per native Iq unit

function Assert-True([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw $Message }
}

function Convert-DeltaToCentiKg([int]$DeltaMv) {
    if ($DeltaMv -le 0) { return 0 }
    if ($DeltaMv -le $lowDeltaMv) {
        return [int](($DeltaMv * $lowCentiKg + [int]($lowDeltaMv / 2)) / $lowDeltaMv)
    }
    $load = $lowCentiKg + [int]((($DeltaMv - $lowDeltaMv) *
        ($highCentiKg - $lowCentiKg) +
        [int](($highDeltaMv - $lowDeltaMv) / 2)) /
        ($highDeltaMv - $lowDeltaMv))
    return [Math]::Min($load, 12000)
}

function Simulate-Filter([int]$Target, [int]$Milliseconds) {
    $filterQ = 0
    $targetQ = $Target * $qOne
    $filterTicks = $filterMs * $ticksPerMs
    for ($tick = 0; $tick -lt ($Milliseconds * $ticksPerMs); $tick++) {
        $errorQ = $targetQ - $filterQ
        if ($errorQ -ne 0) {
            $stepQ = [int]($errorQ / $filterTicks)
            if ($stepQ -eq 0) { $stepQ = if ($errorQ -gt 0) { 1 } else { -1 } }
            $filterQ += $stepQ
        }
    }
    return [int](($filterQ + 128) / $qOne)
}

function Calculate-EmtbIq([int]$TorqueMv, [int]$Parameter, [int]$Cadence) {
    $deltaQ = [int](($TorqueMv * 160 * $qOne + [int]($spanMv / 2)) / $spanMv)
    $denominator = 510 - 2 * $Parameter
    $denominator = [Math]::Max(0, $denominator - $Cadence) + 10
    $targetQ = [int](($deltaQ * $deltaQ) / ($denominator * $qOne))
    if ($targetQ -eq 0) { return 0 }
    $targetQ = [Math]::Min($targetQ, 160 * $qOne)
    return [int](($targetQ * $iqLimit + 160 * $qOne - 1) / (160 * $qOne))
}

$zero = Convert-DeltaToCentiKg 0
$sixKg = Convert-DeltaToCentiKg (886 - $zeroMv)
$eightyFourKg = Convert-DeltaToCentiKg (2320 - $zeroMv)

Assert-True ($zero -eq 0) "740 mV must map to zero load"
Assert-True ($sixKg -eq 600) "886 mV must map to 6.00 kg"
Assert-True ($eightyFourKg -eq 8400) "2320 mV must map to 84.00 kg"
Assert-True ($zero -lt $sixKg -and $sixKg -lt $eightyFourKg) "Load mapping must be monotonic"

$filtered100ms = Simulate-Filter 1000 100
Assert-True ($filtered100ms -ge 900) "35 ms filter must reach at least 90% in 100 ms"

# 6 kg: deadband leaves 136 mV. Seeded-cadence boost uses cadence 0 and +200%.
$boostedSixKg = [Math]::Min((886 - $zeroMv - 10) * 3, $spanMv)
$emtbStartIq = Calculate-EmtbIq $boostedSixKg 180 0
$emtbRunningIq = Calculate-EmtbIq (886 - $zeroMv - 10) 180 60
Assert-True ($emtbStartIq -gt 0) "eMTB start request must not truncate to zero"
Assert-True ($emtbRunningIq -gt 0) "eMTB running request must not truncate to zero"
Assert-True ($emtbStartIq -le $iqLimit) "eMTB start request must respect phase-Iq limit"

Write-Output "FW-016 model tests passed"
Write-Output "mapping: 740mV=$($zero/100)kg, 886mV=$($sixKg/100)kg, 2320mV=$($eightyFourKg/100)kg"
Write-Output "filter@100ms=$filtered100ms/1000; eMTB start Iq=$emtbStartIq, running Iq=$emtbRunningIq, limit=$iqLimit"
