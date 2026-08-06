# FW-084 host tests — compile and run the SHIPPED C module on this PC.
#
#   powershell -File tests/host/run-host-tests.ps1
#
# Why a script: the point of these tests is that they exercise src/assist_extended_boost.c
# itself, so they need a compiler that produces something this machine can execute. If only
# the ARM cross-compiler is installed (the normal state of a firmware box) the tests cannot
# RUN — but a cross compile + link still proves the harness is valid C and that the module
# needs no stubs, so that is done instead and reported as SKIPPED, never as passed.

$ErrorActionPreference = 'Stop'

# Anything a compiler writes to stderr — every gcc warning, and the whole pile of "_close is
# not implemented" notes from the nosys stubs — is turned into a terminating error by
# 'Stop'. Native tools are therefore always invoked through this, which reports the EXIT
# CODE and nothing else. build_firmware.ps1 carries the same note for the same reason.
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
$harness = Join-Path $PSScriptRoot 'fw084_extended_boost_host.c'
$module = Join-Path $root 'src\assist_extended_boost.c'
$inc = Join-Path $root 'inc'
$out = Join-Path $env:TEMP 'fw084_host_tests.exe'

function Find-HostCompiler {
    foreach ($name in @('gcc.exe', 'clang.exe', 'cc.exe')) {
        $found = Get-Command $name -ErrorAction SilentlyContinue
        if ($found) { return @{ Kind = 'gcc'; Path = $found.Source } }
    }
    $cl = Get-Command 'cl.exe' -ErrorAction SilentlyContinue
    if ($cl) { return @{ Kind = 'msvc'; Path = $cl.Source } }
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
    if ($host_cc.Kind -eq 'gcc') {
        $built = Invoke-Native $host_cc.Path @(
            '-std=c11', '-Wall', '-Wextra', '-Werror', "-I$inc", '-o', $out, $harness, $module)
    } else {
        Push-Location $env:TEMP
        try {
            $built = Invoke-Native $host_cc.Path @(
                '/nologo', '/W4', '/WX', "/I$inc", "/Fe:$out", $harness, $module)
        } finally { Pop-Location }
    }
    if ($built -ne 0) { throw 'FW-084 host harness failed to build' }
    $code = Invoke-Native $out @()
    Remove-Item $out -ErrorAction SilentlyContinue
    if ($code -ne 0) { throw "FW-084 host tests FAILED" }
    Write-Host 'FW-084 host tests: PASS'
    exit 0
}

$cross = Find-CrossCompiler
if (-not $cross) {
    Write-Warning 'No C compiler found at all. FW-084 host tests were NOT run.'
    exit 2
}

Write-Warning 'No host C compiler on this machine — the FW-084 host tests were NOT RUN.'
Write-Host "Falling back to a cross compile + link with $cross (syntax and linkage only)."
$elf = Join-Path $env:TEMP 'fw084_host_tests.elf'
$built = Invoke-Native $cross @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', "-I$inc", '-mcpu=cortex-m4', '-mthumb',
    '--specs=nosys.specs', '--specs=nano.specs', '-o', $elf, $harness, $module) -Quiet
if ($built -ne 0) { throw 'FW-084 host harness does not compile' }
Remove-Item $elf -ErrorAction SilentlyContinue
Write-Host 'FW-084 host harness: COMPILES AND LINKS (behaviour NOT verified — SKIPPED).'
Write-Host 'Install MinGW-w64, LLVM or MSVC to actually run these tests.'
exit 2
