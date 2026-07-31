param(
    [ValidateSet("M820_BL820")]
    [string]$Target = "M820_BL820",

    [ValidateSet("debug", "release")]
    [string]$Profile = "debug",

    [ValidateSet("normal", "diagnostic")]
    [string]$Variant = "normal",

    [string]$Version = "",
    [string]$OutputDir = ".build",
    [string]$Toolchain = "",

    [switch]$AllowExperimentalRelease
)

# Windows PowerShell 5.1 can turn native stderr warnings into ErrorRecord objects.
# Native exit codes are checked explicitly after every tool invocation.
$ErrorActionPreference = "Continue"
$ExpectedToolchainVersion = "13.2.1"
$TargetBootloader = "820"
$TargetFlashOrigin = [uint64]0x08005000
$TargetConfigAOrigin = [uint64]0x0803E800
$TargetRamOrigin = [uint64]0x20000000

if ($Profile -eq "release" -and -not $AllowExperimentalRelease) {
    throw "Release is blocked until the ISR/shared-state audit (AUD-200..AUD-211). Use debug for hardware. -AllowExperimentalRelease is for analysis only."
}

function Get-ToolchainPath {
    param([string]$Requested)

    $candidates = @()
    if ($Requested) {
        $candidates += $Requested
    } else {
        $fromPath = Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
        if ($fromPath) {
            $candidates += (Split-Path -Parent $fromPath.Source)
        }
        $candidates +=
            "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin"
    }

    foreach ($candidate in $candidates) {
        if ($candidate -and
            (Test-Path -LiteralPath (Join-Path $candidate "arm-none-eabi-gcc.exe"))) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    throw "Arm GNU Toolchain $ExpectedToolchainVersion not found. Pass -Toolchain with its bin directory."
}

function Assert-NativeSuccess {
    param([string]$Action)

    if ($LASTEXITCODE -ne 0) {
        throw "$Action failed with exit code $LASTEXITCODE."
    }
}

function Get-GitValue {
    param(
        [string]$Repository,
        [string[]]$Arguments
    )

    $gitRepository = $Repository.Replace("\", "/")
    $result = @(& git -c "safe.directory=$gitRepository" `
        -c "core.excludesFile=" -C $Repository @Arguments)
    Assert-NativeSuccess "git $($Arguments -join ' ')"
    return ($result -join "`n").Trim()
}

function Get-SymbolAddress {
    param(
        [string[]]$Symbols,
        [string]$Name
    )

    foreach ($line in $Symbols) {
        if ($line -match "^\s*([0-9A-Fa-f]+)\s+\w\s+$([regex]::Escape($Name))$") {
            return [Convert]::ToUInt64($Matches[1], 16)
        }
    }

    throw "Required linker symbol not found: $Name"
}

$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
$toolchainPath = Get-ToolchainPath $Toolchain
$gcc = Join-Path $toolchainPath "arm-none-eabi-gcc.exe"
$objcopy = Join-Path $toolchainPath "arm-none-eabi-objcopy.exe"
$size = Join-Path $toolchainPath "arm-none-eabi-size.exe"
$nm = Join-Path $toolchainPath "arm-none-eabi-nm.exe"
$readelf = Join-Path $toolchainPath "arm-none-eabi-readelf.exe"

$toolchainVersion = ((& $gcc -dumpfullversion -dumpversion |
    Select-Object -First 1) -as [string]).Trim()
Assert-NativeSuccess "toolchain version check"
if ($toolchainVersion -ne $ExpectedToolchainVersion) {
    throw "Unsupported Arm GNU Toolchain $toolchainVersion; expected $ExpectedToolchainVersion."
}

$commit = Get-GitValue $repoRoot @("rev-parse", "HEAD")
$gitDescription = Get-GitValue $repoRoot @("describe", "--tags", "--always")
$gitRepository = $repoRoot.Replace("\", "/")
$worktreeStatus = @(& git -c "safe.directory=$gitRepository" `
    -c "core.excludesFile=" -C $repoRoot status --porcelain `
    --untracked-files=normal)
Assert-NativeSuccess "git status"
$dirty = $worktreeStatus.Count -gt 0

if (-not $Version) {
    $Version = $gitDescription
    if ($dirty) {
        $Version += "-dirty"
    }
}

if ($Version -notmatch '^[0-9A-Za-z][0-9A-Za-z._+-]{0,47}$') {
    throw "Version must contain 1..48 safe characters: letters, digits, dot, underscore, plus or minus."
}

$outputRoot = if ([IO.Path]::IsPathRooted($OutputDir)) {
    [IO.Path]::GetFullPath($OutputDir)
} else {
    [IO.Path]::GetFullPath((Join-Path $repoRoot $OutputDir))
}
$buildDir = Join-Path $outputRoot (Join-Path $Target (Join-Path $Profile $Variant))
$generatedDir = Join-Path $buildDir "generated"
$objectDir = Join-Path $buildDir "objects"
New-Item -ItemType Directory -Force -Path $generatedDir, $objectDir | Out-Null

$buildVersionHeader = @(
    "#ifndef BUILD_VERSION_H",
    "#define BUILD_VERSION_H",
    "/* Generated in the build directory; never edit or commit. */",
    "#define EBICS_BUILD_VERSION `"$Version`"",
    "#endif"
)
Set-Content -LiteralPath (Join-Path $generatedDir "build_version.h") `
    -Value $buildVersionHeader -Encoding ascii

$diagnosticsEnabled = $Variant -eq "diagnostic"
$diagnosticsValue = if ($diagnosticsEnabled) { "1" } else { "0" }

$commonFlags = @(
    "-mcpu=cortex-m4",
    "-mthumb",
    "-mfloat-abi=hard",
    "-mfpu=fpv4-sp-d16"
)
$definitions = @(
    "-DGD32F30X_HD",
    "-DGD_ECLIPSE_GCC",
    "-DUSE_STDPERIPH_DRIVER",
    "-DBOOTLOADER=$TargetBootloader",
    "-DCAN_DIAGNOSTICS_ENABLE=$diagnosticsValue"
)
$includeFlags = @(
    "-I$generatedDir",
    "-I$repoRoot\inc",
    "-I$repoRoot\Firmware\CMSIS",
    "-I$repoRoot\Firmware\CMSIS\GD\GD32F30x\Include",
    "-I$repoRoot\Firmware\GD32F30x_standard_peripheral\Include"
)
$profileFlags = if ($Profile -eq "release") {
    Write-Warning "EXPERIMENTAL RELEASE: do not flash before the ISR/shared-state audit."
    @("-Os", "-g1", "-DNDEBUG")
} else {
    @("-O0", "-g3")
}
$compilerFlags = $commonFlags + $profileFlags + @(
    "-fmessage-length=0",
    "-fsigned-char",
    "-ffunction-sections",
    "-fdata-sections",
    "-Wall"
) + $definitions + $includeFlags

$sourceManifest = Join-Path $PSScriptRoot "sources-m820.txt"
$sourceEntries = @(Get-Content -LiteralPath $sourceManifest |
    ForEach-Object { $_.Trim() } |
    Where-Object { $_ -and -not $_.StartsWith("#") })
if ($sourceEntries.Count -eq 0) {
    throw "Source manifest is empty: $sourceManifest"
}

$objects = @()
Push-Location $repoRoot
try {
    foreach ($entry in $sourceEntries) {
        $sourcePath = Join-Path $repoRoot ($entry -replace "/", "\")
        if (-not (Test-Path -LiteralPath $sourcePath -PathType Leaf)) {
            throw "Source listed in manifest does not exist: $entry"
        }

        $objectName = ($entry -replace '[\\/:]', '_') + ".o"
        $objectPath = Join-Path $objectDir $objectName
        & $gcc @compilerFlags -c $sourcePath -o $objectPath
        Assert-NativeSuccess "compile $entry"
        $objects += $objectPath
    }

    $startupSource = Join-Path $repoRoot "gcc_startup\startup_gd32f30x_hd.S"
    $startupObject = Join-Path $objectDir "startup_gd32f30x_hd.S.o"
    & $gcc @commonFlags @definitions @includeFlags -x assembler-with-cpp `
        -c $startupSource -o $startupObject
    Assert-NativeSuccess "compile startup"
    $objects += $startupObject

    $artifactBase = $Version
    $elf = Join-Path $buildDir "$artifactBase.elf"
    $bin = Join-Path $buildDir "$artifactBase.bin"
    $hex = Join-Path $buildDir "$artifactBase.hex"
    $map = Join-Path $buildDir "$artifactBase.map"
    $sizeReport = Join-Path $buildDir "$artifactBase.size.txt"
    $programHeaderReport = Join-Path $buildDir "$artifactBase.program-headers.txt"
    $manifestPath = Join-Path $buildDir "$artifactBase.manifest.json"

    $linkerScript = Join-Path $repoRoot "ldscripts\gd32f30x_flash.ld"
    $linkArguments = $commonFlags + @(
        "-T$linkerScript",
        "-Wl,--gc-sections",
        "-Wl,--print-memory-usage",
        "-Wl,-Map,$map",
        "-Wl,--start-group"
    ) + $objects + @(
        "-L$repoRoot\Firmware\CMSIS",
        "-larm_cortexM4lf_math",
        "-specs=nano.specs",
        "-specs=nosys.specs",
        "-Wl,--end-group",
        "-o",
        $elf
    )

    & $gcc @linkArguments
    Assert-NativeSuccess "link"

    & $objcopy -O binary $elf $bin
    Assert-NativeSuccess "objcopy binary"
    & $objcopy -O ihex $elf $hex
    Assert-NativeSuccess "objcopy ihex"

    $sizeOutput = @(& $size $elf)
    Assert-NativeSuccess "size summary"
    $sizeDetails = @(& $size -A $elf)
    Assert-NativeSuccess "size details"
    @($sizeOutput + "" + $sizeDetails) |
        Set-Content -LiteralPath $sizeReport -Encoding ascii

    $programHeaders = @(& $readelf -l $elf)
    Assert-NativeSuccess "readelf program headers"
    $programHeaders |
        Set-Content -LiteralPath $programHeaderReport -Encoding ascii
    if (($programHeaders -join "`n") -match '\bRWE\b') {
        throw "ELF contains an RWE load segment. See $programHeaderReport"
    }

    $symbols = @(& $nm --defined-only $elf)
    Assert-NativeSuccess "nm symbol check"
    $hasDiagnosticSymbol = @($symbols |
        Select-String -Pattern '\bprint_debug_on_CAN$').Count -gt 0
    if ($diagnosticsEnabled -and -not $hasDiagnosticSymbol) {
        throw "Diagnostic build does not contain print_debug_on_CAN."
    }
    if (-not $diagnosticsEnabled -and $hasDiagnosticSymbol) {
        throw "Normal build unexpectedly contains print_debug_on_CAN."
    }

    $appFlashStart = Get-SymbolAddress $symbols "__app_flash_start"
    $appFlashLimit = Get-SymbolAddress $symbols "__app_flash_limit"
    $flashImageEnd = Get-SymbolAddress $symbols "__flash_image_end"
    $configAStart = Get-SymbolAddress $symbols "__config_a_start"
    if ($appFlashStart -ne $TargetFlashOrigin -or
        $configAStart -ne $TargetConfigAOrigin) {
        throw "Linked memory map does not match target $Target."
    }
    if ($flashImageEnd -gt $appFlashLimit -or $appFlashLimit -gt $configAStart) {
        throw "Application Flash range overlaps Config A."
    }

    $sizeColumns = $sizeOutput[-1].Trim() -split '\s+'
    if ($sizeColumns.Count -lt 4) {
        throw "Cannot parse arm-none-eabi-size output."
    }
    $textBytes = [uint64]$sizeColumns[0]
    $gnuDataBytes = [uint64]$sizeColumns[1]
    $gnuBssBytes = [uint64]$sizeColumns[2]

    $dataStart = Get-SymbolAddress $symbols "_sdata"
    $dataEnd = Get-SymbolAddress $symbols "_edata"
    $bssStart = Get-SymbolAddress $symbols "_sbss"
    $bssEnd = Get-SymbolAddress $symbols "_ebss"
    $heapStackEnd = Get-SymbolAddress $symbols "_sp"
    $ramDataBytes = $dataEnd - $dataStart
    $ramBssBytes = $bssEnd - $bssStart
    $heapStackBytes = $heapStackEnd - $bssEnd
    $ramUsedBytes = $heapStackEnd - $TargetRamOrigin

    $bootloaderBin = Join-Path $buildDir "$artifactBase`_M820_BL820.bin"
    & (Join-Path $PSScriptRoot "prepare-m820-bl820.ps1") `
        -InputBin $bin -OutputBin $bootloaderBin | Out-Host
    if (-not (Test-Path -LiteralPath $bootloaderBin -PathType Leaf)) {
        throw "BL820 packaging did not produce $bootloaderBin"
    }

    $rawBinInfo = Get-Item -LiteralPath $bin
    $bootloaderBinInfo = Get-Item -LiteralPath $bootloaderBin
    $rawBinHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bin).Hash
    $bootloaderBinHash =
        (Get-FileHash -Algorithm SHA256 -LiteralPath $bootloaderBin).Hash

    $buildManifest = [ordered]@{
        schema_version = 1
        target = $Target
        profile = $Profile
        variant = $Variant
        diagnostics_enabled = $diagnosticsEnabled
        version = $Version
        git_commit = $commit
        git_description = $gitDescription
        worktree_dirty = $dirty
        hardware_approved_profile = $Profile -eq "debug"
        toolchain = "Arm GNU Toolchain arm-none-eabi"
        toolchain_version = $toolchainVersion
        source_manifest = "scripts/sources-m820.txt"
        source_count = $sourceEntries.Count
        flash = [ordered]@{
            origin = ("0x{0:X8}" -f $appFlashStart)
            image_end = ("0x{0:X8}" -f $flashImageEnd)
            limit = ("0x{0:X8}" -f $appFlashLimit)
            config_a = ("0x{0:X8}" -f $configAStart)
            text_bytes = $textBytes
            gnu_size_data_bytes = $gnuDataBytes
            data_load_bytes = $ramDataBytes
            binary_bytes = $rawBinInfo.Length
        }
        ram = [ordered]@{
            data_bytes = $ramDataBytes
            bss_bytes = $ramBssBytes
            heap_stack_reserved_bytes = $heapStackBytes
            used_including_heap_stack_bytes = $ramUsedBytes
            gnu_size_bss_bytes = $gnuBssBytes
        }
        artifacts = [ordered]@{
            elf = $elf
            map = $map
            size_report = $sizeReport
            program_headers = $programHeaderReport
            binary = $bin
            binary_sha256 = $rawBinHash
            bl820_binary = $bootloaderBin
            bl820_binary_bytes = $bootloaderBinInfo.Length
            bl820_binary_sha256 = $bootloaderBinHash
        }
    }
    $buildManifest | ConvertTo-Json -Depth 6 |
        Set-Content -LiteralPath $manifestPath -Encoding UTF8

    Write-Host ""
    Write-Host "Build completed:"
    Write-Host "  target/profile/variant : $Target / $Profile / $Variant"
    Write-Host "  version                : $Version"
    Write-Host "  toolchain              : $toolchainVersion"
    Write-Host "  source files           : $($sourceEntries.Count) + startup"
    Write-Host "  Flash image            : $($rawBinInfo.Length) B, end $('0x{0:X8}' -f $flashImageEnd)"
    Write-Host "  Flash limit            : $('0x{0:X8}' -f $appFlashLimit)"
    Write-Host "  RAM used/reserved      : $ramUsedBytes B (includes heap + stack)"
    Write-Host "  CAN diagnostics        : $(if ($diagnosticsEnabled) { 'ON' } else { 'OFF' })"
    Write-Host "  RWE segment            : absent"
    Write-Host "  BL820 binary           : $bootloaderBin"
    Write-Host "  BL820 SHA-256          : $bootloaderBinHash"
    Write-Host "  manifest               : $manifestPath"
}
finally {
    Pop-Location
}
