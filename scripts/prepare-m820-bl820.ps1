param(
    [Parameter(Mandatory = $true)]
    [string]$InputBin,

    [Parameter(Mandatory = $true)]
    [string]$OutputBin
)

$ErrorActionPreference = "Stop"

function New-Stm32Crc32Table {
    $polynomial = [uint32]79764919
    $table = New-Object 'uint32[]' 256

    for ($i = 0; $i -lt 256; $i++) {
        $value = [uint32]((([uint64]$i -shl 24) -band [uint64]4294967295))

        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($value -band [uint32]2147483648) -ne 0) {
                $value = [uint32]((((([uint64]$value) -shl 1) -bxor
                    [uint64]$polynomial) -band [uint64]4294967295))
            } else {
                $value = [uint32](((([uint64]$value) -shl 1) -band
                    [uint64]4294967295))
            }
        }

        $table[$i] = $value
    }

    return $table
}

function Update-Stm32Crc32Byte {
    param(
        [uint32]$Crc,
        [uint32]$Value,
        [uint32[]]$Table
    )

    $index = ((([uint64]$Crc -shr 24) -bxor [uint64]$Value) -band
        [uint64]255)
    return [uint32]((((([uint64]$Crc) -shl 8) -band
        [uint64]4294967295) -bxor [uint64]$Table[[int]$index]) -band
        [uint64]4294967295)
}

function Get-Stm32Crc32 {
    param([byte[]]$Data)

    $table = New-Stm32Crc32Table
    $crc = [uint32]4294967295
    $remaining = $Data.Length
    $offset = 0

    while ($remaining -ge 4) {
        $value = [uint32](
            ((([uint64]$Data[$offset]) -shl 24) -band [uint64]4278190080) -bor
            ((([uint64]$Data[$offset + 1]) -shl 16) -band [uint64]16711680) -bor
            ((([uint64]$Data[$offset + 2]) -shl 8) -band [uint64]65280) -bor
            ([uint64]$Data[$offset + 3] -band [uint64]255)
        )

        $crc = Update-Stm32Crc32Byte $crc $value $table
        $crc = Update-Stm32Crc32Byte $crc ($value -shr 8) $table
        $crc = Update-Stm32Crc32Byte $crc ($value -shr 16) $table
        $crc = Update-Stm32Crc32Byte $crc ($value -shr 24) $table

        $offset += 4
        $remaining -= 4
    }

    if ($remaining -gt 0) {
        $value = [uint32]0

        for ($i = 0; $i -lt $remaining; $i++) {
            $value = [uint32](([uint64]$value -bor
                (([uint64]$Data[$offset + $i]) -shl (24 - ($i * 8)))) -band
                [uint64]4294967295)
        }

        if ($remaining -eq 1) {
            $value = [uint32]($value -band [uint32]4278190080)
        } elseif ($remaining -eq 2) {
            $value = [uint32]($value -band [uint32]4294901760)
        } elseif ($remaining -eq 3) {
            $value = [uint32]($value -band [uint32]4294967040)
        }

        $crc = Update-Stm32Crc32Byte $crc $value $table
        $crc = Update-Stm32Crc32Byte $crc ($value -shr 8) $table
        $crc = Update-Stm32Crc32Byte $crc ($value -shr 16) $table
        $crc = Update-Stm32Crc32Byte $crc ($value -shr 24) $table
    }

    return $crc
}

function Get-Crc16CcittFalse {
    param([byte[]]$Data)

    $crc = 0

    foreach ($byte in $Data) {
        $crc = $crc -bxor (([int]$byte) -shl 8)

        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = (($crc -shl 1) -bxor 0x1021) -band 0xffff
            } else {
                $crc = ($crc -shl 1) -band 0xffff
            }
        }
    }

    return $crc
}

function Convert-HexToBytes {
    param([string]$Hex)

    $bytes = New-Object 'byte[]' ($Hex.Length / 2)
    for ($i = 0; $i -lt $bytes.Length; $i++) {
        $bytes[$i] = [Convert]::ToByte($Hex.Substring($i * 2, 2), 16)
    }
    return $bytes
}

$inputPath = (Resolve-Path -LiteralPath $InputBin).Path
$outputPath = [IO.Path]::GetFullPath($OutputBin)
$outputDirectory = Split-Path -Parent $outputPath
if ($outputDirectory) {
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
}

$raw = [IO.File]::ReadAllBytes($inputPath)
$crc32 = Get-Stm32Crc32 $raw
$crc32LittleEndian = [BitConverter]::GetBytes([uint32]$crc32)

$payload = New-Object 'byte[]' ($raw.Length + 4)
[Array]::Copy($raw, 0, $payload, 0, $raw.Length)
[Array]::Copy($crc32LittleEndian, 0, $payload, $raw.Length, 4)

$sizeModulo = $payload.Length % 65536
$crc16 = Get-Crc16CcittFalse $payload
$header = Convert-HexToBytes "0145824040000000000000000000"
$reserved = Convert-HexToBytes "0000000000000000000000000000"
$output = New-Object 'byte[]' ($header.Length + 2 + 2 +
    $reserved.Length + $payload.Length)

$outputOffset = 0
[Array]::Copy($header, 0, $output, $outputOffset, $header.Length)
$outputOffset += $header.Length
$output[$outputOffset++] = [byte](($sizeModulo -shr 8) -band 0xff)
$output[$outputOffset++] = [byte]($sizeModulo -band 0xff)
$output[$outputOffset++] = [byte](($crc16 -shr 8) -band 0xff)
$output[$outputOffset++] = [byte]($crc16 -band 0xff)
[Array]::Copy($reserved, 0, $output, $outputOffset, $reserved.Length)
$outputOffset += $reserved.Length
[Array]::Copy($payload, 0, $output, $outputOffset, $payload.Length)

[IO.File]::WriteAllBytes($outputPath, $output)

[pscustomobject]@{
    Input = $inputPath
    Output = $outputPath
    RawLength = $raw.Length
    OutputLength = $output.Length
    Stm32Crc32 = ("0x{0:X8}" -f $crc32)
    HeaderSizeModulo = ("0x{0:X4}" -f $sizeModulo)
    HeaderCrc16 = ("0x{0:X4}" -f $crc16)
}
