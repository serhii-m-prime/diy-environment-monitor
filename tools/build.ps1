param(
    [ValidateSet('esp32-s3-debug', 'esp32-s3-release')]
    [string[]]$Environment = @('esp32-s3-debug', 'esp32-s3-release'),
    [switch]$FreshConfig,
    [string]$Pio = ''
)
$ErrorActionPreference = 'Stop'
if (-not $Pio) {
    $available = Get-Command platformio -ErrorAction SilentlyContinue
    $Pio = if ($available) { $available.Source } else {
        Join-Path $env:USERPROFILE '.platformio/penv/Scripts/platformio.exe'
    }
}
Push-Location (Join-Path $PSScriptRoot '..')
$previousCoreDir = $env:PLATFORMIO_CORE_DIR
try {
    $env:PLATFORMIO_CORE_DIR = Join-Path (Get-Location).Path '.pio/idf-core'
    New-Item -ItemType Directory -Force '.pio/validation' | Out-Null
    foreach ($profile in $Environment) {
        if ($FreshConfig) {
            # These exact generated files are ignored by Git; committed presets survive.
            foreach ($generated in @("sdkconfig.$profile", "sdkconfig.$profile.old")) {
                if (Test-Path -LiteralPath $generated) { Remove-Item -LiteralPath $generated }
            }
        }
        # Windows PowerShell 5 treats even informational native stderr as an
        # ErrorRecord. Preserve that output, but decide failure by exit code.
        $previousErrorAction = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            & $Pio run -e $profile 2>&1 | ForEach-Object { $_.ToString() } |
                Tee-Object -FilePath ".pio/validation/$profile.log"
            $buildExit = $LASTEXITCODE
        } finally {
            $ErrorActionPreference = $previousErrorAction
        }
        if ($buildExit -ne 0) { throw "Firmware build failed: $profile (exit $buildExit)" }
    }
} finally {
    $env:PLATFORMIO_CORE_DIR = $previousCoreDir
    Pop-Location
}
