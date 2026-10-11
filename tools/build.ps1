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
try {
    New-Item -ItemType Directory -Force '.pio/validation' | Out-Null
    foreach ($profile in $Environment) {
        if ($FreshConfig) {
            # These exact generated files are ignored by Git; committed presets survive.
            foreach ($generated in @("sdkconfig.$profile", "sdkconfig.$profile.old")) {
                if (Test-Path -LiteralPath $generated) { Remove-Item -LiteralPath $generated }
            }
        }
        & $Pio run -e $profile 2>&1 | Tee-Object -FilePath ".pio/validation/$profile.log"
        if ($LASTEXITCODE -ne 0) { throw "Firmware build failed: $profile" }
    }
} finally { Pop-Location }
