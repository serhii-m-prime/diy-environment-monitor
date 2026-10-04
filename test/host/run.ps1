param([string]$Cxx = 'g++')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force '.pio/host-tests' | Out-Null
    & $Cxx -std=c++17 -Wall -Wextra -Itest/host/stubs -Ilib/Sensors -Ilib/App `
        test/host/sensor_regressions.cpp lib/Sensors/PIRMotionSensor.cpp `
        lib/Sensors/CO2Sensor_MHZ19.cpp lib/Sensors/LightSensor_BH1750.cpp `
        -o .pio/host-tests/sensor_regressions.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & ./.pio/host-tests/sensor_regressions.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensor regression tests failed' }
} finally {
    Pop-Location
}
