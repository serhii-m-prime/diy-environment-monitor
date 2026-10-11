param([string]$Cxx = 'g++')
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force '.pio/host-tests' | Out-Null
    & $Cxx -std=c++17 -Wall -Wextra -Itest/host/stubs -Ilib/Sensors -Ilib/App `
        test/host/sensor_regressions.cpp lib/Sensors/PIRMotionSensor.cpp `
        lib/Sensors/CO2Sensor_MHZ19.cpp lib/Sensors/LightSensor_BH1750.cpp `
        lib/Sensors/GPIOButton.cpp lib/Sensors/TempHumPressSensor_BME280.cpp `
        -o .pio/host-tests/sensor_regressions.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & ./.pio/host-tests/sensor_regressions.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensor regression tests failed' }
    & $Cxx -std=c++17 -Wall -Wextra -Ilib/App test/host/monitor_tests.cpp `
        -o .pio/host-tests/monitor_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Monitor test compilation failed' }
    & ./.pio/host-tests/monitor_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Monitor tests failed' }
    & $Cxx -std=c++11 -Wall -Wextra -Ilib/App test/host/history_average_tests.cpp `
        -o .pio/host-tests/history_average_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'History average test compilation failed' }
    & ./.pio/host-tests/history_average_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'History average tests failed' }
    & $Cxx -std=c++11 -Wall -Wextra -Ilib/App test/host/pressure_tests.cpp `
        -o .pio/host-tests/pressure_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Pressure test compilation failed' }
    & ./.pio/host-tests/pressure_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Pressure tests failed' }
    & $Cxx -std=c++11 -Wall -Wextra -Ilib/App test/host/brightness_tests.cpp `
        -o .pio/host-tests/brightness_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Brightness test compilation failed' }
    & ./.pio/host-tests/brightness_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Brightness tests failed' }
    & $Cxx -std=c++17 -Wall -Wextra -Ilib/App '-I.pio/libdeps/esp32-s3-debug/Adafruit GFX Library' test/host/ui_tests.cpp `
        -o .pio/host-tests/ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'UI test compilation failed' }
    & ./.pio/host-tests/ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'UI tests failed' }
    & $Cxx -std=c++17 -Wall -Wextra -Werror -Ilib/App '-I.pio/libdeps/esp32-s3-debug/Adafruit GFX Library' test/host/tile_tests.cpp `
        -o .pio/host-tests/tile_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Tile test compilation failed' }
    & ./.pio/host-tests/tile_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Tile tests failed' }
    & $Cxx -std=c++17 -Wall -Wextra -Itest/host/runtime -Itest/host/stubs -Ilib/Sensors -Ilib/App -Ilib/Platform `
        test/host/runtime_tests.cpp lib/Sensors/GPIOButton.cpp lib/Sensors/PIRMotionSensor.cpp `
        lib/Sensors/CO2Sensor_MHZ19.cpp lib/Sensors/LightSensor_BH1750.cpp `
        lib/Sensors/TempHumPressSensor_BME280.cpp -o .pio/host-tests/runtime_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Runtime test compilation failed' }
    & ./.pio/host-tests/runtime_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Runtime tests failed' }
} finally {
    Pop-Location
}
