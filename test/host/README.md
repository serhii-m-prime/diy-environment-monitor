# Host regression tests

Run with a C++17 host compiler:

```powershell
./test/host/run.ps1
./test/host/run.ps1 -Cxx C:/msys64/ucrt64/bin/g++.exe
```

- `sensor_regressions.cpp` compiles production button/PIR and sensor adapters
  against Arduino/library stubs. Covers debounce, missing BME280, invalid readings,
  timeout/CRC errors, recovery and retaining good data.
- `monitor_tests.cpp` covers RAM history encoding and ring wrap, navigation,
  display activity timeout, millis rollover and fan/sample scheduling.
- `ui_tests.cpp` runs the production renderer against a recording canvas. Checks
  screen bounds, no rendering while off or clean, unchanged overview tiles and
  missing-data graph gaps. Exports all six screen SVGs with synthetic readings.
  The installed Adafruit GFX bitmap font is used when available, otherwise SVG text.
- `runtime_tests.cpp` executes the actual `setup()` and `loop()` with mocked
  mailboxes and TFT transport. Checks history collection while off, wake/navigation,
  no duplicate history during light updates, retained brightness on failure and
  rendering only the selected page at its configured cadence.

Generated binaries and previews are in `.pio/host-tests`. These tests do not run
FreeRTOS scheduling, real UART/I2C, physical TFT transfers or hardware timing.
