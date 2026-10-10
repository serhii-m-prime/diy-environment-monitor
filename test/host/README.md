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
  display activity timeout, millis rollover, fan/settling/sample scheduling and exact
  temperature/humidity/CO2 guidance boundaries, including neutral pressure.
- `pressure_tests.cpp` covers 3-hour pressure deltas, five-sample averaging,
  intensity/direction boundaries, warm-up, stale/invalid/missing samples, timestamp
  jitter, circular overwrite and timer rollover. This target uses C++11, matching
  the firmware's language standard.
- `brightness_tests.cpp` (C++11) covers deadband boundaries, transient shadows,
  confirmation/reset, smooth monotonic ramps, invalid data, saturation, sleep/wake
  and clock rollover. Runtime tests also verify that repeated light snapshots do
  not satisfy confirmation and PWM changes alone never redraw the selected page.
- `ui_tests.cpp` runs the production renderer against a recording canvas. Checks
  screen bounds, no rendering while off or clean, unchanged overview tiles and
  missing-data graph gaps and pressure in mmHg. Exports all seven screen SVGs with synthetic readings.
  The installed Adafruit GFX bitmap font is used when available, otherwise SVG text.
  Overview checks include text containment and overlap at normal/maximum readings,
  negative temperature and N/A, plus previews of partial updates after digit-count changes.
  Also checks rising/falling multi-band transitions, flat boundary values,
  invalid data colors, unavailable pressure trends and colored isolated samples.
  Pressure previews cover historical tendency bands, min/max/span, rising/falling
  arrows, steady conditions and stale data.
- `runtime_tests.cpp` executes the actual `setup()` and `loop()` with mocked
  mailboxes and TFT transport. Checks history collection while off, wake/navigation,
  no duplicate history during light updates, retained brightness on failure and
  rendering only the selected page at its configured cadence, and a single redraw
  when a pressure tendency expires. A bounded execution of the real sensor worker
  verifies that BME measurements occur after 15 seconds of fan operation and
  2 seconds of settling, with minute sample cadence.

Generated binaries and previews are in `.pio/host-tests`. These tests do not run
FreeRTOS scheduling, real UART/I2C, physical TFT transfers or hardware timing.
