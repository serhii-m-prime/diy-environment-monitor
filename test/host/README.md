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
- `pressure_tests.cpp` covers 3-hour pressure deltas over paired three-minute means,
  intensity/direction boundaries, warm-up, stale/invalid/missing samples, timestamp
  jitter, circular overwrite and timer rollover. Core algorithms retain C++11
  compatibility; the SDK uses its configured modern C++ standard.
- `history_average_tests.cpp` (C++11) covers three-minute means, signed rounding,
  per-channel invalid data, missed samples, duplicate/out-of-order snapshots,
  outage recovery and timestamp rollover.
- `brightness_tests.cpp` (C++11) covers deadband boundaries, transient shadows,
  confirmation/reset, smooth monotonic ramps, invalid data, saturation, sleep/wake
  and clock rollover. Runtime tests also verify that repeated light snapshots do
  not satisfy confirmation and PWM changes alone never redraw the selected page.
- `ui_tests.cpp` runs the production renderer against a recording canvas. Checks
  screen bounds, no rendering while off or clean, unchanged overview tiles and
  missing-data graph gaps and pressure in mmHg. Exports all seven screen SVGs with synthetic readings.
  The committed Adafruit classic bitmap font is always used, matching the device.
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
  when a pressure tendency expires. Replays over 24 hours of minute samples through
  the real loop to check averaged points, fresh live readings, ring overwrite and
  pressure analysis with the display off. A bounded execution of the real sensor worker
  verifies that BME measurements occur after 15 seconds of fan operation and
  2 seconds of settling, with minute sample cadence.
- `tile_tests.cpp` compares a complete software framebuffer with clipped tile
  replay for all seven pages, partial overview updates and sleep/wake. Checks
  buffer boundaries, odd widths, clipping and RGB565 wire byte order. Framebuffers
  are used by this host test only; production allocates two small DMA tiles.
- `power_policy_tests.cpp` covers notification/debounce waits, held-input wake
  levels, delayed fan-off settling deadlines and time rollover.

Generated binaries and previews are in `.pio/host-tests`. These tests do not run
FreeRTOS scheduling, real UART/I2C, physical TFT transfers or hardware timing.
