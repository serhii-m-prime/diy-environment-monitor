# DIY environment monitor

ESP32-S3 + ST7796S (480 x 320 landscape), MH-Z19, BME280, BH1750 and PIR.

## Firmware framework

ESP-IDF 5.5.5 with Arduino 3.3.12 compiled as a component, pinned through
PIOArduino 55.03.312-1. CMake builds the SDK from source. The application owns
`app_main()` and its UI task; Arduino automatic `setup()/loop()` startup is disabled.
Only Wire and SPI Arduino libraries are selected. Sensor algorithms and UI behavior
are preserved. PWM uses the native IDF LEDC driver and a fixed XTAL clock.

PlatformIO packages and their locks live in `.pio/idf-core`, independent of other
projects. Build both profiles with `platformio run`, or open configuration with
`platformio run -e esp32-s3-debug -t menuconfig`. Committed defaults are in
`config/sdkconfig.common`, `.debug` and `.release`; app settings are under
**Environment monitor**. The ESP32-S3-DevKitC-1 target uses 8 MB flash and no PSRAM.
The explicit partition table reserves a crash-dump partition separately from RAM history.

All three application tasks and both CPU idle tasks are watched by a 10-second
task watchdog; a stalled task causes a panic/reboot with an ELF core dump in the
128 KiB `coredump` partition. Keep the matching `.pio/build/<profile>/firmware.elf`
to decode a crash. Service diagnostics include reset reason, largest free internal
heap block and the minimum unused stack bytes for input/sensor/UI tasks.
The debug profile enables heap poisoning, periodic integrity checks and a bounded
128-record leak trace. Serial commands: `?` help, `h` heap statistics, `t` start
leak tracing, `e` stop/dump tracing, `c` show the stored crash dump address/size.
Tracing and heap poisoning are compiled out of the release profile.

The ST7796S transport uses `esp_lcd` on SPI2 at the existing 80 MHz frequency,
with RGB565 data sent through DMA. Two 480 x 16-pixel internal DMA buffers use
30,720 bytes by default; no full-screen framebuffer or PSRAM is allocated.
A planning pass finds the changed rectangle, then the complete scene is clipped
into successive tiles within that rectangle. This preserves transparent text,
labels and backgrounds during incremental overview updates. Rasterization of the
next tile overlaps the previous SPI transfer. Completion is acknowledged before
buffer reuse, display commands or backlight wake-up. Hidden/sleeping pages do not
send pixel data. Service shows transfer count and buffer size. Tile height is a
menuconfig setting; physical SPI timing and actual render speed need board tests.

Dynamic frequency scaling uses 40..240 MHz, with a maximum-frequency lock while
rasterizing/transmitting a frame. Automatic light sleep is enabled through tickless
idle; internal RAM history is retained. Display-on, fan-on and active sensor I/O
hold separate no-light-sleep locks, preserving PWM and UART/I2C transactions.
Input GPIO edge interrupts wake the debouncer; 10 ms polling is limited to its
70 ms settling window. Stable/held inputs wait for notifications, with a 1-second
watchdog heartbeat. GPIO sleep wakeup uses the opposite of each current level,
so held buttons and sustained PIR do not create a wake storm. The sensor worker
waits for the next light/fan/settling deadline; the sleeping UI wakes on snapshots
and input changes. No deep sleep or flash persistence is used. DFS/light sleep and
minimum frequency are configurable under **Environment monitor**.
Actual current draw, wake behavior, UART activity and PWM continuity need board tests.

## Controls and screens

- Left button: **GPIO7 to GND**.
- Right button: **GPIO14 to GND**.
- Both use internal pull-ups and 50 ms debounce. One press advances one page;
  holding a button does not auto-repeat. Navigation wraps in both directions.
- The old GPIO19 test button is no longer configured or used.
- Screen order: environment overview, temperature, humidity, CO2, ambient light,
  pressure (mmHg), service diagnostics. Labels use the built-in ASCII bitmap font.
- The overview shows pressure in both hPa and mmHg (millimetres of mercury).
  Its four tiles use large centered readings with inline units; long values shrink
  to fit within the card. Labels stay in place during numeric updates.
- Temperature, humidity and CO2 graphs color each historical segment and the latest
  reading green/yellow/red using [documented guidance and sources](docs/measurement-colors.md).
  These are project advisory bands, not universal WHO safety limits. Ambient light
  remains neutral blue. Pressure graph colors indicate the magnitude of its 3-hour
  change: <=1.5 hPa green, >1.5 to 3.5 hPa yellow, >3.5 hPa red, in either direction.
  The pressure screen shows the signed change in mmHg, direction, and min/max/span
  of available three-minute averages over 24 hours. The absolute pressure number stays blue.
- Every graph spans 426 x 180 pixels, with the current value and its guidance (or the
  pressure 3-hour change) above, 24-hour min/max/span below and the color legend
  between the navigation labels.

The display is initially visible for 30 seconds. PIR motion keeps it on; a button
press wakes it and changes the page. After motion ends or the last button press,
there is a 30-second grace period. BH1750 is sampled once per second. Backlight
control has a 15% deadband around the accepted target (at least 3 lux), then requires
three fresh readings in the same direction spanning at least 2 seconds. Brief shadows
and repeated worker snapshots do not change the target. Invalid readings or gaps
over 1.5 seconds cancel pending confirmation and retain the accepted target.

PWM approaches the target smoothly in the main loop (1.2-second time constant),
without redrawing the UI. Wake-up fades in faster; sleep turns the backlight off
immediately. A delayed loop advances at most 50 ms of the ramp to avoid a catch-up
flash. Before the first valid reading the target is 240/4095; output remains limited
to 3..4095 when settled and enabled. Raw lux values and history are unchanged.
Sensor collection and target updates continue with the screen off.

## History and rendering

- 24 hours, one point per three minutes, 481 records in a fixed RAM ring buffer.
  Environment/CO2 measurements and live numbers still update every minute.
  Each history point averages three consecutive minute samples; the first point
  appears after the third measurement and is timestamped at that group's last sample.
  Every channel needs all three valid readings; a sensor failure invalidates only
  that channel. A missed minute invalidates the group, including gaps between groups.
  Duplicate/out-of-order snapshots do not contribute; partial groups are not stored.
- Each record is at most 16 bytes: timestamp, temperature (0.01 C), humidity
  (0.01%), CO2 (ppm), light (lux), pressure (0.1 hPa, converted to mmHg for the graph),
  and validity flags. The buffer and indices use **7,704 bytes of internal RAM**
  on ESP32, plus 28 bytes for averaging, and do not require PSRAM.
- No history writes to flash/NVS/filesystem. History is lost on reset or power loss.
- Time axis is a trailing 24-hour window with about 400 horizontal pixels.
  Vertical scale follows the valid data with a minimum span for nearly flat traces.
  Invalid averages and gaps over 270 seconds break the line; no synthetic backfill.
  Brief peaks are smoothed; min/max/span describe stored means, not raw extrema.
- Pressure tendency compares means of two adjacent history points (six minute
  measurements per endpoint), about three hours apart. It requires complete valid
  endpoint windows and a stored point no older than 270 seconds; unavailable trends
  are blue. Initial warm-up is 3 h 3 min after the first stored point, or 3 h 5 min
  after the first raw measurement under normal sampling.
  The oldest approximately three hours of a full buffer have no earlier reference
  and remain blue. Analysis uses O(N) time and constant working memory; no new buffer.
- The fan runs for 15 seconds, then stops for 2 seconds before each environment/CO2
  sample. These phases do not block the worker's light reads or the buttons/UI.
  The first sample is available after approximately 17 seconds plus sensor I/O;
  subsequent samples normally start every minute. Values show N/A before the first
  sample. The 2-second settling delay must be checked against physical fan spin-down.
- Only the selected screen is rendered, and only while the display is enabled.
  The overview updates only the numeric band of changed tiles; graphs update with
  fresh measurements/page/wake events; the service screen refreshes once per second.
  No full-screen framebuffer.
- Buttons and PIR are polled every 10 ms in a dedicated task. A separate task owns
  I2C/UART and the sensors. The Arduino loop alone owns the TFT, history and UI.
  Fixed-size, one-element FreeRTOS queues transfer snapshots without sharing
  mutable sensor/UI objects. Button counters preserve presses across overwrites.

Service shows last/max render duration, last/max complete main-loop duration
(including delay), environment/CO2 read duration, last sample age, free/minimum
heap, history occupancy/size, uptime, render passes, failed read count, PIR/fan
state and backlight duty. Timings are measured on the running device, not estimates
from a desktop test. BME280 errors do not halt the UI; initialization is retried on
subsequent minute samples. BH1750 initialization is retried after errors every 30 s.

## Pin allocation

| Function | GPIO |
| --- | --- |
| Left / right | 7 / 14 |
| PIR | 4 |
| I2C SDA / SCL | 41 / 40 |
| MH-Z19 TX / RX (ESP side) | 18 / 17 |
| TFT CS / DC / reset | 10 / 9 / 15 |
| TFT MOSI / clock | 11 / 12 |
| TFT backlight | 5 |
| Fan PWM | 6 |

The PlatformIO profile is `esp32-s3-devkitc-1` (8 MB flash, no PSRAM), pinned to
`platformio/espressif32@6.13.0` (Arduino ESP32 2.0.17, including the LEDC channel API).
Physical wiring and the exact module variant must match your hardware.

## Build and verify

```powershell
pio run -e esp32-s3-debug -e esp32-s3-release
./test/host/run.ps1 -Cxx C:/msys64/ucrt64/bin/g++.exe
```

Host tests cover history capacity/wrap/invalid samples, timer rollover, navigation,
render gating, layout bounds, graph gaps, guidance boundaries and color transitions,
pressure tendency/expiry, button/PIR debounce, sensor errors and the real worker's
fan-off settling interval using a simulated clock.
They generate SVG previews in `.pio/host-tests`; with the downloaded Adafruit GFX
library present, the previews use the same bitmap font as the device.

On hardware, check both buttons during CO2 timeouts and TFT redraws, PIR wake/sleep,
brightness and light-sensor disconnection, 24-hour history retention/overwrite,
and actual render/loop timing. Desktop tests do not prove electrical behavior or
physical display refresh performance.
