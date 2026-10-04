# DIY environment monitor

ESP32-S3 + ST7796S (480 x 320 landscape), MH-Z19, BME280, BH1750 and PIR.

## Controls and screens

- Left button: **GPIO7 to GND**.
- Right button: **GPIO14 to GND**.
- Both use internal pull-ups and 50 ms debounce. One press advances one page;
  holding a button does not auto-repeat. Navigation wraps in both directions.
- The old GPIO19 test button is no longer configured or used.
- Screen order: environment overview, temperature, humidity, CO2, ambient light,
  service diagnostics. Labels use the built-in ASCII bitmap font.

The display is initially visible for 30 seconds. PIR motion keeps it on; a button
press wakes it and changes the page. After motion ends or the last button press,
there is a 30-second grace period. BH1750 controls brightness once per second.
A failed light reading retains the last valid brightness; before any valid reading
there is a fixed fallback duty of 240/4095. Sensor collection continues with the
screen off.

## History and rendering

- Six hours, one point per minute, 361 records in a fixed RAM ring buffer.
- Each record is at most 16 bytes: timestamp, temperature (0.01 C), humidity
  (0.01%), CO2 (ppm), light (lux), and validity flags. The buffer uses about
  **5.7 KiB of internal RAM** and does not require PSRAM.
- No history writes to flash/NVS/filesystem. History is lost on reset or power loss.
- Time axis is a trailing six-hour window with about 400 horizontal pixels.
  Vertical scale follows the valid data with a minimum span for nearly flat traces.
  Invalid readings and gaps over 90 seconds break the line; no synthetic backfill.
- The fan runs for 15 seconds before each environment/CO2 sample, including the
  first sample after startup. Before the first sample those values show N/A.
- Only the selected screen is rendered, and only while the display is enabled.
  The overview updates changed tiles; graphs update with history/page/wake events;
  the service screen refreshes once per second. No full-screen framebuffer.
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

The PlatformIO profile is `esp32-s3-devkitc-1` (8 MB flash, no PSRAM).
Physical wiring and the exact module variant must match your hardware.

## Build and verify

```powershell
pio run -e esp32-s3-debug -e esp32-s3-release
./test/host/run.ps1 -Cxx C:/msys64/ucrt64/bin/g++.exe
```

Host tests cover history capacity/wrap/invalid samples, timer rollover, navigation,
render gating, layout bounds, graph gaps, button/PIR debounce and sensor errors.
They generate SVG previews in `.pio/host-tests`; with the downloaded Adafruit GFX
library present, the previews use the same bitmap font as the device.

On hardware, check both buttons during CO2 timeouts and TFT redraws, PIR wake/sleep,
brightness and light-sensor disconnection, six-hour history retention/overwrite,
and actual render/loop timing. Desktop tests do not prove electrical behavior or
physical display refresh performance.
