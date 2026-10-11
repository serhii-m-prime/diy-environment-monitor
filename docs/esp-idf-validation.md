# Перенесення монітора на ESP-IDF

Дата перевірки: 2026-10-11. Гілка: `codex/esp-idf-monitor`.
Поточні зміни до міграції збережено комітом `5c2b8f4`; реалізація й виправлення збірки — до `e2c51ec`.

ESP-IDF 5.5.5 збирається із джерел; Arduino 3.3.12 підключено як компонент для чинних драйверів датчиків, SPI та Wire. Застосунок має власний `app_main()` і три задачі. Ціль — ESP32-S3-DevKitC-1-N8, 8 MB flash, без PSRAM.

| Пункт | Реалізація |
| --- | --- |
| Діагностика | Watchdog 10 с для трьох задач і двох idle-задач; ELF core dump у flash; reset reason, найбільший блок heap і залишок стеків на service-сторінці. Debug має heap poisoning, перевірку цілісності й 128 записів heap trace; release виключає tracing/poisoning. |
| TFT через DMA | ST7796S, SPI2 80 MHz, два буфери 480 × 16 RGB565: 30 720 байтів RAM. Передаються tiles у зміненому прямокутнику; завершення DMA підтверджується до повторного використання буфера. Повного framebuffer немає. |
| Конфігурація IDF | CMake, Kconfig, спільні й окремі debug/release presets, налаштування задач і розміру tiles. Factory app: 3 MiB з 0x10000; core dump: 128 KiB з 0x310000. Пакети та артефакти ізольовано в .pio. |
| Енергоспоживання | DFS 40–240 MHz і automatic light sleep. Display/fan/sensor I/O утримують окремі locks; render — максимальну частоту CPU. GPIO wakeup налаштовується на протилежний рівень; ISR маскує пін до повторного налаштування, debounce має короткий період polling. |

24-годинна RAM-історія, 481 точка з трьохвилинними середніми, хвилинні live-значення, 15 с fan / 2 с settling та render gating збережені.

## Перевірки

Усі дев'ять host-наборів пройшли. Серед них — replay понад 24 години в реальному loop/worker, помилки датчиків, rollover, debounce/held inputs, fan/settling deadlines, geometry/render gating, піксельна відповідність повного й tiled-рендера для семи сторінок та incremental/sleep/wake. Старі попередження signed comparison в State.h залишаються в host-збірці.

Первинні native-збірки й перевірка ELF/BIN до виправлення яскравості:

| Профіль | Результат | BIN застосунку, байтів | Статична RAM, байтів |
| --- | --- | ---: | ---: |
| debug | PASS | 474304 | 43036 |
| release | PASS | 407168 | 34528 |

Обидві прошивки мають project version `e2c51ec`. Статична RAM не включає два DMA-буфери (30720 байтів), стеки трьох задач (17408 байтів) та інші runtime allocations.

`tools/verify_firmware.py` підтвердив у generated sdkconfig watchdog/core dump/PM/tickless/DMA-параметри, лише SPI/Wire з Arduino libraries, таблицю розділів і вкладення BIN у 3 MiB app partition. У ELF обох профілів є `esp_task_wdt_add/reset`, `esp_core_dump_write_start`, `spi_bus_initialize`, `esp_lcd_panel_io_tx_color`, `esp_pm_configure`, PM locks і `gpio_wakeup_enable`. Heap trace start/stop/dump є лише в debug. Повний результат і hashes — `.pio/validation/firmware.json`.

SHA-256 первинних артефактів версії `e2c51ec`:

```text
debug BIN  75c4328ce1715688bdef75a0bb0688190a93240e4294d491e2754bbe8609bf9e
debug ELF  993a6a1fc1ad812310427820904ead649ce0ec9622aba1952565aa4b9bc98966
release BIN 265b2a55880e0f729278e3dbab14e78250794fa8acaa7f33b350d71f81a4eb74
release ELF efed576e8202b2c2e738114a7a464f9436c09b6aaf57e026d582c77e7201dfda
```

Перевірено й combined `firmware.factory.bin`: його bootloader, partitions та app збігаються з окремими перевіреними файлами за offset 0, 0x8000 та 0x10000. Артефакти розміщено в `.pio/native-build/esp32-s3-debug` та `.pio/native-build/esp32-s3-release`. Логи — `.pio/validation/<profile>.log`, host — `.pio/host-final.log`.

Відтворення:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -FreshConfig
powershell -NoProfile -ExecutionPolicy Bypass -File test/host/run.ps1 -Cxx C:/msys64/ucrt64/bin/g++.exe
python tools/verify_firmware.py
```

## Виправлення яскравості після перевірки на пристрої

Коміт коду: `19227c8`. Користувач повідомив 30 lux / LED 115 та значне
зменшення яскравості після міграції. Проба з попереднім контролером
`DisplayBrightness` з коміту `5784122` відтворила усталений target/PWM 120:
це лише 2,930% на окремому 12-бітному таймері IDF.

У baseline `5c2b8f4` backlight channel 0 і fan channel 1 ділили timer 0.
Arduino 2.0.17 обирає timer як `(channel / 2) % 4`; другий `ledcSetup`
для вентилятора встановлював 8 біт замість початкових 12. Тому ті самі
120 відліків давали приблизно 46,875% до міграції.
Підтверджено локальним HAL та [джерелом Espressif Arduino 2.0.17](https://github.com/espressif/arduino-esp32/blob/2.0.17/cores/esp32/esp32-hal-ledc.c).

Виправлено шкалу на незалежному 12-бітному таймері: 64 відліки/lux,
мінімум 48, fallback 3840, максимум 4095; абсолютний deadband теж
масштабовано до 192. При 30 lux нова проба і реальний loop у host-тесті
дають 1920/4096 (46,875%), приблизно від 64 lux — максимум.
Гістерезис, підтвердження, плавність, збереження target при помилках
датчика та sleep/wake перевірено; всі дев'ять host-наборів пройшли.
Лог: `.pio/brightness-host.log`.

Обидві native-збірки пройшли повторно; у BIN перевірено app description
з версією `19227c8`, а також конфігурацію, native symbols, partitions та
відповідність усіх частин factory image. Лог: `.pio/brightness-native-build.log`.
Поточні artifact hashes: `.pio/validation/firmware.json` і
`.pio/validation/brightness-artifacts.json`.

| Профіль `19227c8` | Результат | BIN застосунку, байтів | Статична RAM, байтів |
| --- | --- | ---: | ---: |
| debug | PASS | 474304 | 43052 |
| release | PASS | 407184 | 34532 |

SHA-256 поточних BIN:

```text
debug BIN   d661252d83bb3b96ef2f8a261ee87cdb2dfb923fc8c2cdbe76d619fdd68da28e
release BIN 88d90dd550074fcd174287c585c910a740145f6067ef8dc1bfc9fd70bf0cdaad
```

## Межа перевірки

Виправлену прошивку `19227c8` ще не перевірено на платі. Watchdog panic/core dump recovery, реальну швидкість TFT, SPI 80 MHz, енергоспоживання, light-sleep wakeup кнопками/PIR, UART/I2C після пробудження та PWM/час фізичного зупинення вентилятора потрібно перевірити на пристрої. Desktop tests, native build і ELF/BIN checks підтверджують код та артефакти, але не ці фізичні ефекти.

