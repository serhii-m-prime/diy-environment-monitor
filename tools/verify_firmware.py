"""Check generated SDK configuration, partitions and linked ESP-IDF features."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def verify(root, profile, nm):
    build = root / profile
    config = json.loads((build / "config/sdkconfig.json").read_text())
    expected = {
        "IDF_TARGET": "esp32s3", "ESPTOOLPY_FLASHSIZE": "8MB",
        "FREERTOS_HZ": 1000, "ESP_TASK_WDT_EN": True,
        "ESP_TASK_WDT_INIT": True, "ESP_TASK_WDT_PANIC": True,
        "ESP_TASK_WDT_TIMEOUT_S": 10, "ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0": True,
        "ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1": True,
        "ESP_COREDUMP_ENABLE_TO_FLASH": True, "ESP_COREDUMP_DATA_FORMAT_ELF": True,
        "ESP_COREDUMP_STACK_SIZE": 4096, "PM_ENABLE": True,
        "ESP_DEFAULT_CPU_FREQ_MHZ": 240, "FREERTOS_USE_TICKLESS_IDLE": True,
        "MONITOR_POWER_MANAGEMENT": True, "MONITOR_AUTO_LIGHT_SLEEP": True,
        "MONITOR_MIN_CPU_MHZ": 40, "MONITOR_DMA_TILE_ROWS": 16,
        "MONITOR_INPUT_STACK_BYTES": 3072, "MONITOR_SENSOR_STACK_BYTES": 6144,
        "MONITOR_UI_STACK_BYTES": 8192, "ARDUINO_SELECTIVE_COMPILATION": True,
        "ARDUINO_SELECTIVE_SPI": True, "ARDUINO_SELECTIVE_Wire": True,
    }
    for key, value in expected.items():
        require(config.get(key) == value, f"{profile}: {key} != {value!r}")
    for key in ("AUTOSTART_ARDUINO", "SPIRAM", "BT_ENABLED"):
        require(not config.get(key), f"{profile}: unexpected {key}")
    for key, value in config.items():
        if key.startswith("ARDUINO_SELECTIVE_") and key not in (
            "ARDUINO_SELECTIVE_COMPILATION", "ARDUINO_SELECTIVE_SPI", "ARDUINO_SELECTIVE_Wire"
        ):
            require(not value, f"{profile}: unexpected Arduino library {key}")
    debug = profile.endswith("debug")
    for key in ("HEAP_TRACING_STANDALONE", "HEAP_POISONING_LIGHT"):
        require(bool(config.get(key)) == debug, f"{profile}: wrong {key}")
    require(config.get("COMPILER_OPTIMIZATION_DEBUG" if debug else "COMPILER_OPTIMIZATION_SIZE"),
            f"{profile}: wrong optimization")

    elf, binary = build / "firmware.elf", build / "firmware.bin"
    output = subprocess.check_output([str(nm), "--defined-only", str(elf)], text=True)
    symbols = {line.split()[-1] for line in output.splitlines() if line.split()}
    native = ("esp_task_wdt_add", "esp_task_wdt_reset", "esp_core_dump_write_start",
              "spi_bus_initialize", "esp_lcd_panel_io_tx_color", "esp_pm_configure",
              "esp_pm_lock_acquire", "esp_pm_lock_release", "gpio_wakeup_enable")
    for symbol in native:
        require(symbol in symbols, f"{profile}: missing native symbol {symbol}")
    for symbol in ("heap_trace_start", "heap_trace_stop", "heap_trace_dump"):
        require((symbol in symbols) == debug, f"{profile}: wrong trace linkage {symbol}")

    partitions_file = build / "partitions.bin"
    partitions = {}
    data = partitions_file.read_bytes()
    for offset in range(0, len(data), 32):
        record = data[offset:offset + 32]
        if len(record) != 32 or record[:2] != b"\xaa\x50":
            break
        _, kind, subtype, start, size, label, flags = struct.unpack("<HBBII16sI", record)
        name = label.rstrip(b"\x00").decode()
        partitions[name] = dict(type=kind, subtype=subtype, offset=start, size=size, flags=flags)
    require(partitions["factory"]["offset"] == 0x10000 and
            partitions["factory"]["size"] == 0x300000, f"{profile}: wrong app partition")
    require(partitions["coredump"]["offset"] == 0x310000 and
            partitions["coredump"]["size"] == 0x20000 and
            partitions["coredump"]["subtype"] == 3, f"{profile}: wrong core dump partition")
    require(binary.stat().st_size <= partitions["factory"]["size"], f"{profile}: app too large")
    artifacts = {}
    for path in (elf, binary, partitions_file, build / "bootloader.bin"):
        artifacts[path.name] = dict(bytes=path.stat().st_size,
                                   sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return dict(profile=profile, checked_config=expected, debug_heap_trace=debug,
                dma_buffer_bytes=480 * config["MONITOR_DMA_TILE_ROWS"] * 2 * 2,
                native_symbols=list(native), partitions=partitions, artifacts=artifacts,
                hardware_tested=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-root", type=Path, default=Path(".pio/native-build"))
    parser.add_argument("--nm", type=Path, default=Path(
        ".pio/idf-core/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-nm.exe"))
    parser.add_argument("--output", type=Path, default=Path(".pio/validation/firmware.json"))
    args = parser.parse_args()
    results = [verify(args.build_root, profile, args.nm) for profile in (
        "esp32-s3-debug", "esp32-s3-release")]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2) + "\n")
    for result in results:
        print(f"{result['profile']}: configuration, native symbols and partitions verified; "
              f"app={result['artifacts']['firmware.bin']['bytes']} bytes")
    print(f"Artifact hashes: {args.output}")
