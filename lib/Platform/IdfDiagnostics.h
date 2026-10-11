#pragma once
#include <MonitorModel.h>
#include <cinttypes>

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <esp_core_dump.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/task.h>
#if CONFIG_HEAP_TRACING_STANDALONE
#include <esp_heap_trace.h>
#endif
#endif

namespace monitor {
#ifdef ESP_PLATFORM
inline void allocationFailed(size_t bytes, uint32_t caps, const char *function) {
    ESP_EARLY_LOGE("memory", "allocation failed: %u B caps=%" PRIu32 " at %s",
                   static_cast<unsigned>(bytes), caps, function);
}
inline void initializeDiagnostics() {
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(allocationFailed));
    ESP_LOGI("diagnostics", "reset=%d IDF=%s", static_cast<int>(esp_reset_reason()), esp_get_idf_version());
}
inline void watchCurrentTask() { ESP_ERROR_CHECK(esp_task_wdt_add(nullptr)); }
inline void taskHeartbeat() { ESP_ERROR_CHECK(esp_task_wdt_reset()); }

class RuntimeDiagnostics {
public:
    void collect(Diagnostics &d, TaskHandle_t input, TaskHandle_t sensors, TaskHandle_t ui) {
        d.freeHeap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        d.minHeap = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        d.largestHeapBlock = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        d.inputStackFree = input ? uxTaskGetStackHighWaterMark(input) : 0;
        d.sensorStackFree = sensors ? uxTaskGetStackHighWaterMark(sensors) : 0;
        d.uiStackFree = ui ? uxTaskGetStackHighWaterMark(ui) : 0;
        d.resetReason = static_cast<uint8_t>(esp_reset_reason());
    }
    void poll(uint32_t now) {
#if CONFIG_HEAP_POISONING_LIGHT || CONFIG_HEAP_POISONING_COMPREHENSIVE
        if (uint32_t(now - checked_) >= 5000) {
            checked_ = now;
            if (!heap_caps_check_integrity_all(true)) abort();
        }
#endif
#ifdef DEBUG_ENABLED
        // A bounded nonblocking console. No input or display task owns Serial.
        for (unsigned i = 0; i < 8 && Serial.available(); ++i) command(Serial.read());
#else
        (void)now;
#endif
    }
private:
#ifdef DEBUG_ENABLED
    void command(int input) {
        if (input == 'h') heap_caps_print_heap_info(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (input == 'c') {
            size_t address = 0, size = 0;
            const auto result = esp_core_dump_image_get(&address, &size);
            Serial.printf("Core dump: %s address=0x%x size=%u; preserve the matching firmware.elf\n",
                          esp_err_to_name(result), unsigned(address), unsigned(size));
        }
#if CONFIG_HEAP_TRACING_STANDALONE
        if (input == 't' && !tracing_) {
            ESP_ERROR_CHECK(heap_trace_init_standalone(records_, TraceRecords));
            ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
            tracing_ = true;
            Serial.println("Heap leak trace started (128 records). Send e to stop/dump.");
        }
        if (input == 'e' && tracing_) {
            ESP_ERROR_CHECK(heap_trace_stop());
            tracing_ = false;
            heap_trace_dump();
        }
#endif
        if (input == '?') Serial.println("h: heap, t: start leak trace, e: stop/dump, c: crash dump location");
    }
#endif
#if CONFIG_HEAP_TRACING_STANDALONE
    static constexpr size_t TraceRecords = 128;
    heap_trace_record_t records_[TraceRecords]{};
    bool tracing_ = false;
#endif
    uint32_t checked_ = 0;
};
#else
inline void initializeDiagnostics() {}
inline void watchCurrentTask() {}
inline void taskHeartbeat() {}
#endif
} // namespace monitor
