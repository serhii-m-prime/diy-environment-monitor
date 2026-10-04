#pragma once
#include <freertos/FreeRTOS.h>
inline int xTaskCreatePinnedToCore(void (*)(void*), const char*, int, void*, int, void*, int) { return pdPASS; }
inline TickType_t xTaskGetTickCount() { return 0; }
inline void vTaskDelayUntil(TickType_t*, TickType_t) {}
inline void (*mockTaskDelay)(TickType_t) = nullptr;
inline void vTaskDelay(TickType_t ticks) { if (mockTaskDelay) mockTaskDelay(ticks); }
