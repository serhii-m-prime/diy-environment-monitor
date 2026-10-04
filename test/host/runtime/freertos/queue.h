#pragma once
#include <freertos/FreeRTOS.h>
#include <vector>
#include <cstring>
struct MockQueue { std::vector<unsigned char> data; bool ready = false; };
using QueueHandle_t = MockQueue*;
inline QueueHandle_t xQueueCreate(int count, size_t bytes) {
    if (count != 1) return nullptr;
    return new MockQueue{std::vector<unsigned char>(bytes), false};
}
inline int xQueueOverwrite(QueueHandle_t queue, const void *value) {
    std::memcpy(queue->data.data(), value, queue->data.size()); queue->ready = true; return pdTRUE;
}
inline int xQueueReceive(QueueHandle_t queue, void *out, int) {
    if (!queue->ready) return 0;
    std::memcpy(out, queue->data.data(), queue->data.size()); queue->ready = false; return pdTRUE;
}
