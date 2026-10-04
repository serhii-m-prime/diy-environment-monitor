#pragma once
#include <cstdint>
using TickType_t = uint32_t;
using BaseType_t = int;
constexpr int pdTRUE = 1, pdPASS = 1;
#define pdMS_TO_TICKS(ms) (ms)
