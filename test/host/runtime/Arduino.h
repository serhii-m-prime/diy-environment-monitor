#pragma once
#include "../stubs/Arduino.h"
#include <cstdlib>
inline uint32_t micros() { return static_cast<uint32_t>(testTime * 1000); }
inline void delay(uint32_t ms) { testTime += ms; }
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int, int) {}
inline unsigned mockBacklight = 0;
inline void ledcWrite(int channel, unsigned value) { if (channel == 0) mockBacklight = value; }
struct MockEsp {
    uint32_t getFreeHeap() { return 240000; }
    uint32_t getMinFreeHeap() { return 230000; }
};
inline MockEsp ESP;
