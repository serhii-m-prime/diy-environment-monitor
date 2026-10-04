#pragma once
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
using String = std::string;
constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int INPUT = 0;
constexpr int SERIAL_8N1 = 0;
extern unsigned long testTime;
extern int testPin;
inline unsigned long millis() { return testTime; }
inline int digitalRead(uint8_t) { return testPin; }
inline void pinMode(uint8_t, int) {}
struct FakeSerial { void begin(int, int, int, int) {} };
extern FakeSerial Serial2;
