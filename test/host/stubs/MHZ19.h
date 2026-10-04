#pragma once
#include <Arduino.h>
constexpr int RESULT_OK = 1;
constexpr int RESULT_TIMEOUT = 2;
constexpr int RESULT_CRC = 3;
struct MHZ19 {
    static int co2Result, temperatureResult, ppm, temperatureCalls;
    int errorCode = RESULT_OK;
    void begin(FakeSerial&) {}
    void autoCalibration(bool) {}
    int getCO2() {
        errorCode = co2Result;
        return errorCode == RESULT_OK ? ppm : 0;
    }
    float getTemperature() {
        ++temperatureCalls;
        errorCode = temperatureResult;
        return errorCode == RESULT_OK ? 23.0f : -273.15f;
    }
};
