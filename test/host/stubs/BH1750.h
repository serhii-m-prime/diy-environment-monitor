#pragma once
struct BH1750 {
    static bool ready, configured;
    static float lux;
    enum { CONTINUOUS_HIGH_RES_MODE = 0 };
    bool begin(int) { return configured; }
    bool measurementReady() { return ready; }
    float readLightLevel() { return configured ? lux : -2.0f; }
};
