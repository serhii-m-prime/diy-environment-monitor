#pragma once
struct Adafruit_BME280 {
    enum { MODE_FORCED = 1, SAMPLING_X1 = 1, FILTER_OFF = 0 };
    static bool connected, measurementOk;
    static float temperature, humidity, pressure;
    inline static void (*beforeMeasurement)() = nullptr;
    bool begin(int) { return connected; }
    void setSampling(int, int, int, int, int) {}
    bool takeForcedMeasurement() { if (beforeMeasurement) beforeMeasurement(); return measurementOk; }
    float readTemperature() { return temperature; }
    float readHumidity() { return humidity; }
    float readPressure() { return pressure; }
};
