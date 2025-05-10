#pragma once

struct SensorData {
    virtual ~SensorData() = default;
};

struct CO2Data : public SensorData {
    int ppm;
    float temperature;
};

struct TempHumPressureData : public SensorData {
    float temperature;
    float humidity;
    float pressure;
};

struct TempHumData : public SensorData {
    float temperature;
    float humidity;
};

struct LightData : public SensorData {
    float lux;
};