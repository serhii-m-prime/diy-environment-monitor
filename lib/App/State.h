#pragma once
#include <Arduino.h>
#include <SensorData.h>

class State
{
public:
    State()
    {
        _co2 = 0;
        _co2LastUpdateTime = 0;
        _temperature = 0.0;
        _temperatureLastUpdateTime = 0;
        _humidity = 0.0;
        _humidityLastUpdateTime = 0;
        _pressure = 0.0;
        _pressureLastUpdateTime = 0;
        _lightLevel = 0.0;
        _lightLevelLastUpdateTime = 0;
        _isDisplayOn = false;
        _isMotionDetected = false;
        _displayPage = 0;
    }
    void fill(CO2Data *co2Data)
    {
        _co2 = co2Data->ppm;
        _co2LastUpdateTime = millis();
    }
    void fill(TempHumPressureData *tempHumPressData)
    {
        _temperature = tempHumPressData->temperature;
        _humidity = tempHumPressData->humidity;
        _pressure = tempHumPressData->pressure;
        _temperatureLastUpdateTime = millis();
        _humidityLastUpdateTime = millis();
        _pressureLastUpdateTime = millis();
    }
    void fill(LightData *lightSensorData)
    {
        _lightLevel = lightSensorData->lux;
        _lightLevelLastUpdateTime = millis();
    }
    void setCO2(int co2)
    {
        _co2 = co2;
        _co2LastUpdateTime = millis();
    }
    void setTemperature(float temperature)
    {
        _temperature = temperature;
        _temperatureLastUpdateTime = millis();
    }
    void setHumidity(float humidity)
    {
        _humidity = humidity;
        _humidityLastUpdateTime = millis();
    }
    void setPressure(float pressure)
    {
        _pressure = pressure;
        _pressureLastUpdateTime = millis();
    }
    void setLightLevel(float lightLevel)
    {
        _lightLevel = lightLevel;
        _lightLevelLastUpdateTime = millis();
    }
    int getCO2() { return _co2; }
    float getTemperature() { return _temperature; }
    float getHumidity() { return _humidity; }
    float getPressure() { return _pressure; }
    float getLightLevel() { return _lightLevel; }
    bool isDisplayOn() { return _isDisplayOn; }
    void setDisplayOn(bool displayOn)
    {
        _isDisplayOn = displayOn;
    }
    bool isMotionDetected() { return _isMotionDetected; }
    void setMotionDetected(bool motionDetected)
    {
        _isMotionDetected = motionDetected;
    }
    bool isDataUpdated(int expiration_ms)
    {
        return (millis() - _co2LastUpdateTime < expiration_ms) ||
               (millis() - _temperatureLastUpdateTime < expiration_ms) ||
               (millis() - _humidityLastUpdateTime < expiration_ms) ||
               (millis() - _pressureLastUpdateTime < expiration_ms) ||
               (millis() - _lightLevelLastUpdateTime < expiration_ms);
    }
    bool isCO2Actual(int expiration_ms)
    {
        return (millis() - _co2LastUpdateTime < expiration_ms);
    }
    bool isTemperatureActual(int expiration_ms)
    {
        return (millis() - _temperatureLastUpdateTime < expiration_ms);
    }
    bool isHumidityActual(int expiration_ms)
    {
        return (millis() - _humidityLastUpdateTime < expiration_ms);
    }
    bool isPressureActual(int expiration_ms)
    {
        return (millis() - _pressureLastUpdateTime < expiration_ms);
    }
    bool isLightLevelActual(int expiration_ms)
    {
        return (millis() - _lightLevelLastUpdateTime < expiration_ms);
    }

    void setNextPage()
    {
        _displayPage++;
        if (_displayPage > _displayPageMax)
        {
            _displayPage = _displayPageMin;
        }
    }

    void flushData()
    {
        _co2 = 0;
        _temperature = 0.0;
        _humidity = 0.0;
        _pressure = 0.0;
        _lightLevel = 0.0;
    }

    String toString() const {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
        "CO2: %u ppm, Temperature: %.2f °C, Humidity: %.2f %%, "
        "Pressure: %.2f hPa, Light Level: %.2f lx",
        _co2, _temperature, _humidity, _pressure, _lightLevel);
    return String(buf);
}


private:
    int _co2;
    unsigned long _co2LastUpdateTime;
    float _temperature;
    unsigned long _temperatureLastUpdateTime;
    float _humidity;
    unsigned long _humidityLastUpdateTime;
    float _pressure;
    unsigned long _pressureLastUpdateTime;
    float _lightLevel;
    unsigned long _lightLevelLastUpdateTime;
    bool _isDisplayOn;
    bool _isMotionDetected;
    uint8_t _displayPage;
    uint8_t _displayPageMax = 3;
    uint8_t _displayPageMin = 0;
};