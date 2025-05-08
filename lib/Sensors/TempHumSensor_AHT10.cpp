#include <TempHumSensor_AHT10.h>
#include <Adafruit_AHTX0.h>
#include <Debug.h>

/**
 * @brief AHT10 temperature and humidity sensor
 */
TempHumSensor_AHT10::TempHumSensor_AHT10()
{
    _sensor = new Adafruit_AHTX0();
    _lastUpdateTime = 0;
    _data.temperature = 0;
    _data.humidity = 0;
    _prevData = _data;
}

/**
 * @brief Initialize the AHT10 sensor
 */
void TempHumSensor_AHT10::begin()
{
    if (!_sensor->begin()) {
        DEBUG_PRINTLN("AHT10 not found");
        return;
    }
    _lastUpdateTime = millis();
}

/**
 * @brief Get the current temperature and humidity data
 * @return Pointer to the current sensor data
 */
SensorData *TempHumSensor_AHT10::getData()
{
    sensors_event_t humidity, temperature;
    if (_sensor->getEvent(&humidity, &temperature)) {
        _prevData = _data;
        _data.temperature = temperature.temperature;
        _data.humidity = humidity.relative_humidity;
        _lastUpdateTime = millis();
        return &_data;
    }
    return nullptr;
}

/**
 * @brief Get the previous temperature and humidity data
 * @return Pointer to the previous sensor data
 */
SensorData *TempHumSensor_AHT10::getPrevData()
{
    return &_prevData;
}
