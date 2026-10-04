#include <TempHumPressSensor_BME280.h>
#include <Adafruit_BME280.h>
#include <Debug.h>
#include <cmath>

TempHumPressSensor_BME280::TempHumPressSensor_BME280(int address)
{
    _address = address;
    _sensor = new Adafruit_BME280();
    _data.temperature = 0;
    _data.humidity = 0;
    _data.pressure = 0;
    _prevData = _data;
}

/**
 * @brief BME280 sensor initialization
 *
 */
void TempHumPressSensor_BME280::begin()
{
    _initialized = _sensor->begin(_address);
    if (!_initialized)
    {
        DEBUG_PRINTLN("Could not find a valid BME280 sensor, check wiring!");
        return;
    }
    _sensor->setSampling(
        Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1, // Temp oversampling
        Adafruit_BME280::SAMPLING_X1, // Humidity oversampling
        Adafruit_BME280::SAMPLING_X1, // Pressure oversampling
        Adafruit_BME280::FILTER_OFF   // IIR filter off
    );
    DEBUG_PRINTLN("BME280 Sensor initialized");
}

/**
 * @brief Retrieve and return the BME280 sensor data
 *
 * @return SensorData* Pointer to the TempHumPressureData structure containing the sensor data
 */
SensorData *TempHumPressSensor_BME280::getData()
{
    if (!_initialized || !_sensor->takeForcedMeasurement()) return nullptr;
    const float temperature = _sensor->readTemperature();
    const float humidity = _sensor->readHumidity();
    const float pressure = _sensor->readPressure() / 100.0F;
    if (!std::isfinite(temperature) || temperature < -40 || temperature > 85 ||
        !std::isfinite(humidity) || humidity < 0 || humidity > 100 ||
        !std::isfinite(pressure) || pressure < 300 || pressure > 1100) return nullptr;
    _prevData = _data;
    _data.temperature = temperature;
    _data.humidity = humidity;
    _data.pressure = pressure;
    return &_data;
}

/**
 * @brief Retrieve the previous BME280 sensor data
 *
 * @return SensorData* Pointer to the previous TempHumPressureData structure containing the sensor data
 */
SensorData *TempHumPressSensor_BME280::getPrevData()
{
    return &_prevData;
}
