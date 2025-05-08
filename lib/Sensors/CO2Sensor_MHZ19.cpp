#include "CO2Sensor_MHZ19.h"
#include <Arduino.h>
#include <MHZ19.h>
#include <Debug.h>

/**
 * @brief CO2Sensor_MHZ19 constructor
 */
CO2Sensor_MHZ19::CO2Sensor_MHZ19(int txPin, int rxPin)
{
    _txPin = txPin;
    _rxPin = rxPin;
    _lastUpdateTime = 0;
    _sensor = new MHZ19();

}

/**
 * @brief MZH19 CO2 sensor initialization
 */
void CO2Sensor_MHZ19::begin()
{
    Serial2.begin(9600, SERIAL_8N1, _rxPin, _txPin);
    _sensor->begin(Serial2);
    _sensor->autoCalibration(true);
    DEBUG_PRINTLN("CO2 Sensor initialized");
}

/**
 * @brief Retrive and return the CO2 sensor data
 * 
 * @return SensorData* Pointer to the CO2Data structure containing the sensor data
 */
SensorData* CO2Sensor_MHZ19::getData()
{
    _lastUpdateTime = millis();
    _prevData = _data;
    _data.ppm = _sensor->getCO2();
    _data.temperature = _sensor->getTemperature();
    return &_data;
}

/**
 * @brief Retrieve the previous CO2 sensor data
 * 
 * @return SensorData* Pointer to the previous CO2Data structure containing the sensor data
 */
SensorData* CO2Sensor_MHZ19::getPrevData()
{
    return &_prevData;
}