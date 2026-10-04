#include <LightSensor_BH1750.h>
#include <BH1750.h>
#include <Debug.h>
#include <cmath>

/**
 * @brief BH1750 light sensor
 */
LightSensor_BH1750::LightSensor_BH1750()
{
    _sensor = new BH1750();
    _data.lux = 0;
    _prevData = _data;
}

/**
 * @brief Initialize the BH1750 sensor
 */
void LightSensor_BH1750::begin()
{
    if (!_sensor->begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        DEBUG_PRINTLN("BH1750 not found");
        return;
    }
}

/**
 * @brief Retrieve and return the BH1750 sensor data
 *
 * @return SensorData* Pointer to the LightData structure containing the sensor data
 */
SensorData *LightSensor_BH1750::getData()
{
    if (_sensor->measurementReady()) {
        const float lux = _sensor->readLightLevel();
        if (!std::isfinite(lux) || lux < 0.0f) {
            return nullptr;
        }
        _prevData = _data;
        _data.lux = lux;
        return &_data;
    }
    return nullptr;
}

/**
 * @brief Retrieve the previous BH1750 sensor data
 *
 * @return SensorData* Pointer to the previous LightData structure containing the sensor data
 */
SensorData *LightSensor_BH1750::getPrevData()
{
    return &_prevData;
}
