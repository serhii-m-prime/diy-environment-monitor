#pragma once

#include "IDataSensor.h"
#include <Adafruit_BME280.h>
#include <SensorData.h>

class TempHumPressSensor_BME280 : public IDataSensor
{
public:
    TempHumPressSensor_BME280(int address);
    void begin() override;
    SensorData *getData() override;
    SensorData *getPrevData() override;

private:
    int _address;
    Adafruit_BME280 *_sensor;
    TempHumPressureData _data;
    TempHumPressureData _prevData;
};
