#pragma once

#include <IDataSensor.h>
#include <SensorData.h>
#include <BH1750.h>

class LightSensor_BH1750 : public IDataSensor
{
public:
    LightSensor_BH1750();
    void begin() override;
    SensorData *getData() override;
    SensorData *getPrevData() override;
private:
    int _address;
    BH1750 *_sensor;
    LightData _data;
    LightData _prevData;
};