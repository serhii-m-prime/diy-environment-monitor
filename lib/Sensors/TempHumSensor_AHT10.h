#pragma once

#include <IDataSensor.h>
#include <SensorData.h>
#include <Adafruit_AHTX0.h>

class TempHumSensor_AHT10 : public IDataSensor
{
public:
    TempHumSensor_AHT10();
    void begin() override;
    SensorData *getData() override;
    SensorData *getPrevData() override;
private:
    int _address;
    Adafruit_AHTX0 *_sensor;
    TempHumData _data;
    TempHumData _prevData;
};