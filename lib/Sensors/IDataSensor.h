#pragma once

#include <SensorData.h>

class IDataSensor
{
public:
    virtual void begin() = 0;
    virtual SensorData *getData() = 0;
    virtual SensorData *getPrevData() = 0;
    virtual unsigned long getLastUpdateTime() = 0;
    virtual ~IDataSensor() = default;
};