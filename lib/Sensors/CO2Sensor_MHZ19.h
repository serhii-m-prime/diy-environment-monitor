#pragma

#include "IDataSensor.h"
#include <MHZ19.h>

class CO2Sensor_MHZ19: public IDataSensor
{
public:
    CO2Sensor_MHZ19(int txPin, int rxPin);
    void begin() override;
    SensorData *getData() override;
    SensorData *getPrevData() override;
private:
    int _txPin;
    int _rxPin;
    MHZ19 *_sensor;
    CO2Data _data;
    CO2Data _prevData;
};

