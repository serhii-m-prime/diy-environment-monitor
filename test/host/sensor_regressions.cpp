#include <PIRMotionSensor.h>
#include <CO2Sensor_MHZ19.h>
#include <LightSensor_BH1750.h>
#include <State.h>
#include <GPIOButton.h>
#include <TempHumPressSensor_BME280.h>
#include <cassert>
#include <limits>

unsigned long testTime = 0;
int testPin = LOW;
FakeSerial Serial2;
int MHZ19::co2Result = RESULT_OK;
int MHZ19::temperatureResult = RESULT_OK;
int MHZ19::ppm = 800;
int MHZ19::temperatureCalls = 0;
bool BH1750::ready = true;
bool BH1750::configured = true;
float BH1750::lux = 120.0f;
bool Adafruit_BME280::connected = false;
bool Adafruit_BME280::measurementOk = true;
float Adafruit_BME280::temperature = 22.5f;
float Adafruit_BME280::humidity = 40.0f;
float Adafruit_BME280::pressure = 101325.0f;

void testButtons() {
    GPIOButton button(7);
    int presses = 0, starts = 0, ends = 0;
    button.onEvent([&] { ++presses; });
    button.onEventStart([&] { ++starts; });
    button.onEventEnd([&] { ++ends; });
    button.begin();
    auto step = [&](unsigned long t, int pin) {
        testTime = t; testPin = pin; button.update();
    };
    step(0, HIGH); step(100, LOW); step(110, HIGH); step(180, HIGH);
    assert(presses == 0 && starts == 0 && ends == 0);
    step(200, LOW); step(260, LOW); step(300, LOW);
    assert(presses == 1 && starts == 1 && ends == 0);
    step(310, HIGH); step(320, LOW); step(380, LOW);
    assert(presses == 1 && ends == 0);
    step(400, HIGH); step(460, HIGH); step(500, LOW); step(560, LOW);
    assert(presses == 2 && starts == 2 && ends == 1);
}

void testEnvironment() {
    TempHumPressSensor_BME280 sensor(0x76);
    sensor.begin(); // Missing sensor must return instead of hanging the worker.
    assert(sensor.getData() == nullptr);
    Adafruit_BME280::connected = true;
    sensor.begin();
    auto good = static_cast<TempHumPressureData*>(sensor.getData());
    assert(good && good->temperature == 22.5f && good->pressure == 1013.25f);
    Adafruit_BME280::measurementOk = false;
    assert(sensor.getData() == nullptr && good->temperature == 22.5f);
    Adafruit_BME280::measurementOk = true;
    Adafruit_BME280::temperature = std::numeric_limits<float>::quiet_NaN();
    assert(sensor.getData() == nullptr);
    Adafruit_BME280::temperature = 24;
    assert(sensor.getData() && good->temperature == 24);
}

void testPIR() {
    PIRMotionSensor sensor(4);
    int starts = 0, confirmed = 0, ends = 0;
    bool displayOn = false;
    sensor.setDebounceDelay(20);
    sensor.onEventStart([&] { ++starts; });
    sensor.onEvent([&] { ++confirmed; displayOn = true; });
    sensor.onEventEnd([&] { ++ends; displayOn = false; });
    auto step = [&](unsigned long t, int pin) {
        testTime = t; testPin = pin; sensor.update();
    };
    step(0, LOW);
    // A short HIGH pulse must not emit either edge callback.
    step(100, HIGH); step(110, LOW); step(140, LOW);
    assert(starts == 0 && confirmed == 0 && ends == 0);
    step(200, HIGH); step(230, HIGH);
    assert(displayOn && starts == 1 && confirmed == 1);
    // Regression: a short LOW pulse used to turn the screen off indefinitely.
    step(240, LOW); step(250, HIGH); step(280, HIGH); step(1000, HIGH);
    assert(displayOn && confirmed == 1 && ends == 0);
    step(1100, LOW); step(1130, LOW); step(1200, LOW);
    assert(!displayOn && ends == 1);
    step(1300, HIGH); step(1330, HIGH); step(1400, HIGH);
    assert(displayOn && starts == 2 && confirmed == 2 && ends == 1);
}

void testCO2() {
    CO2Sensor_MHZ19 sensor(18, 17);
    sensor.begin();
    State state;
    auto sample = [&] {
        auto data = static_cast<CO2Data*>(sensor.getData());
        if (data) state.fill(data); else state.invalidateCO2();
        return data;
    };
    assert(!state.isCO2Valid() && !state.isCO2Actual(60000));
    auto good = sample();
    assert(good && state.getCO2() == 800 && state.isCO2Valid());
    for (int error : {RESULT_TIMEOUT, RESULT_CRC}) {
        MHZ19::co2Result = error;
        int calls = MHZ19::temperatureCalls;
        assert(sample() == nullptr);
        assert(MHZ19::temperatureCalls == calls);
        assert(!state.isCO2Valid() && !state.isCO2Actual(60000));
        assert(state.getCO2() == 800 && good->ppm == 800);
        assert(state.toString().find("CO2: N/A") != String::npos);
    }
    MHZ19::co2Result = RESULT_OK;
    MHZ19::ppm = 950;
    MHZ19::temperatureResult = RESULT_TIMEOUT;
    assert(sample() == nullptr && good->ppm == 800);
    MHZ19::temperatureResult = RESULT_OK;
    assert(sample() && state.isCO2Valid() && state.getCO2() == 950);
    assert(static_cast<CO2Data*>(sensor.getPrevData())->ppm == 800);
    assert(state.toString().find("CO2: 950 ppm") != String::npos);
}

void testLight() {
    LightSensor_BH1750 sensor;
    sensor.begin();
    State state;
    float brightnessInput = 0;
    auto sample = [&] {
        auto data = static_cast<LightData*>(sensor.getData());
        if (data) {
            state.fill(data);
            brightnessInput = state.getLightLevel();
        } else state.invalidateLightLevel();
        return data;
    };
    assert(!state.isLightLevelValid() && !state.isLightLevelActual(60000));
    auto good = sample();
    assert(good && brightnessInput == 120.0f);
    for (float invalid : {-1.0f, -2.0f, std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity()}) {
        BH1750::lux = invalid;
        assert(sample() == nullptr);
        assert(!state.isLightLevelValid() && !state.isLightLevelActual(60000));
        assert(brightnessInput == 120.0f && good->lux == 120.0f);
        // PIR wakeup reads the retained value, even while the sample is invalid.
        assert(state.getLightLevel() == 120.0f);
        assert(state.toString().find("Light Level: N/A") != String::npos);
    }
    BH1750::ready = false;
    assert(sample() == nullptr);
    BH1750::ready = true;
    BH1750::lux = 0.0f;
    assert(sample() && state.isLightLevelValid() && brightnessInput == 0.0f);
    assert(static_cast<LightData*>(sensor.getPrevData())->lux == 120.0f);
    BH1750::configured = false;
    LightSensor_BH1750 missing;
    missing.begin();
    assert(missing.getData() == nullptr);
}

int main() {
    testButtons();
    testEnvironment();
    testPIR();
    testCO2();
    testLight();
    std::puts("PASS: buttons/PIR debounce, BME startup/failure/recovery, CO2 and light errors/recovery");
}
