#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ST7796S.h>
#include <Debug.h>
#include <GPIOButton.h>
#include <PIRMotionSensor.h>
#include <CO2Sensor_MHZ19.h>
#include <TempHumPressSensor_BME280.h>
#include <LightSensor_BH1750.h>
#include <MonitorUi.h>
#include <DisplayBrightness.h>
#include <HistoryAverage.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

using namespace monitor;

namespace {
constexpr int LeftPin = 7, RightPin = 14, PirPin = 4;
constexpr int SdaPin = 41, SclPin = 40, EnvironmentAddress = 0x76;
constexpr int Co2TxPin = 18, Co2RxPin = 17;
constexpr int TftCs = 10, TftDc = 9, TftReset = 15, TftMosi = 11, TftClock = 12;
constexpr int BacklightPin = 5, BacklightChannel = 0;
constexpr int FanPin = 6, FanChannel = 1;
constexpr uint32_t SpiFrequency = 80000000;
constexpr uint32_t LightPeriodMs = 1000, LightRetryMs = 30000;

SPIClass displaySpi(HSPI);
Adafruit_ST7796S display(&displaySpi, TftCs, TftDc, TftReset);
MonitorUi<Adafruit_ST7796S> ui(display);
Navigation navigation;
History history;
HistoryAverage historyAverage;
Uptime uptime;
DisplayActivity activity;
DisplayBrightness backlight;
Diagnostics diagnostics;
InputSnapshot inputs;
SensorSnapshot sensors;
QueueHandle_t inputMailbox = nullptr, sensorMailbox = nullptr;

// No callbacks in this task touch the display or shared application state.
void inputTask(void *) {
    GPIOButton left(LeftPin), right(RightPin);
    PIRMotionSensor pir(PirPin);
    InputSnapshot snapshot;
    left.onEvent([&] { ++snapshot.leftPresses; });
    right.onEvent([&] { ++snapshot.rightPresses; });
    pir.onEvent([&] { snapshot.motion = true; });
    pir.onEventEnd([&] { snapshot.motion = false; });
    left.begin(); right.begin(); pir.begin();
    pir.setDebounceDelay(20);
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        left.update(); right.update(); pir.update();
        // Counters survive an overwritten snapshot while TFT is busy.
        xQueueOverwrite(inputMailbox, &snapshot);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}

// Sole owner of Wire, Serial2 and sensor objects. Slow I/O cannot block buttons/UI.
void sensorTask(void *) {
    CO2Sensor_MHZ19 co2(Co2TxPin, Co2RxPin);
    TempHumPressSensor_BME280 environment(EnvironmentAddress);
    LightSensor_BH1750 light;
    SensorSnapshot snapshot;
    Wire.begin(SdaPin, SclPin);
    Wire.setTimeOut(50);
    co2.begin(); environment.begin(); light.begin();
    Uptime sensorUptime;
    SampleSchedule schedule(millis());
    uint32_t lastLight = millis() - LightPeriodMs;
    uint32_t lastLightRetry = millis();
    bool environmentHealthy = true;

    for (;;) {
        const uint32_t now = millis();
        sensorUptime.tick(now);
        bool publish = false;
        schedule.update(now);
        const bool fan = schedule.fanNeeded();
        if (fan != snapshot.fanOn) {
            snapshot.fanOn = fan;
            ledcWrite(FanChannel, fan ? 142 : 0);
            publish = true;
        }
        const bool sampleDue = schedule.due(now);
        if (sampleDue) {
            const uint32_t started = micros();
            if (!environmentHealthy) environment.begin();
            snapshot.readings.valid &= ~(Environment | CO2);
            if (auto data = static_cast<CO2Data *>(co2.getData())) {
                snapshot.readings.co2 = data->ppm;
                snapshot.readings.valid |= CO2;
            } else ++snapshot.errors;
            if (auto data = static_cast<TempHumPressureData *>(environment.getData())) {
                snapshot.readings.temperature = data->temperature;
                snapshot.readings.humidity = data->humidity;
                snapshot.readings.pressure = data->pressure;
                snapshot.readings.valid |= Environment;
                environmentHealthy = true;
            } else {
                environmentHealthy = false;
                ++snapshot.errors;
            }
            snapshot.measurementUs = micros() - started;
        }
        if (sampleDue || uint32_t(now - lastLight) >= LightPeriodMs) {
            lastLight = now;
            ++snapshot.lightSequence;
            if (!(snapshot.readings.valid & Light) && uint32_t(now - lastLightRetry) >= LightRetryMs) {
                light.begin();
                lastLightRetry = now;
            }
            snapshot.readings.valid &= ~Light;
            if (auto data = static_cast<LightData *>(light.getData())) {
                snapshot.readings.light = data->lux;
                snapshot.readings.valid |= Light;
            } else ++snapshot.errors;
            publish = true;
        }
        if (sampleDue) {
            sensorUptime.tick(millis());
            snapshot.samplePoint = HistoryPoint::from(sensorUptime.seconds(), snapshot.readings);
            ++snapshot.sampleSequence;
            schedule.sampled(now);
            snapshot.fanOn = false;
            ledcWrite(FanChannel, 0);
            publish = true;
        }
        if (publish) xQueueOverwrite(sensorMailbox, &snapshot);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

} // namespace

void setup() {
    DEBUG_BEGIN(115200);
    ledcSetup(BacklightChannel, 5000, 12);
    ledcAttachPin(BacklightPin, BacklightChannel);
    ledcWrite(BacklightChannel, 0);
    ledcSetup(FanChannel, 5000, 8);
    ledcAttachPin(FanPin, FanChannel);
    ledcWrite(FanChannel, 0);
    displaySpi.begin(TftClock, -1, TftMosi, TftCs);
    display.init(320, 480, 0, 0, ST7796S_BGR);
    display.setSPISpeed(SpiFrequency);
    display.setRotation(3);
    display.sendCommand(ST77XX_INVOFF);
    display.sendCommand(ST77XX_DISPOFF);

    inputMailbox = xQueueCreate(1, sizeof(InputSnapshot));
    sensorMailbox = xQueueCreate(1, sizeof(SensorSnapshot));
    if (!inputMailbox || !sensorMailbox) abort();
    // ESP-IDF task stack sizes are bytes. No task allocates a framebuffer.
    if (xTaskCreatePinnedToCore(inputTask, "buttons", 3072, nullptr, 2, nullptr, 0) != pdPASS) abort();
    if (xTaskCreatePinnedToCore(sensorTask, "sensors", 4096, nullptr, 1, nullptr, 0) != pdPASS) abort();
}

void loop() {
    static uint32_t lastLoopUs = micros(), lastServiceMs = 0;
    const uint32_t loopStart = micros();
    diagnostics.loopUs = loopStart - lastLoopUs; // complete cycle, including render and delay
    diagnostics.maxLoopUs = std::max(diagnostics.maxLoopUs, diagnostics.loopUs);
    lastLoopUs = loopStart;
    const uint32_t now = millis();
    uptime.tick(now);

    bool pressed = false;
    InputSnapshot nextInput;
    if (xQueueReceive(inputMailbox, &nextInput, 0) == pdTRUE) {
        const uint32_t left = nextInput.leftPresses - inputs.leftPresses;
        const uint32_t right = nextInput.rightPresses - inputs.rightPresses;
        pressed = left || right;
        constexpr uint32_t pageCount = static_cast<uint32_t>(Page::Count);
        if (pressed) navigation.move(static_cast<int>(right % pageCount) - static_cast<int>(left % pageCount));
        inputs = nextInput;
    }
    SensorSnapshot nextSensor;
    if (xQueueReceive(sensorMailbox, &nextSensor, 0) == pdTRUE) {
        if (nextSensor.lightSequence != sensors.lightSequence)
            backlight.observe(nextSensor.readings.light, nextSensor.readings.valid & Light, now);
        if (nextSensor.sampleSequence != sensors.sampleSequence) {
            HistoryPoint averaged;
            if (historyAverage.add(nextSensor.samplePoint, averaged)) history.append(averaged);
            // Live readings stay fresh every minute; history advances every third sample.
            navigation.invalidate();
        }
        if (navigation.page() == Page::Overview ||
            (navigation.page() == Page::Light && ((nextSensor.readings.valid ^ sensors.readings.valid) & Light)))
            navigation.invalidate();
        sensors = nextSensor;
    }

    const bool on = activity.update(now, inputs.motion, pressed);
    if (on != navigation.isOn()) {
        if (!on) ledcWrite(BacklightChannel, 0);
        display.sendCommand(on ? ST77XX_DISPON : ST77XX_DISPOFF);
        navigation.setOn(on);
    }
    if (navigation.isOn() && navigation.page() == Page::Service && uint32_t(now - lastServiceMs) >= 1000) {
        navigation.invalidate();
        lastServiceMs = now;
    }
    // A worker can publish across a second boundary after this loop's first tick.
    uptime.tick(millis());
    // Expire the pressure tendency once if the worker stops publishing. Do not
    // redraw the graph on every idle loop or render a hidden/sleeping page.
    static bool pressureFreshPreviously = false;
    const bool pressureFresh = history.size() &&
        uint32_t(uptime.seconds() - history.at(history.size() - 1).seconds) <= PressureTrendToleranceSeconds;
    if (pressureFresh != pressureFreshPreviously && navigation.page() == Page::Pressure)
        navigation.invalidate();
    pressureFreshPreviously = pressureFresh;
    diagnostics.motion = inputs.motion;
    diagnostics.fanOn = sensors.fanOn;
    diagnostics.brightness = backlight.update(millis(), navigation.isOn());
    diagnostics.sensorUs = sensors.measurementUs;
    diagnostics.sensorErrors = sensors.errors;
    diagnostics.uptimeSeconds = uptime.seconds();
    diagnostics.sampleAgeSeconds = sensors.sampleSequence ? uptime.seconds() - sensors.samplePoint.seconds : 0;
    if (navigation.needsRender()) {
        diagnostics.freeHeap = ESP.getFreeHeap();
        diagnostics.minHeap = ESP.getMinFreeHeap();
        const uint32_t renderStart = micros();
        ui.render(navigation, sensors.readings, history, uptime.seconds(), diagnostics);
        diagnostics.renderUs = micros() - renderStart;
        diagnostics.maxRenderUs = std::max(diagnostics.maxRenderUs, diagnostics.renderUs);
        ++diagnostics.renders;
    }
    static uint16_t previousBrightness = 0;
    if (diagnostics.brightness != previousBrightness) {
        ledcWrite(BacklightChannel, diagnostics.brightness);
        previousBrightness = diagnostics.brightness;
    }
    delay(10);
}
