// Run the actual main loop with mailboxes and display transport mocked.
// Main-loop tests inject snapshots. A bounded worker run also verifies fan/read timing.
#include "../../src/main.cpp"
#include <cassert>
#include <cstdio>

unsigned long testTime = 0;
int testPin = HIGH;
FakeSerial Serial2;
int MHZ19::co2Result = RESULT_OK, MHZ19::temperatureResult = RESULT_OK;
int MHZ19::ppm = 800, MHZ19::temperatureCalls = 0;
bool BH1750::ready = true, BH1750::configured = true;
float BH1750::lux = 120;
bool Adafruit_BME280::connected = true, Adafruit_BME280::measurementOk = true;
float Adafruit_BME280::temperature = 22.5f, Adafruit_BME280::humidity = 40;
float Adafruit_BME280::pressure = 101325;

int main() {
    setup();
    assert(!display.on && display.draws == 0 && mockBacklight == 0);
    SensorSnapshot sample;
    sample.readings.valid = Environment | CO2 | Light;
    sample.readings.temperature = 22; sample.readings.humidity = 45;
    sample.readings.pressure = 1013; sample.readings.co2 = 800; sample.readings.light = 30;
    sample.lightSequence = 1;
    sample.sampleSequence = 1;
    sample.samplePoint = HistoryPoint::from(1, sample.readings);
    xQueueOverwrite(sensorMailbox, &sample);
    testTime = 1000; loop();
    assert(display.on && history.size() == 0 && display.draws > 0 && mockBacklight == 0);
    const auto initialDraws = display.draws;
    while (testTime < 9000) loop();
    assert(display.draws == initialDraws && mockBacklight == 1920); // 30 lux gives 46.875% of the 12-bit timer
    sample.readings.light = 0;
    ++sample.lightSequence;
    xQueueOverwrite(sensorMailbox, &sample);
    testTime = 10000; loop();
    // Repeated fan/status snapshots must not turn one shadow sample into three.
    for (unsigned t : {11000U, 12000U}) {
        xQueueOverwrite(sensorMailbox, &sample);
        testTime = t; loop();
        assert(backlight.target() == 1920 && mockBacklight == 1920);
    }
    sample.readings.light = 30;
    ++sample.lightSequence;
    xQueueOverwrite(sensorMailbox, &sample);
    testTime = 13000; loop();
    assert(backlight.target() == 1920);
    sample.readings.light = 60;
    for (unsigned t : {14000U, 15000U, 16000U}) {
        ++sample.lightSequence;
        xQueueOverwrite(sensorMailbox, &sample);
        testTime = t; loop();
        assert(mockBacklight < 3840);
    }
    assert(backlight.target() == 3840);
    const auto settledDraws = display.draws;
    while (testTime < 29000) loop();
    assert(history.size() == 0 && mockBacklight == 3840 && display.draws == settledDraws);
    testTime = 31000; loop();
    assert(!display.on && mockBacklight == 0);
    const auto offDraws = display.draws;
    testTime = 61000;
    sample.sampleSequence = 2;
    sample.readings.temperature = 24; sample.readings.co2 = 1000;
    sample.samplePoint = HistoryPoint::from(61, sample.readings);
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(history.size() == 0 && display.draws == offDraws && mockBacklight == 0);
    testTime = 62000; loop();
    assert(diagnostics.sampleAgeSeconds == 1); // live sample age even before the first mean
    testTime = 121000;
    sample.sampleSequence = 3;
    sample.readings.temperature = 26; sample.readings.co2 = 1200;
    sample.samplePoint = HistoryPoint::from(121, sample.readings);
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(history.size() == 1 && display.draws == offDraws && mockBacklight == 0);
    assert(history.at(0).temperature == 2400 && history.at(0).co2 == 1000);
    assert(sensors.readings.temperature == 26 && sensors.readings.co2 == 1200);
    float pressure = 0;
    assert(history.at(0).value(Metric::Pressure, pressure) && std::fabs(pressure - 759.8125f) < 0.05f);
    InputSnapshot input;
    input.rightPresses = 1;
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(display.on && navigation.page() == Page::Temperature && display.draws > offDraws);
    assert(mockBacklight == 0); // wake does not jump to retained target
    const auto graphDraws = display.draws;
    while (testTime < 128000) loop();
    assert(mockBacklight == 3840 && display.draws == graphDraws);
    sample.readings.light = 15;
    ++sample.lightSequence;
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(display.draws == graphDraws && mockBacklight == 3840); // transient and hidden overview not drawn
    sample.readings.valid &= ~Light; // retain brightness through sensor failure
    ++sample.lightSequence;
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(display.draws == graphDraws && mockBacklight == 3840);
    testTime = 160000; loop();
    assert(!display.on);
    input.rightPresses = 5; // four presses accumulated while loop was busy
    input.motion = true;
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(display.on && navigation.page() == Page::Pressure);
    const auto pressureDraws = display.draws;
    testTime = 391000; loop(); // latest mean ends at 121 s: still fresh at 270 s
    assert(display.draws == pressureDraws);
    testTime = 392000; loop();
    assert(display.draws > pressureDraws && display.on);
    const auto expiredDraws = display.draws;
    loop();
    assert(display.draws == expiredDraws); // expiration causes one redraw only
    input.motion = false;
    input.rightPresses = 6;
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(display.on && navigation.page() == Page::Service);
    const auto serviceDraws = display.draws;
    testTime += 500; loop();
    assert(display.draws == serviceDraws);
    testTime += 600; loop();
    assert(display.draws > serviceDraws);
    input.rightPresses = 7;
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(navigation.page() == Page::Overview);
    input.leftPresses = 1;
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(navigation.page() == Page::Service);
    input.rightPresses += static_cast<uint32_t>(Page::Count);
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(navigation.page() == Page::Service); // a complete seven-page turn
    // Replay more than a day through the real loop while the display sleeps.
    // This verifies cadence, ring wrap, no duplicate points and valid 3h trends.
    history = History{};
    historyAverage = HistoryAverage{};
    sensors = SensorSnapshot{};
    testTime = 600000; loop();
    assert(!display.on);
    const auto sleepingDraws = display.draws;
    sample.readings.valid = Environment | CO2 | Light;
    for (unsigned minute = 0; minute < (HistoryCapacity + 5) * 3; ++minute) {
        testTime = 600000 + minute * 60000;
        sample.sampleSequence = minute + 1;
        sample.readings.co2 = 600 + (minute % 3) * 300;
        sample.samplePoint = HistoryPoint::from(testTime / 1000, sample.readings);
        xQueueOverwrite(sensorMailbox, &sample); loop();
        assert(history.size() == std::min(size_t((minute + 1) / 3), HistoryCapacity));
        xQueueOverwrite(sensorMailbox, &sample); loop(); // repeated snapshot is not a new sample
        assert(display.draws == sleepingDraws);
    }
    assert(history.at(0).seconds == 600 + 5 * 180 + 120);
    assert(history.at(HistoryCapacity - 1).seconds - history.at(0).seconds == HistorySeconds);
    assert(history.at(0).co2 == 900 && sensors.readings.co2 == 1200);
    assert(latestPressureTrend(history, uptime.seconds()).valid);
    // Run the real sensor worker for two samples, stopping via the delay hook.
    struct WorkerFinished {};
    testTime = 0;
    unsigned measurements = 0;
    static unsigned *measurementCounter;
    measurementCounter = &measurements;
    Adafruit_BME280::beforeMeasurement = [] {
        assert(mockFan == 0);
        assert(testTime == (*measurementCounter == 0 ? 17000 : 77000));
        ++*measurementCounter;
    };
    mockTaskDelay = [](TickType_t ticks) {
        if (testTime < 15000 || (testTime >= 60000 && testTime < 75000)) assert(mockFan == 142);
        else assert(mockFan == 0);
        SensorSnapshot snapshot;
        if (xQueueReceive(sensorMailbox, &snapshot, 0) == pdTRUE && snapshot.sampleSequence == 2) {
            assert(snapshot.samplePoint.seconds == 77 && !snapshot.fanOn);
            throw WorkerFinished{};
        }
        assert(testTime < 78000);
        testTime += ticks;
    };
    try { sensorTask(nullptr); } catch (const WorkerFinished &) {}
    assert(measurements == 2);
    Adafruit_BME280::beforeMeasurement = nullptr;
    mockTaskDelay = nullptr;
    std::puts("PASS: 24h averaged history in actual loop, live readings, render gating, pressure expiry, brightness and fan settling");
}
