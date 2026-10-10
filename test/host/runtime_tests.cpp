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
    sample.readings.pressure = 1013; sample.readings.co2 = 800; sample.readings.light = 100;
    sample.lightSequence = 1;
    sample.historySequence = 1;
    sample.historyPoint = HistoryPoint::from(1, sample.readings);
    xQueueOverwrite(sensorMailbox, &sample);
    testTime = 1000; loop();
    assert(display.on && history.size() == 1 && display.draws > 0 && mockBacklight == 0);
    const auto initialDraws = display.draws;
    while (testTime < 9000) loop();
    assert(display.draws == initialDraws && mockBacklight == 400); // PWM fade does not redraw pixels
    sample.readings.light = 0;
    ++sample.lightSequence;
    xQueueOverwrite(sensorMailbox, &sample);
    testTime = 10000; loop();
    // Repeated fan/status snapshots must not turn one shadow sample into three.
    for (unsigned t : {11000U, 12000U}) {
        xQueueOverwrite(sensorMailbox, &sample);
        testTime = t; loop();
        assert(backlight.target() == 400 && mockBacklight == 400);
    }
    sample.readings.light = 100;
    ++sample.lightSequence;
    xQueueOverwrite(sensorMailbox, &sample);
    testTime = 13000; loop();
    assert(backlight.target() == 400);
    sample.readings.light = 200;
    for (unsigned t : {14000U, 15000U, 16000U}) {
        ++sample.lightSequence;
        xQueueOverwrite(sensorMailbox, &sample);
        testTime = t; loop();
        assert(mockBacklight < 800);
    }
    assert(backlight.target() == 800);
    const auto settledDraws = display.draws;
    while (testTime < 29000) loop();
    assert(history.size() == 1 && mockBacklight == 800 && display.draws == settledDraws);
    testTime = 31000; loop();
    assert(!display.on && mockBacklight == 0);
    const auto offDraws = display.draws;
    testTime = 61000;
    sample.historySequence = 2;
    sample.historyPoint = HistoryPoint::from(61, sample.readings);
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(history.size() == 2 && display.draws == offDraws && mockBacklight == 0);
    float pressure = 0;
    assert(history.at(1).value(Metric::Pressure, pressure) && std::fabs(pressure - 759.8125f) < 0.05f);
    InputSnapshot input;
    input.rightPresses = 1;
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(display.on && navigation.page() == Page::Temperature && display.draws > offDraws);
    assert(mockBacklight == 0); // wake does not jump to retained target
    const auto graphDraws = display.draws;
    while (testTime < 68000) loop();
    assert(mockBacklight == 800 && display.draws == graphDraws);
    sample.readings.light = 50;
    ++sample.lightSequence;
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(display.draws == graphDraws && mockBacklight == 800); // transient and hidden overview not drawn
    sample.readings.valid &= ~Light; // retain brightness through sensor failure
    ++sample.lightSequence;
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(display.draws == graphDraws && mockBacklight == 800);
    testTime = 100000; loop();
    assert(!display.on);
    input.rightPresses = 5; // four presses accumulated while loop was busy
    input.motion = true;
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(display.on && navigation.page() == Page::Pressure);
    const auto pressureDraws = display.draws;
    testTime = 151000; loop(); // latest stored sample is 61 s: still fresh at 90 s
    assert(display.draws == pressureDraws);
    testTime = 152000; loop();
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
        if (xQueueReceive(sensorMailbox, &snapshot, 0) == pdTRUE && snapshot.historySequence == 2) {
            assert(snapshot.historyPoint.seconds == 77 && !snapshot.fanOn);
            throw WorkerFinished{};
        }
        assert(testTime < 78000);
        testTime += ticks;
    };
    try { sensorTask(nullptr); } catch (const WorkerFinished &) {}
    assert(measurements == 2);
    Adafruit_BME280::beforeMeasurement = nullptr;
    mockTaskDelay = nullptr;
    std::puts("PASS: loop render gating, pressure expiry, brightness, and actual worker waits 2 s after fan stop before sampling");
}
