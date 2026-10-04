// Run the actual main loop with mailboxes and display transport mocked.
// Workers are not scheduled here; their outputs are injected into the real queues.
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
    sample.lightSeen = true;
    sample.historySequence = 1;
    sample.historyPoint = HistoryPoint::from(1, sample.readings);
    xQueueOverwrite(sensorMailbox, &sample);
    testTime = 1000; loop();
    assert(display.on && history.size() == 1 && display.draws > 0 && mockBacklight == 400);
    const auto initialDraws = display.draws;
    loop();
    assert(display.draws == initialDraws); // idle loop does not render
    sample.readings.light = 200;
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(history.size() == 1 && mockBacklight == 800 && display.draws > initialDraws);
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
    const auto graphDraws = display.draws;
    sample.readings.light = 50;
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(display.draws == graphDraws && mockBacklight == 200); // hidden overview not drawn
    sample.readings.valid &= ~Light; // retain brightness through sensor failure
    xQueueOverwrite(sensorMailbox, &sample);
    loop();
    assert(display.draws == graphDraws && mockBacklight == 200);
    testTime = 100000; loop();
    assert(!display.on);
    input.rightPresses = 5; // four presses accumulated while loop was busy
    xQueueOverwrite(inputMailbox, &input);
    loop();
    assert(display.on && navigation.page() == Page::Pressure);
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
    std::puts("PASS: actual loop collects while off, wakes/navigates, retains brightness and draws only selected page");
}
