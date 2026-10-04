#include <MonitorModel.h>
#include <MeasurementGuidance.h>
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
using namespace monitor;

int main() {
    // Exact boundaries and both sides are intentional; sources do not define
    // symmetric traffic-light bands (e.g. no arbitrary red low-humidity limit).
    struct Case { Metric metric; float value; Guidance expected; };
    const Case cases[] = {
        {Metric::Temperature, 17.99f, Guidance::Attention},
        {Metric::Temperature, 18, Guidance::Reference},
        {Metric::Temperature, 24, Guidance::Reference},
        {Metric::Temperature, 24.01f, Guidance::Watch},
        {Metric::Temperature, 26, Guidance::Watch},
        {Metric::Temperature, 26.01f, Guidance::Attention},
        {Metric::Humidity, 0, Guidance::Watch},
        {Metric::Humidity, 29.99f, Guidance::Watch},
        {Metric::Humidity, 30, Guidance::Reference},
        {Metric::Humidity, 50, Guidance::Reference},
        {Metric::Humidity, 50.01f, Guidance::Watch},
        {Metric::Humidity, 59.99f, Guidance::Watch},
        {Metric::Humidity, 60, Guidance::Attention},
        {Metric::Humidity, 100, Guidance::Attention},
        {Metric::Humidity, 101, Guidance::Unavailable},
        {Metric::CO2, -1, Guidance::Unavailable},
        {Metric::CO2, 1000, Guidance::Reference},
        {Metric::CO2, 1001, Guidance::Watch},
        {Metric::CO2, 1500, Guidance::Watch},
        {Metric::CO2, 1501, Guidance::Attention},
        {Metric::Pressure, 600, Guidance::Neutral},
        {Metric::Pressure, 760, Guidance::Neutral},
        {Metric::Pressure, 825, Guidance::Neutral},
        {Metric::Light, 0, Guidance::Neutral}
    };
    for (const auto &c : cases) assert(measurementGuidance(c.metric, c.value) == c.expected);
    for (Metric m : {Metric::Temperature, Metric::Humidity, Metric::CO2, Metric::Light, Metric::Pressure}) {
        assert(measurementGuidance(m, std::numeric_limits<float>::quiet_NaN()) == Guidance::Unavailable);
        assert(measurementGuidance(m, std::numeric_limits<float>::infinity()) == Guidance::Unavailable);
    }
    Readings r;
    r.temperature = -12.34f; r.humidity = 56.78f;
    r.co2 = 1234; r.light = 4321; r.valid = Environment | CO2 | Light;
    r.pressure = 1013.25f;
    auto p = HistoryPoint::from(60, r);
    float value = 0;
    assert(p.value(Metric::Temperature, value) && std::fabs(value + 12.34f) < 0.01f);
    assert(p.value(Metric::Humidity, value) && std::fabs(value - 56.78f) < 0.01f);
    assert(p.value(Metric::CO2, value) && value == 1234);
    assert(p.value(Metric::Light, value) && value == 4321);
    assert(p.value(Metric::Pressure, value) && std::fabs(value - 760.0f) < 0.05f);
    for (float pressure : {300.0f, 1100.0f}) {
        r.pressure = pressure;
        assert(HistoryPoint::from(60, r).value(Metric::Pressure, value));
        assert(std::fabs(value - pressure * HpaToMmHg) < 0.05f);
    }
    for (float pressure : {0.0f, 299.0f, 1101.0f, std::numeric_limits<float>::quiet_NaN()}) {
        r.pressure = pressure;
        assert(!HistoryPoint::from(60, r).value(Metric::Pressure, value));
    }
    r.temperature = std::numeric_limits<float>::quiet_NaN();
    r.co2 = -1; r.light = -1;
    assert(HistoryPoint::from(120, r).valid == 0);
    History history;
    assert(history.size() == 0);
    for (size_t i = 0; i < HistoryCapacity + 20; ++i) {
        p.seconds = static_cast<uint32_t>(i * 60);
        history.append(p);
    }
    assert(history.size() == HistoryCapacity);
    assert(history.at(0).seconds == 20 * 60);
    assert(history.at(HistoryCapacity - 1).seconds - history.at(0).seconds == HistorySeconds);
    Navigation nav;
    assert(!nav.needsRender());
    nav.move(-1);
    assert(nav.page() == Page::Service && !nav.needsRender());
    nav.setOn(true);
    assert(nav.needsRender());
    nav.rendered();
    assert(!nav.needsRender());
    nav.move(1);
    assert(nav.page() == Page::Overview && nav.needsRender());
    nav.rendered();
    nav.setOn(false); nav.invalidate();
    assert(!nav.needsRender());
    nav.setOn(true);
    assert(nav.needsRender());
    nav.move(1000000);
    assert(nav.page() == Page::Temperature);
    Uptime clock;
    clock.tick(0xfffffff0U);
    const auto before = clock.seconds();
    clock.tick(2000);
    assert(clock.seconds() >= before + 2);
    DisplayActivity activity;
    assert(activity.update(0, false, false));
    assert(!activity.update(30000, false, false));
    assert(activity.update(40000, false, true));
    assert(activity.update(80000, true, false));
    assert(activity.update(109999, false, false));
    assert(!activity.update(110000, false, false));
    assert(activity.update(0xfffffff0U, false, true));
    assert(activity.update(1000, false, false));
    assert(!activity.update(40000, false, false));
    assert(!activity.update(0, false, false)); // no spontaneous wake on subsequent wrap
    SampleSchedule schedule(100);
    assert(schedule.fanNeeded(100) && !schedule.due(100));
    assert(!schedule.due(15099) && schedule.due(15100));
    schedule.sampled(15100);
    assert(!schedule.fanNeeded(60099) && schedule.fanNeeded(60100));
    assert(!schedule.due(75099) && schedule.due(75100));
    schedule.sampled(0xfffffff0U);
    assert(!schedule.due(1000) && schedule.due(60000));
    std::printf("PASS: guidance boundaries, history encoding, ring wrap, navigation, display gating, uptime wrap (%zu bytes history)\n", sizeof(History));
}
