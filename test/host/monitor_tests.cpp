#include <MonitorModel.h>
#include <cassert>
#include <cstdio>
#include <limits>
using namespace monitor;

int main() {
    Readings r;
    r.temperature = -12.34f; r.humidity = 56.78f;
    r.co2 = 1234; r.light = 4321; r.valid = Environment | CO2 | Light;
    auto p = HistoryPoint::from(60, r);
    float value = 0;
    assert(p.value(Metric::Temperature, value) && std::fabs(value + 12.34f) < 0.01f);
    assert(p.value(Metric::Humidity, value) && std::fabs(value - 56.78f) < 0.01f);
    assert(p.value(Metric::CO2, value) && value == 1234);
    assert(p.value(Metric::Light, value) && value == 4321);
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
    assert(nav.page() == Page::Light);
    Uptime clock;
    clock.tick(0xfffffff0U);
    const auto before = clock.seconds();
    clock.tick(2000);
    assert(clock.seconds() >= before + 2);
    std::printf("PASS: history encoding, ring wrap, navigation, display gating, uptime wrap (%zu bytes history)\n", sizeof(History));
}
