#include <DisplayBrightness.h>
#include <cassert>
#include <cstdio>
#include <limits>
using namespace monitor;

int main() {
    DisplayBrightness b;
    assert(b.target() == 240 && b.update(0, false) == 0);
    b.observe(100, true, 0);
    assert(b.target() == 400 && b.update(0, true) == 0);
    uint16_t previous = 0;
    for (uint32_t t = 10; t <= 8000; t += 10) {
        const auto value = b.update(t, true);
        assert(value >= previous && value <= 400 && value - previous <= 16);
        previous = value;
    }
    assert(previous == 400);
    // One shadow sample, then recovery: no change at all, not a dim/rebrighten cycle.
    b.observe(0, true, 9000);
    b.observe(100, true, 10000);
    assert(b.target() == 400 && b.update(10000, true) == 400);
    for (uint32_t t = 11000; t <= 15000; t += 1000) b.observe(t % 2000 ? 85 : 115, true, t);
    assert(b.target() == 400); // +/-15% including exact boundaries stays inside hysteresis
    b.observe(10, true, 16000);
    b.observe(10, true, 17000);
    assert(b.target() == 400);
    b.observe(10, true, 18000);
    assert(b.target() == 40);
    previous = b.update(18000, true);
    assert(previous > 380); // even a long gap cannot jump straight to the target
    for (uint32_t t = 18010; t <= 30000; t += 10) {
        const auto value = b.update(t, true);
        assert(value <= previous && value >= 40 && previous - value <= 4);
        previous = value;
    }
    assert(previous == 40);
    // Invalid/nonfinite samples, missed samples and reversals restart confirmation.
    b.observe(1000, true, 31000);
    b.observe(1000, false, 32000);
    b.observe(1000, true, 33000);
    b.observe(std::numeric_limits<float>::quiet_NaN(), true, 34000);
    b.observe(-1, true, 35000);
    b.observe(std::numeric_limits<float>::infinity(), true, 36000);
    assert(b.target() == 40);
    b.observe(1000, true, 37000);
    b.observe(1000, true, 40000); // no confirmation across a gap
    b.observe(1000, true, 41000);
    assert(b.target() == 40);
    b.observe(0, true, 42000); // reversed direction starts a new candidate
    b.observe(1000, true, 43000);
    b.observe(1000, true, 44000);
    assert(b.target() == 40);
    b.observe(1000, true, 45000);
    assert(b.target() == 4000);
    previous = b.update(45000, true);
    assert(previous < 210); // long delay still advances only one capped smoothing step
    for (uint32_t t = 45010; t <= 57000; t += 10) {
        const auto value = b.update(t, true);
        assert(value >= previous && value <= 4000 && value - previous <= 34);
        previous = value;
    }
    assert(previous == 4000);
    assert(b.update(58000, false) == 0); // sleep is immediate
    b.observe(10, true, 58000); b.observe(10, true, 59000); b.observe(10, true, 60000);
    assert(b.target() == 40 && b.update(100000, false) == 0);
    assert(b.update(200000, true) == 0 && b.update(200010, true) < 40); // no sleep-time catch-up

    DisplayBrightness dark;
    dark.observe(0, true, 0);
    assert(dark.target() == 3);
    for (uint32_t t = 1000; t <= 4000; t += 1000) dark.observe(3.75f, true, t);
    assert(dark.target() == 3); // absolute floor: 12 duty counts = 3 lux
    dark.observe(1e30f, true, 5000); dark.observe(1e30f, true, 6000); dark.observe(1e30f, true, 7000);
    assert(dark.target() == 4095);
    DisplayBrightness wrapped;
    const uint32_t start = 0xfffffc00U;
    wrapped.observe(100, true, start);
    wrapped.update(start, true);
    wrapped.observe(200, true, start + 1000);
    wrapped.observe(200, true, start + 2000);
    wrapped.observe(200, true, start + 3000);
    assert(wrapped.target() == 800);
    assert(wrapped.update(start + 3010, true) > 0 && wrapped.update(start + 3020, true) < 800);
    std::puts("PASS: brightness hysteresis, transient shadows, confirmation, smooth ramps, failures, sleep/wake and clock wrap");
}
