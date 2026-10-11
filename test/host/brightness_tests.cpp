#include <DisplayBrightness.h>
#include <cassert>
#include <cstdio>
#include <limits>
using namespace monitor;

int main() {
    // Actual PWM ratios, including the reported 30-lux migration regression.
    // The previous shared timer had 256 ticks; the independent timer has 4096.
    const float luxCases[] = {0, 0.5f, 1, 4, 10, 30, 63, 64, 100, 1e30f};
    const uint16_t dutyCases[] = {48, 48, 64, 256, 640, 1920, 4032, 4095, 4095, 4095};
    for (unsigned i = 0; i < sizeof(luxCases) / sizeof(luxCases[0]); ++i) {
        DisplayBrightness curve;
        curve.observe(luxCases[i], true, 0);
        assert(curve.target() == dutyCases[i]);
        if (luxCases[i] >= 1 && luxCases[i] <= 63)
            assert(float(curve.target()) / 4096 == luxCases[i] * 4 / 256);
    }
    DisplayBrightness b;
    assert(b.target() == 3840 && b.update(0, false) == 0);
    b.observe(25, true, 0);
    assert(b.target() == 1600 && b.update(0, true) == 0);
    uint16_t previous = 0;
    for (uint32_t t = 10; t <= 8000; t += 10) {
        const auto value = b.update(t, true);
        assert(value >= previous && value <= 1600 && value - previous <= 64);
        previous = value;
    }
    assert(previous == 1600);
    // One shadow sample, then recovery: no change at all, not a dim/rebrighten cycle.
    b.observe(0, true, 9000);
    b.observe(25, true, 10000);
    assert(b.target() == 1600 && b.update(10000, true) == 1600);
    for (uint32_t t = 11000; t <= 15000; t += 1000) b.observe(t % 2000 ? 21.25f : 28.75f, true, t);
    assert(b.target() == 1600); // +/-15% including exact boundaries stays inside hysteresis
    b.observe(2.5f, true, 16000);
    b.observe(2.5f, true, 17000);
    assert(b.target() == 1600);
    b.observe(2.5f, true, 18000);
    assert(b.target() == 160);
    previous = b.update(18000, true);
    assert(previous > 1520); // even a long gap cannot jump straight to the target
    for (uint32_t t = 18010; t <= 30000; t += 10) {
        const auto value = b.update(t, true);
        assert(value <= previous && value >= 160 && previous - value <= 16);
        previous = value;
    }
    assert(previous == 160);
    // Invalid/nonfinite samples, missed samples and reversals restart confirmation.
    b.observe(62.5f, true, 31000);
    b.observe(62.5f, false, 32000);
    b.observe(62.5f, true, 33000);
    b.observe(std::numeric_limits<float>::quiet_NaN(), true, 34000);
    b.observe(-1, true, 35000);
    b.observe(std::numeric_limits<float>::infinity(), true, 36000);
    assert(b.target() == 160);
    b.observe(62.5f, true, 37000);
    b.observe(62.5f, true, 40000); // no confirmation across a gap
    b.observe(62.5f, true, 41000);
    assert(b.target() == 160);
    b.observe(0, true, 42000); // inside the absolute deadband: cancel the brightening candidate
    b.observe(62.5f, true, 43000);
    b.observe(62.5f, true, 44000);
    assert(b.target() == 160);
    b.observe(62.5f, true, 45000);
    assert(b.target() == 4000);
    previous = b.update(45000, true);
    assert(previous < 330); // long delay still advances only one capped smoothing step
    for (uint32_t t = 45010; t <= 57000; t += 10) {
        const auto value = b.update(t, true);
        assert(value >= previous && value <= 4000 && value - previous <= 34);
        previous = value;
    }
    assert(previous == 4000);
    assert(b.update(58000, false) == 0); // sleep is immediate
    b.observe(2.5f, true, 58000); b.observe(2.5f, true, 59000); b.observe(2.5f, true, 60000);
    assert(b.target() == 160 && b.update(100000, false) == 0);
    assert(b.update(200000, true) == 0 && b.update(200010, true) < 160); // no sleep-time catch-up

    DisplayBrightness dark;
    dark.observe(0, true, 0);
    assert(dark.target() == 48);
    for (uint32_t t = 1000; t <= 4000; t += 1000) dark.observe(3.75f, true, t);
    assert(dark.target() == 48); // absolute floor: 192 duty counts = 3 lux
    dark.observe(1e30f, true, 5000); dark.observe(1e30f, true, 6000); dark.observe(1e30f, true, 7000);
    assert(dark.target() == 4095);
    DisplayBrightness reversed;
    reversed.observe(30, true, 0);
    reversed.observe(10, true, 1000);
    reversed.observe(50, true, 2000); // reversed direction starts a new candidate
    reversed.observe(50, true, 3000);
    assert(reversed.target() == 1920);
    reversed.observe(50, true, 4000);
    assert(reversed.target() == 3200);
    DisplayBrightness wrapped;
    const uint32_t start = 0xfffffc00U;
    wrapped.observe(25, true, start);
    wrapped.update(start, true);
    wrapped.observe(50, true, start + 1000);
    wrapped.observe(50, true, start + 2000);
    wrapped.observe(50, true, start + 3000);
    assert(wrapped.target() == 3200);
    assert(wrapped.update(start + 3010, true) > 0 && wrapped.update(start + 3020, true) < 3200);
    std::puts("PASS: calibrated 12-bit PWM ratios, brightness hysteresis, transient shadows, confirmation, smooth ramps, failures, sleep/wake and clock wrap");
}
