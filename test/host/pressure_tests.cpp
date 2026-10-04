#include <PressureTrend.h>
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace monitor;

static HistoryPoint point(uint32_t seconds, uint16_t tenths = 10000, bool valid = true) {
    HistoryPoint p;
    p.seconds = seconds; p.pressure = tenths; p.valid = valid ? Pressure : 0;
    return p;
}

int main() {
    assert(sizeof(HistoryPoint) == 16);
    assert(!latestPressureTrend(History{}, 0).valid);
    // Boundary values, direction symmetry and the average's 0.02 hPa increments.
    for (int change : {0, 4, 5, 75, 76, 175, 176, -4, -5, -75, -76, -175, -176}) {
        History h;
        for (unsigned i = 0; i <= 184; ++i)
            h.append(point(i * 60, 10000 + (i == 184 ? change : 0)));
        const auto trend = latestPressureTrend(h, 184 * 60);
        assert(trend.valid && std::fabs(trend.deltaHpa - change / 50.0f) < 0.00001f);
        const int magnitude = std::abs(change);
        assert(trend.guidance() == (magnitude > 175 ? Guidance::Attention :
                                   magnitude > 75 ? Guidance::Watch : Guidance::Reference));
        assert(trend.direction() == (magnitude < 5 ? 0 : change > 0 ? 1 : -1));
    }
    // Every plotted point gets its own trend, not the last point's color.
    History ramp;
    for (unsigned i = 0; i <= 360; ++i) ramp.append(point(i * 60, 10000 + i));
    PressureTrendSeries series(ramp);
    for (size_t i = 0; i < ramp.size(); ++i) {
        const auto trend = series.at(i);
        assert(trend.valid == (i >= 184));
        if (trend.valid) assert(std::fabs(trend.deltaHpa - 18) < 0.0001f && trend.direction() == 1);
    }
    assert(latestPressureTrend(ramp, 21690).valid);
    assert(!latestPressureTrend(ramp, 21691).valid); // stale current data
    assert(!latestPressureTrend(ramp, 21599).valid); // future sample is not fresh

    // A failed endpoint sample or a missed minute invalidates the mean. An
    // interior gap outside the endpoint windows does not invalidate their delta.
    for (unsigned bad : {0, 4, 100, 180, 184}) {
        for (bool missing : {false, true}) {
            History h;
            for (unsigned i = 0; i <= 184; ++i) {
                if (i == bad && missing) continue;
                h.append(point(i * 60, 10000 + i, i != bad));
            }
            assert(latestPressureTrend(h, 184 * 60).valid == (bad == 100));
        }
    }
    // Timestamps, not array offsets: tolerate worker jitter and reject a missing
    // old reference. No extrapolation from a shorter or different time interval.
    for (uint32_t spacing : {60U, 61U, 90U}) {
        History h;
        for (unsigned i = 0; i <= 184; ++i) h.append(point(i * spacing, 10000));
        assert(latestPressureTrend(h, 184 * spacing).valid == (spacing != 90));
    }
    {
        History h;
        for (unsigned i = 0; i <= 190; ++i) {
            if (i >= 5 && i <= 10) continue;
            h.append(point(i * 60));
        }
        assert(!latestPressureTrend(h, 190 * 60).valid);
    }
    // Circular overwrite and uint32 seconds rollover retain chronological meaning.
    for (uint32_t origin : {0U, 0xffffe000U}) {
        History h;
        for (unsigned i = 0; i <= 800; ++i) h.append(point(origin + i * 60, 10000 + i));
        const auto trend = latestPressureTrend(h, origin + 800 * 60);
        assert(trend.valid && std::fabs(trend.deltaHpa - 18) < 0.0001f);
    }
    std::puts("PASS: pressure means, 3h boundaries, historical colors, missing/stale data, jitter and ring/time rollover");
}
