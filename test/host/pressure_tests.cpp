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
    // Boundary values, direction symmetry and the endpoint mean's 0.05 hPa increments.
    for (int change : {0, 1, 2, 30, 31, 70, 71, -1, -2, -30, -31, -70, -71}) {
        History h;
        for (unsigned i = 0; i <= 61; ++i)
            h.append(point(i * 180, 10000 + (i == 61 ? change : 0)));
        const auto trend = latestPressureTrend(h, 61 * 180);
        assert(trend.valid && std::fabs(trend.deltaHpa - change / 20.0f) < 0.00001f);
        const int magnitude = std::abs(change);
        assert(trend.guidance() == (magnitude > 70 ? Guidance::Attention :
                                   magnitude > 30 ? Guidance::Watch : Guidance::Reference));
        assert(trend.direction() == (magnitude < 2 ? 0 : change > 0 ? 1 : -1));
    }
    // Every plotted point gets its own trend, not the last point's color.
    History ramp;
    for (unsigned i = 0; i <= 480; ++i) ramp.append(point(i * 180, 10000 + i));
    PressureTrendSeries series(ramp);
    for (size_t i = 0; i < ramp.size(); ++i) {
        const auto trend = series.at(i);
        assert(trend.valid == (i >= 61));
        if (trend.valid) assert(std::fabs(trend.deltaHpa - 6) < 0.0001f && trend.direction() == 1);
    }
    assert(latestPressureTrend(ramp, 86670).valid);
    assert(!latestPressureTrend(ramp, 86671).valid); // stale current data
    assert(!latestPressureTrend(ramp, 86399).valid); // future sample is not fresh

    // A failed endpoint sample or a missed three-minute point invalidates the mean. An
    // interior gap outside the endpoint windows does not invalidate their delta.
    for (unsigned bad : {0, 1, 20, 60, 61}) {
        for (bool missing : {false, true}) {
            History h;
            for (unsigned i = 0; i <= 61; ++i) {
                if (i == bad && missing) continue;
                h.append(point(i * 180, 10000 + i, i != bad));
            }
            assert(latestPressureTrend(h, 61 * 180).valid == (bad == 20));
        }
    }
    // Timestamps, not array offsets: tolerate worker jitter and reject a missing
    // old reference. No extrapolation from a shorter or different time interval.
    for (uint32_t spacing : {90U, 180U, 181U, 270U, 271U}) {
        History h;
        for (unsigned i = 0; i <= 200; ++i) h.append(point(i * spacing, 10000));
        assert(latestPressureTrend(h, 200 * spacing).valid == (spacing <= 270));
    }
    {
        History h;
        for (unsigned i = 0; i <= 65; ++i) {
            if (i >= 2 && i <= 5) continue;
            h.append(point(i * 180));
        }
        assert(!latestPressureTrend(h, 65 * 180).valid);
    }
    // Circular overwrite and uint32 seconds rollover retain chronological meaning.
    for (uint32_t origin : {0U, 0xffffe000U}) {
        History h;
        for (unsigned i = 0; i <= 800; ++i) h.append(point(origin + i * 180, 10000 + i));
        const auto trend = latestPressureTrend(h, origin + 800 * 180);
        assert(trend.valid && std::fabs(trend.deltaHpa - 6) < 0.0001f);
    }
    std::puts("PASS: pressure means, 3h boundaries, historical colors, missing/stale data, jitter and ring/time rollover");
}
