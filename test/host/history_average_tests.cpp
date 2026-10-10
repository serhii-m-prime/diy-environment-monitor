#include <HistoryAverage.h>
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace monitor;

static HistoryPoint sample(uint32_t seconds, int temperature = 2200) {
    HistoryPoint p;
    p.seconds = seconds; p.temperature = temperature;
    p.humidity = 4567; p.co2 = 65535; p.light = 65535; p.pressure = 11000;
    p.valid = Environment | CO2 | Light | Pressure;
    return p;
}

int main() {
    static_assert(sizeof(HistoryAverage) <= 32, "Unexpected accumulator RAM growth");
    HistoryAverage average;
    HistoryPoint out;
    assert(!average.add(sample(17, -1234), out));
    assert(!average.add(sample(77, -1235), out));
    assert(average.add(sample(137, -1235), out));
    assert(out.seconds == 137 && out.temperature == -1235 && out.humidity == 4567);
    assert(out.co2 == 65535 && out.light == 65535 && out.pressure == 11000);
    assert(out.valid == (Environment | CO2 | Light | Pressure));
    assert(!average.add(sample(197, 1234), out));
    assert(!average.add(sample(257, 1235), out));
    assert(average.add(sample(317, 1235), out) && out.temperature == 1235);

    for (uint8_t flag : {uint8_t(Environment), uint8_t(CO2), uint8_t(Light), uint8_t(Pressure)}) {
        HistoryAverage a;
        auto bad = sample(60); bad.valid &= ~flag;
        assert(!a.add(sample(0), out) && !a.add(bad, out) && a.add(sample(120), out));
        assert(out.valid == ((Environment | CO2 | Light | Pressure) & ~flag));
        assert(!a.add(sample(180), out) && !a.add(sample(240), out) && a.add(sample(300), out));
        assert(out.valid == (Environment | CO2 | Light | Pressure));
    }
    // A lost sample invalidates that group, including losses at a group boundary.
    for (uint32_t missing : {60U, 180U}) {
        HistoryAverage a;
        for (uint32_t t = 0, count = 0; t <= 540; t += 60) {
            if (t == missing) continue;
            const bool complete = a.add(sample(t), out);
            assert(complete == (++count % 3 == 0));
            if (complete) assert((out.valid != 0) == (count != (missing == 60 ? 3U : 6U)));
        }
    }
    // Replayed snapshots and backwards timestamps do not contribute twice.
    {
        HistoryAverage a;
        assert(!a.add(sample(100), out));
        assert(!a.add(sample(100), out) && !a.add(sample(99), out));
        assert(!a.add(sample(160), out) && a.add(sample(220), out));
        assert(out.valid != 0);
    }
    for (uint32_t origin : {0U, 0xffffffd0U}) {
        HistoryAverage a;
        assert(!a.add(sample(origin), out));
        assert(!a.add(sample(origin + 61), out));
        assert(a.add(sample(origin + 122), out) && out.valid != 0);
    }
    // A long outage never blends old data into a valid new average.
    {
        HistoryAverage a;
        assert(!a.add(sample(0), out) && !a.add(sample(10000), out));
        assert(a.add(sample(10060), out) && out.valid == 0);
        assert(!a.add(sample(10120), out) && !a.add(sample(10180), out));
        assert(a.add(sample(10240), out) && out.valid != 0);
    }
    std::puts("PASS: three-minute means, rounding, per-channel validity, gaps, duplicates, recovery and clock wrap");
}
