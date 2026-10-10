#pragma once

#include <MonitorModel.h>

namespace monitor {

// Main-loop owned, non-overlapping groups of three minute samples. No heap or
// second history buffer. A channel needs all three valid samples for its mean.
class HistoryAverage {
public:
    bool add(const HistoryPoint &sample, HistoryPoint &out) {
        const uint32_t gap = sample.seconds - lastSeconds_;
        // Ignore duplicate/out-of-order snapshots; unsigned subtraction supports wrap.
        if (seen_ && (!gap || gap > UINT32_MAX / 2)) return false;
        valid_ = count_ ? valid_ & sample.valid : sample.valid;
        if (seen_ && (gap < SamplePeriodMs / 1000 / 2 || gap > SampleMaxGapSeconds))
            valid_ = 0; // also check the boundary between two groups
        lastSeconds_ = sample.seconds;
        seen_ = true;
        temperature_ += sample.temperature;
        humidity_ += sample.humidity;
        co2_ += sample.co2;
        light_ += sample.light;
        pressure_ += sample.pressure;
        if (++count_ < HistorySamplesPerPoint) return false;

        out = HistoryPoint{};
        out.seconds = sample.seconds; // end of the group, never a synthetic timestamp
        out.temperature = static_cast<int16_t>((temperature_ + (temperature_ < 0 ? -1 : 1)) / 3);
        out.humidity = static_cast<uint16_t>((humidity_ + 1) / 3);
        out.co2 = static_cast<uint16_t>((co2_ + 1) / 3);
        out.light = static_cast<uint16_t>((light_ + 1) / 3);
        out.pressure = static_cast<uint16_t>((pressure_ + 1) / 3);
        out.valid = valid_;
        count_ = 0;
        temperature_ = 0;
        humidity_ = co2_ = light_ = pressure_ = 0;
        return true;
    }

private:
    static_assert(HistorySamplesPerPoint == 3, "Averaging rounding assumes three samples");
    int32_t temperature_ = 0;
    uint32_t humidity_ = 0, co2_ = 0, light_ = 0, pressure_ = 0, lastSeconds_ = 0;
    uint8_t count_ = 0, valid_ = 0;
    bool seen_ = false;
};

} // namespace monitor
