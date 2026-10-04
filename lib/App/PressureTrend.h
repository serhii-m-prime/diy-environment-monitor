#pragma once

#include <MeasurementGuidance.h>

namespace monitor {

// Five consecutive minute samples at each end; the reference ends 3 h earlier.
// Only the existing RAM history is used. See docs/measurement-colors.md.
constexpr uint32_t PressureTrendSeconds = 3 * 3600;
constexpr uint32_t PressureTrendToleranceSeconds = 90;
constexpr size_t PressureMeanSamples = 5;

struct PressureTrend {
    float deltaHpa = 0;
    bool valid = false;

    Guidance guidance() const {
        if (!valid) return Guidance::Neutral;
        const float magnitude = std::fabs(deltaHpa);
        return magnitude > 3.5f ? Guidance::Attention :
               magnitude > 1.5f ? Guidance::Watch : Guidance::Reference;
    }
    // Small differences below the stored 0.1 hPa resolution have no arrow.
    int direction() const { return !valid || std::fabs(deltaHpa) < 0.1f ? 0 : deltaHpa > 0 ? 1 : -1; }
};

// Visit indices in chronological order. The reference cursor advances at most N
// times and each mean reads five points: O(N) for the entire graph, O(1) storage.
class PressureTrendSeries {
public:
    explicit PressureTrendSeries(const History &history) : history_(history) {}

    PressureTrend at(size_t index) {
        if (index >= history_.size()) return {};
        const uint32_t now = history_.at(index).seconds;
        if (uint32_t(now - history_.at(0).seconds) < PressureTrendSeconds) return {};
        while (reference_ + 1 < index &&
               uint32_t(now - history_.at(reference_ + 1).seconds) >= PressureTrendSeconds)
            ++reference_;
        if (uint32_t(now - history_.at(reference_).seconds) >
            PressureTrendSeconds + PressureTrendToleranceSeconds) return {};
        uint32_t currentSum = 0, referenceSum = 0;
        if (!meanSum(index, currentSum) || !meanSum(reference_, referenceSum)) return {};
        // Subtract fixed-point sums before conversion: no cancellation near 1000 hPa.
        const float delta = (static_cast<int32_t>(currentSum) - static_cast<int32_t>(referenceSum)) /
                            (10.0f * PressureMeanSamples);
        return {delta, true};
    }

private:
    const History &history_;
    size_t reference_ = 0;

    bool meanSum(size_t last, uint32_t &sum) const {
        if (last + 1 < PressureMeanSamples) return false;
        const size_t first = last + 1 - PressureMeanSamples;
        if (uint32_t(history_.at(last).seconds - history_.at(first).seconds) > 300) return false;
        for (size_t i = first; i <= last; ++i) {
            const auto &p = history_.at(i);
            if (!(p.valid & Pressure)) return false;
            if (i > first) {
                const uint32_t gap = p.seconds - history_.at(i - 1).seconds;
                if (gap < 30 || gap > PressureTrendToleranceSeconds) return false;
            }
            sum += p.pressure;
        }
        return true;
    }
};

inline PressureTrend latestPressureTrend(const History &history, uint32_t now) {
    if (!history.size() || uint32_t(now - history.at(history.size() - 1).seconds) >
        PressureTrendToleranceSeconds) return {};
    return PressureTrendSeries(history).at(history.size() - 1);
}

} // namespace monitor
