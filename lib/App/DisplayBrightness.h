#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace monitor {

// Main-loop owned. observe() must be called only for a NEW light measurement,
// not for repeated snapshots, fan updates or on every PWM tick.
class DisplayBrightness {
public:
    static constexpr uint32_t ConfirmationMs = 2000, MaxSampleGapMs = 1500;
    static constexpr uint32_t SmoothMs = 1200, WakeSmoothMs = 250, WakeDurationMs = 900;
    static constexpr uint16_t FallbackDuty = 240, MinDuty = 3, MaxDuty = 4095;

    void observe(float lux, bool valid, uint32_t now) {
        if (!valid || !std::isfinite(lux) || lux < 0) {
            pendingDirection_ = 0; // retain the last accepted target after a failure
            return;
        }
        const uint16_t requested = static_cast<uint16_t>(std::max(3.0f, std::min(4095.0f, lux * 4.0f)));
        if (!lightSeen_) {
            target_ = requested;
            lightSeen_ = true;
            lastSample_ = now;
            return;
        }
        if (uint32_t(now - lastSample_) > MaxSampleGapMs) pendingDirection_ = 0;
        lastSample_ = now;
        // Hysteresis relative to the accepted target, not the moving PWM output.
        const int deadband = std::max(12, int(target_) * 15 / 100);
        const int difference = int(requested) - target_;
        const int direction = difference > deadband ? 1 : difference < -deadband ? -1 : 0;
        if (!direction) { pendingDirection_ = 0; return; }
        if (direction != pendingDirection_) {
            pendingDirection_ = direction;
            pendingSince_ = now;
            pendingSamples_ = 1;
            return;
        }
        if (pendingSamples_ < 3) ++pendingSamples_;
        if (pendingSamples_ >= 3 && uint32_t(now - pendingSince_) >= ConfirmationMs) {
            target_ = requested;
            pendingDirection_ = 0;
        }
    }

    uint16_t update(uint32_t now, bool on) {
        if (!on) {
            currentQ8_ = 0;
            lastUpdate_ = now;
            on_ = false;
            return 0;
        }
        if (!on_) {
            on_ = true;
            wakeStarted_ = lastUpdate_ = now;
            return 0; // ramp from darkness after the first visible frame is drawn
        }
        // A slow TFT frame must not turn into a large catch-up brightness jump.
        const uint32_t elapsed = std::min(uint32_t(now - lastUpdate_), uint32_t(50));
        lastUpdate_ = now;
        const int32_t difference = int32_t(target_) * 256 - currentQ8_;
        if (elapsed && difference) {
            const uint32_t tau = uint32_t(now - wakeStarted_) < WakeDurationMs ? WakeSmoothMs : SmoothMs;
            int32_t step = difference * int32_t(elapsed) / int32_t(tau + elapsed);
            if (!step) step = difference > 0 ? 1 : -1;
            if (std::abs(difference) < 256) step = difference;
            currentQ8_ += step;
        }
        return static_cast<uint16_t>((currentQ8_ + 128) / 256);
    }

    uint16_t target() const { return target_; }

private:
    uint32_t lastSample_ = 0, pendingSince_ = 0, lastUpdate_ = 0, wakeStarted_ = 0;
    int32_t currentQ8_ = 0;
    uint16_t target_ = FallbackDuty;
    int8_t pendingDirection_ = 0;
    uint8_t pendingSamples_ = 0;
    bool lightSeen_ = false, on_ = false;
};

} // namespace monitor
