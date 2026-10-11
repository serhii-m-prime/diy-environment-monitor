#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace monitor {

constexpr uint32_t SamplePeriodMs = 60000;
constexpr uint32_t SampleMaxGapSeconds = SamplePeriodMs / 1000 * 3 / 2;
constexpr size_t HistorySamplesPerPoint = 3;
constexpr uint32_t HistoryPeriodSeconds = SamplePeriodMs / 1000 * HistorySamplesPerPoint;
constexpr uint32_t HistoryMaxGapSeconds = HistoryPeriodSeconds * 3 / 2;
constexpr uint32_t HistoryHours = 24;
constexpr uint32_t HistorySeconds = HistoryHours * 3600;
constexpr size_t HistoryCapacity = HistorySeconds / HistoryPeriodSeconds + 1;
// 1 mmHg = 133.322387415 Pa.
constexpr float HpaToMmHg = 0.750061683f;

enum Valid : uint8_t { Environment = 1, CO2 = 2, Light = 4, Pressure = 8 };
enum class Page : uint8_t { Overview, Temperature, Humidity, CO2, Light, Pressure, Service, Count };
enum class Metric : uint8_t { Temperature, Humidity, CO2, Light, Pressure };

struct Readings {
    float temperature = 0;
    float humidity = 0;
    float pressure = 0;
    float light = 0;
    int co2 = 0;
    uint8_t valid = 0;
};

// Fixed-point samples keep a full day in internal RAM without a framebuffer or PSRAM.
struct HistoryPoint {
    uint32_t seconds = 0;
    int16_t temperature = 0; // hundredths of a degree C
    uint16_t humidity = 0; // hundredths of percent
    uint16_t co2 = 0;
    uint16_t light = 0; // lux
    uint16_t pressure = 0; // tenths of hPa; fits in the existing record padding
    uint8_t valid = 0;

    bool value(Metric metric, float &out) const {
        switch (metric) {
        case Metric::Temperature: out = temperature / 100.0f; return valid & Environment;
        case Metric::Humidity: out = humidity / 100.0f; return valid & Environment;
        case Metric::CO2: out = co2; return valid & CO2;
        case Metric::Light: out = light; return valid & Light;
        case Metric::Pressure: out = pressure * 0.1f * HpaToMmHg; return valid & Pressure;
        }
        return false;
    }

    static HistoryPoint from(uint32_t seconds, const Readings &r) {
        HistoryPoint p;
        p.seconds = seconds;
        if ((r.valid & Environment) && std::isfinite(r.temperature) &&
            r.temperature >= -40 && r.temperature <= 85 &&
            std::isfinite(r.humidity) && r.humidity >= 0 && r.humidity <= 100) {
            p.temperature = static_cast<int16_t>(std::lround(r.temperature * 100));
            p.humidity = static_cast<uint16_t>(std::lround(r.humidity * 100));
            p.valid |= Environment;
        }
        if ((r.valid & CO2) && r.co2 >= 0 && r.co2 <= 65535) {
            p.co2 = static_cast<uint16_t>(r.co2);
            p.valid |= CO2;
        }
        if ((r.valid & Light) && std::isfinite(r.light) && r.light >= 0 && r.light <= 65535) {
            p.light = static_cast<uint16_t>(std::lround(r.light));
            p.valid |= Light;
        }
        if ((r.valid & Environment) && std::isfinite(r.pressure) && r.pressure >= 300 && r.pressure <= 1100) {
            p.pressure = static_cast<uint16_t>(std::lround(r.pressure * 10));
            p.valid |= Pressure;
        }
        return p;
    }
};
static_assert(sizeof(HistoryPoint) <= 16, "History sample exceeds RAM budget");

class History {
public:
    void append(const HistoryPoint &point) {
        points_[next_] = point;
        next_ = (next_ + 1) % HistoryCapacity;
        if (size_ < HistoryCapacity) ++size_;
    }
    size_t size() const { return size_; }
    const HistoryPoint &at(size_t chronologicalIndex) const {
        return points_[(next_ + HistoryCapacity - size_ + chronologicalIndex) % HistoryCapacity];
    }
private:
    HistoryPoint points_[HistoryCapacity]{};
    size_t next_ = 0;
    size_t size_ = 0;
};

// Extend millis() across its 49-day wrap. Tick at least once per millis() period.
class Uptime {
public:
    void tick(uint32_t nowMs) {
        milliseconds_ += static_cast<uint32_t>(nowMs - previousMs_);
        previousMs_ = nowMs;
    }
    uint32_t seconds() const { return static_cast<uint32_t>(milliseconds_ / 1000); }
private:
    uint64_t milliseconds_ = 0;
    uint32_t previousMs_ = 0;
};

class Navigation {
public:
    Page page() const { return page_; }
    void move(int steps) {
        constexpr int count = static_cast<int>(Page::Count);
        page_ = static_cast<Page>((static_cast<int>(page_) + steps % count + count) % count);
        dirty_ = true;
    }
    void setOn(bool on) {
        if (on && !on_) dirty_ = true;
        on_ = on;
    }
    bool isOn() const { return on_; }
    void invalidate() { dirty_ = true; }
    bool needsRender() const { return on_ && dirty_; }
    void rendered() { dirty_ = false; }
private:
    Page page_ = Page::Overview;
    bool on_ = false;
    bool dirty_ = true;
};

struct Diagnostics {
    uint32_t renderUs = 0, maxRenderUs = 0, renders = 0;
    uint32_t loopUs = 0, maxLoopUs = 0;
    uint32_t sensorUs = 0, sampleAgeSeconds = 0;
    uint32_t freeHeap = 0, minHeap = 0, uptimeSeconds = 0;
    uint32_t sensorErrors = 0;
    uint32_t largestHeapBlock = 0, inputStackFree = 0, sensorStackFree = 0, uiStackFree = 0;
    uint8_t resetReason = 0;
    uint32_t dmaTransfers = 0, dmaBufferBytes = 0;
    uint16_t brightness = 0;
    bool fanOn = false, motion = false;
};

struct InputSnapshot {
    uint32_t leftPresses = 0, rightPresses = 0;
    bool motion = false;
};

// The latest minute sample remains present through intervening light updates.
struct SensorSnapshot {
    Readings readings;
    HistoryPoint samplePoint;
    uint32_t sampleSequence = 0, lightSequence = 0, measurementUs = 0, errors = 0;
    bool fanOn = false;
};

class DisplayActivity {
public:
    static constexpr uint32_t TimeoutMs = 30000;
    bool update(uint32_t now, bool motion, bool buttonPressed) {
        if (motion || buttonPressed) { lastActivity_ = now; active_ = true; }
        if (active_ && uint32_t(now - lastActivity_) >= TimeoutMs) active_ = false;
        return active_;
    }
private:
    uint32_t lastActivity_ = 0;
    bool active_ = true; // show the overview for 30 seconds after boot
};

class SampleSchedule {
public:
    static constexpr uint32_t FanLeadMs = 15000;
    // Initial engineering choice; verify actual fan spin-down on the breadboard.
    static constexpr uint32_t FanSettleMs = 2000;
    static_assert(FanLeadMs + FanSettleMs < SamplePeriodMs, "Fan cycle exceeds sample period");
    explicit SampleSchedule(uint32_t now) : phaseStarted_(now), lastSample_(now) {}
    void update(uint32_t now) {
        if (phase_ == Phase::Idle && uint32_t(now - lastSample_) >= SamplePeriodMs - FanLeadMs - FanSettleMs) {
            phase_ = Phase::Fan;
            phaseStarted_ = now;
        } else if (phase_ == Phase::Fan && uint32_t(now - phaseStarted_) >= FanLeadMs) {
            phase_ = Phase::Settling;
            phaseStarted_ = now; // measure from actual switch-off, even after a late worker tick
        }
    }
    bool fanNeeded() const { return phase_ == Phase::Fan; }
    bool due(uint32_t now) const {
        return phase_ == Phase::Settling && uint32_t(now - phaseStarted_) >= FanSettleMs;
    }
    void sampled(uint32_t started) { lastSample_ = started; phase_ = Phase::Idle; }
private:
    enum class Phase : uint8_t { Idle, Fan, Settling };
    Phase phase_ = Phase::Fan;
    uint32_t phaseStarted_;
    uint32_t lastSample_;
};

} // namespace monitor
