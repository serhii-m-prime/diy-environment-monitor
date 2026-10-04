#pragma once

#include <MonitorModel.h>

namespace monitor {

// Project advisory bands, not a WHO traffic-light standard or medical alarm.
// Sources, exact boundaries and applicability: docs/measurement-colors.md.
enum class Guidance : uint8_t { Reference, Watch, Attention, Neutral, Unavailable };

namespace guidance {
constexpr float TemperatureMin = 18, TemperatureReferenceMax = 24, TemperatureHot = 26;
constexpr float HumidityMin = 30, HumidityReferenceMax = 50, HumidityHigh = 60;
constexpr float CO2ReferenceMax = 1000, CO2High = 1500;
constexpr float TemperatureEdges[] = {TemperatureMin, TemperatureReferenceMax, TemperatureHot};
constexpr float HumidityEdges[] = {HumidityMin, HumidityReferenceMax, HumidityHigh};
constexpr float CO2Edges[] = {CO2ReferenceMax, CO2High};
}

inline Guidance measurementGuidance(Metric metric, float value) {
    if (!std::isfinite(value)) return Guidance::Unavailable;
    switch (metric) {
    case Metric::Temperature:
        if (value < guidance::TemperatureMin || value > guidance::TemperatureHot) return Guidance::Attention;
        return value > guidance::TemperatureReferenceMax ? Guidance::Watch : Guidance::Reference;
    case Metric::Humidity:
        if (value < 0 || value > 100) return Guidance::Unavailable;
        if (value >= guidance::HumidityHigh) return Guidance::Attention;
        return value < guidance::HumidityMin || value > guidance::HumidityReferenceMax ? Guidance::Watch : Guidance::Reference;
    case Metric::CO2:
        if (value < 0) return Guidance::Unavailable;
        if (value > guidance::CO2High) return Guidance::Attention;
        return value > guidance::CO2ReferenceMax ? Guidance::Watch : Guidance::Reference;
    case Metric::Pressure:
    case Metric::Light:
        return Guidance::Neutral;
    }
    return Guidance::Unavailable;
}

struct GuidanceEdges { const float *values; size_t count; };

inline GuidanceEdges guidanceEdges(Metric metric) {
    switch (metric) {
    case Metric::Temperature: return {guidance::TemperatureEdges, 3};
    case Metric::Humidity: return {guidance::HumidityEdges, 3};
    case Metric::CO2: return {guidance::CO2Edges, 2};
    default: return {nullptr, 0};
    }
}

} // namespace monitor
