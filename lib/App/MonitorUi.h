#pragma once

#include <MonitorModel.h>
#include <MeasurementGuidance.h>
#include <algorithm>
#include <cstdio>

namespace monitor {

// Canvas is the existing Adafruit display, or the recording canvas in host tests.
// Only the selected page is drawn. No off-screen canvas or framebuffer is allocated.
template<class Canvas> class MonitorUi {
public:
    explicit MonitorUi(Canvas &canvas) : canvas_(canvas) {}

    bool render(Navigation &nav, const Readings &r, const History &history,
                uint32_t nowSeconds, const Diagnostics &diag) {
        if (!nav.needsRender()) return false;
        const Page page = nav.page();
        const bool changedPage = !painted_ || page != paintedPage_;
        canvas_.setTextWrap(false);
        if (changedPage) {
            canvas_.fillScreen(Background);
            header(page);
        }
        switch (page) {
        case Page::Overview: overview(r, changedPage); break;
        case Page::Temperature: graph(Metric::Temperature, r, history, nowSeconds); break;
        case Page::Humidity: graph(Metric::Humidity, r, history, nowSeconds); break;
        case Page::CO2: graph(Metric::CO2, r, history, nowSeconds); break;
        case Page::Light: graph(Metric::Light, r, history, nowSeconds); break;
        case Page::Pressure: graph(Metric::Pressure, r, history, nowSeconds); break;
        case Page::Service: service(history, diag); break;
        default: break;
        }
        painted_ = true;
        paintedPage_ = page;
        nav.rendered();
        return true;
    }

private:
    static constexpr uint16_t Background = 0x0862, Panel = 0x10c4;
    static constexpr uint16_t Foreground = 0xef7d, Muted = 0x8c92, Grid = 0x2947;
    static constexpr uint16_t Accent = 0x4f18;
    static constexpr uint16_t Reference = 0x4ee9, Watch = 0xff20, Attention = 0xfa8a, Neutral = 0x6ddf;
    Canvas &canvas_;
    bool painted_ = false;
    Page paintedPage_ = Page::Overview;
    Readings previous_{};

    void text(int x, int y, const char *value, uint8_t size = 2, uint16_t color = Foreground) {
        canvas_.setCursor(x, y);
        canvas_.setTextSize(size);
        canvas_.setTextColor(color);
        canvas_.print(value);
    }
    void header(Page page) {
        static const char *titles[] = {"ENVIRONMENT", "TEMPERATURE / 6h", "HUMIDITY / 6h",
                                      "CO2 / 6h", "LIGHT / 6h", "PRESSURE / 6h", "SERVICE"};
        static_assert(sizeof(titles) / sizeof(titles[0]) == static_cast<unsigned>(Page::Count), "Missing page title");
        text(14, 14, titles[static_cast<uint8_t>(page)]);
        char count[12];
        std::snprintf(count, sizeof(count), "%u/%u", static_cast<unsigned>(page) + 1, static_cast<unsigned>(Page::Count));
        text(430, 14, count, 2, Accent);
        canvas_.drawFastHLine(12, 42, 456, Grid);
        text(12, 302, "< LEFT", 1, Muted);
        text(426, 302, "RIGHT >", 1, Muted);
    }
    void tile(int x, int y, const char *name, float value, bool valid, const char *unit,
              int decimals, uint16_t color) {
        canvas_.fillRect(x, y, 222, 91, Panel);
        text(x + 12, y + 10, name, 1, Muted);
        char buffer[24];
        if (valid) std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
        else std::snprintf(buffer, sizeof(buffer), "N/A");
        text(x + 12, y + 30, buffer, 3, color);
        text(x + 12, y + 68, unit, 1, Muted);
    }
    void overview(const Readings &r, bool force) {
        const uint8_t changes = previous_.valid ^ r.valid;
        if (force || (changes & Environment) || r.temperature != previous_.temperature)
            tile(12, 55, "TEMPERATURE", r.temperature, r.valid & Environment, "Celsius", 1, 0xfd69);
        if (force || (changes & Environment) || r.humidity != previous_.humidity)
            tile(246, 55, "HUMIDITY", r.humidity, r.valid & Environment, "% relative", 1, 0x6ddf);
        if (force || (changes & CO2) || r.co2 != previous_.co2)
            tile(12, 157, "CO2", r.co2, r.valid & CO2, "ppm", 0, Accent);
        if (force || (changes & Light) || r.light != previous_.light)
            tile(246, 157, "AMBIENT LIGHT", r.light, r.valid & Light, "lux / auto brightness", 0, 0xff16);
        if (force || (changes & Environment) || r.pressure != previous_.pressure) {
            canvas_.fillRect(12, 262, 456, 24, Background);
            char buffer[40];
            if (r.valid & Environment) std::snprintf(buffer, sizeof(buffer), "Pressure %.1f hPa / %.1f mmHg",
                                                   r.pressure, r.pressure * HpaToMmHg);
            else std::snprintf(buffer, sizeof(buffer), "Pressure  N/A");
            text(16, 266, buffer, 2, Muted);
        }
        previous_ = r;
    }
    static uint16_t guidanceColor(Guidance level) {
        switch (level) {
        case Guidance::Reference: return Reference;
        case Guidance::Watch: return Watch;
        case Guidance::Attention: return Attention;
        case Guidance::Neutral: return Neutral;
        default: return Muted;
        }
    }
    static const char *guidanceLabel(Metric metric, Guidance level, float value) {
        if (level == Guidance::Unavailable) return "No valid reading";
        switch (metric) {
        case Metric::Temperature:
            if (level == Guidance::Reference) return "Reference range";
            if (level == Guidance::Watch) return "Warm / check comfort";
            return value < guidance::TemperatureMin ? "Cold / check heating" : "Hot / check cooling";
        case Metric::Humidity:
            if (level == Guidance::Reference) return "Reference range";
            if (level == Guidance::Attention) return "Humid / check moisture";
            return value < guidance::HumidityMin ? "Dry / below reference" : "Above ideal humidity";
        case Metric::CO2:
            if (level == Guidance::Reference) return "Ventilation reference";
            if (level == Guidance::Watch) return "Ventilation / monitor";
            return "High CO2 / check ventilation";
        case Metric::Pressure: return "Atmospheric pressure / No health bands";
        case Metric::Light: return "Ambient light / auto brightness";
        }
        return "";
    }
    void graphSegment(Metric metric, int x0, int y0, float v0, int x1, int y1, float v1) {
        // Split only at crossed thresholds: <= 4 lines, no heap or per-pixel color loop.
        const auto edges = guidanceEdges(metric);
        float previous = 0;
        int lastX = x0, lastY = y0;
        auto part = [&](float end) {
            const int x = x0 + static_cast<int>(std::lround((x1 - x0) * end));
            const int y = y0 + static_cast<int>(std::lround((y1 - y0) * end));
            const float middle = v0 + (v1 - v0) * ((previous + end) * 0.5f);
            canvas_.drawLine(lastX, lastY, x, y, guidanceColor(measurementGuidance(metric, middle)));
            previous = end; lastX = x; lastY = y;
        };
        for (size_t i = 0; i < edges.count; ++i) {
            const float edge = edges.values[v1 > v0 ? i : edges.count - 1 - i];
            if ((v0 < edge && edge < v1) || (v1 < edge && edge < v0))
                part((edge - v0) / (v1 - v0));
        }
        part(1);
    }
    void graph(Metric metric, const Readings &r, const History &history, uint32_t now) {
        constexpr int x0 = 64, x1 = 464, y0 = 104, y1 = 258;
        canvas_.fillRect(0, 50, 480, 242, Background);
        float latest = 0;
        const bool valid = HistoryPoint::from(now, r).value(metric, latest);
        const char *unit = metric == Metric::Temperature ? "C" :
                           metric == Metric::Humidity ? "%" : metric == Metric::CO2 ? "ppm" :
                           metric == Metric::Pressure ? "mmHg" : "lx";
        char buffer[48];
        if (valid) std::snprintf(buffer, sizeof(buffer), "%.*f %s",
                                metric == Metric::Temperature || metric == Metric::Humidity || metric == Metric::Pressure ? 1 : 0, latest, unit);
        else std::snprintf(buffer, sizeof(buffer), "N/A");
        const Guidance level = valid ? measurementGuidance(metric, latest) : Guidance::Unavailable;
        text(64, 61, buffer, 3, guidanceColor(level));
        text(64, 90, guidanceLabel(metric, level, latest), 1, guidanceColor(level));

        float low = 0, high = 0;
        size_t validCount = 0;
        for (size_t i = 0; i < history.size(); ++i) {
            const auto &p = history.at(i);
            float v;
            if (now - p.seconds > HistorySeconds || !p.value(metric, v)) continue;
            if (validCount++ == 0) low = high = v;
            else { low = std::min(low, v); high = std::max(high, v); }
        }
        if (!validCount) { low = 0; high = 1; }
        const float minimumSpan = metric == Metric::Temperature || metric == Metric::Pressure ? 2.0f :
                                  metric == Metric::Humidity ? 10.0f : metric == Metric::CO2 ? 200.0f : 20.0f;
        const float span = std::max(minimumSpan, high - low);
        const float mid = (high + low) * 0.5f;
        low = mid - span * 0.6f;
        high = mid + span * 0.6f;
        if (metric != Metric::Temperature) low = std::max(0.0f, low);
        if (metric == Metric::Humidity) high = std::min(100.0f, high);
        for (int i = 0; i <= 4; ++i) {
            const int y = y0 + (y1 - y0) * i / 4;
            canvas_.drawFastHLine(x0, y, x1 - x0 + 1, Grid);
            std::snprintf(buffer, sizeof(buffer), "%.*f", metric == Metric::Temperature || metric == Metric::Pressure ? 1 : 0,
                          high - (high - low) * i / 4);
            text(4, y - 3, buffer, 1, Muted);
        }
        for (int i = 0; i <= 6; ++i)
            canvas_.drawFastVLine(x0 + (x1 - x0) * i / 6, y0, y1 - y0 + 1, Grid);
        text(x0, 269, "-6h", 1, Muted);
        text((x0 + x1) / 2 - 9, 269, "-3h", 1, Muted);
        text(x1 - 18, 269, "now", 1, Muted);

        bool connected = false;
        int lastX = 0, lastY = 0;
        float lastValue = 0;
        uint32_t lastSeconds = 0;
        for (size_t i = 0; i < history.size(); ++i) {
            const auto &p = history.at(i);
            float v;
            if (now - p.seconds > HistorySeconds || !p.value(metric, v)) {
                connected = false;
                continue;
            }
            const int x = x1 - static_cast<int>((uint64_t(now - p.seconds) * (x1 - x0)) / HistorySeconds);
            const int y = y1 - static_cast<int>((v - low) * (y1 - y0) / (high - low));
            // Do not join across missed sample periods or failed sensor reads.
            if (connected && p.seconds - lastSeconds <= 90)
                graphSegment(metric, lastX, lastY, lastValue, x, y, v);
            else canvas_.drawPixel(x, y, guidanceColor(measurementGuidance(metric, v)));
            connected = true; lastX = x; lastY = y; lastSeconds = p.seconds;
            lastValue = v;
        }
        if (!validCount) text(154, 178, "No history yet", 2, Muted);
        if (guidanceEdges(metric).count) {
            text(64, 287, "Reference", 1, Reference);
            text(160, 287, "Watch", 1, Watch);
            text(238, 287, "Attention", 1, Attention);
            text(360, 287, "1 min/pt", 1, Muted);
        } else text(90, 287, "1 min / point | gaps = no data", 1, Muted);
    }
    void service(const History &history, const Diagnostics &d) {
        canvas_.fillRect(12, 52, 456, 242, Background);
        char buffer[72];
        int y = 56;
        auto row = [&](const char *s) { text(14, y, s, 2); y += 25; };
        std::snprintf(buffer, sizeof(buffer), "Render %.2f / max %.2f ms", d.renderUs / 1000.0, d.maxRenderUs / 1000.0); row(buffer);
        std::snprintf(buffer, sizeof(buffer), "Loop   %.2f / max %.2f ms", d.loopUs / 1000.0, d.maxLoopUs / 1000.0); row(buffer);
        std::snprintf(buffer, sizeof(buffer), "Sensors %.1f ms | age %lu s", d.sensorUs / 1000.0, (unsigned long)d.sampleAgeSeconds); row(buffer);
        std::snprintf(buffer, sizeof(buffer), "Heap %lu / min %lu B", (unsigned long)d.freeHeap, (unsigned long)d.minHeap); row(buffer);
        std::snprintf(buffer, sizeof(buffer), "History %u/%u | %u B", (unsigned)history.size(), (unsigned)HistoryCapacity, (unsigned)sizeof(History)); row(buffer);
        std::snprintf(buffer, sizeof(buffer), "Uptime %lu h %lu m", (unsigned long)(d.uptimeSeconds / 3600), (unsigned long)(d.uptimeSeconds / 60 % 60)); row(buffer);
        std::snprintf(buffer, sizeof(buffer), "Frames %lu | errors %lu", (unsigned long)d.renders, (unsigned long)d.sensorErrors); row(buffer);
        std::snprintf(buffer, sizeof(buffer), "PIR %s | fan %s | LED %u", d.motion ? "ON" : "OFF", d.fanOn ? "ON" : "OFF", d.brightness); row(buffer);
        text(14, y + 2, "Loop = work + delay; render = last frame", 1, Muted);
    }
};

} // namespace monitor
