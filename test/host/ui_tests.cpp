#include <MonitorUi.h>
#include <cassert>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#if __has_include("glcdfont.c")
#include "glcdfont.c"
#define PREVIEW_BITMAP_FONT
#endif

// Recording canvas checks geometry and exports SVG previews using GFX's 6x8 cell size.
struct Canvas {
    int operations = 0, lines = 0, pixels = 0, x = 0, y = 0, size = 1;
    uint16_t color = 0;
    std::string svg;
    std::vector<std::string> labels;
    std::vector<uint16_t> textColors, lineColors, pixelColors;
    struct Segment { int x0, y0, x1, y1; };
    std::vector<Segment> segments;
    void setTextWrap(bool) {}
    void setCursor(int a, int b) { x = a; y = b; }
    void setTextSize(int s) { size = s; }
    void setTextColor(uint16_t c) { color = c; }
    static std::string rgb(uint16_t c) {
        char b[16];
        std::snprintf(b, sizeof(b), "#%02x%02x%02x", ((c >> 11) & 31) * 255 / 31,
                      ((c >> 5) & 63) * 255 / 63, (c & 31) * 255 / 31);
        return b;
    }
    void bounds(int a, int b, int w, int h) {
        assert(a >= 0 && b >= 0 && w >= 0 && h >= 0 && a + w <= 480 && b + h <= 320);
        ++operations;
    }
    void fillScreen(uint16_t c) { fillRect(0, 0, 480, 320, c); }
    void fillRect(int a, int b, int w, int h, uint16_t c) {
        bounds(a, b, w, h);
        svg += "<rect x='" + std::to_string(a) + "' y='" + std::to_string(b) + "' width='" +
               std::to_string(w) + "' height='" + std::to_string(h) + "' fill='" + rgb(c) + "'/>\n";
    }
    void drawFastHLine(int a, int b, int w, uint16_t c) { fillRect(a,b,w,1,c); }
    void drawFastVLine(int a, int b, int h, uint16_t c) { fillRect(a,b,1,h,c); }
    void drawPixel(int a, int b, uint16_t c) { ++pixels; pixelColors.push_back(c); fillRect(a,b,1,1,c); }
    void drawLine(int a, int b, int c, int d, uint16_t col) {
        bounds(a,b,1,1); bounds(c,d,1,1); ++lines;
        lineColors.push_back(col);
        segments.push_back({a,b,c,d});
        svg += "<line x1='" + std::to_string(a) + "' y1='" + std::to_string(b) + "' x2='" +
               std::to_string(c) + "' y2='" + std::to_string(d) + "' stroke='" + rgb(col) + "'/>\n";
    }
    void print(const char *s) {
        bounds(x, y, static_cast<int>(std::strlen(s)) * 6 * size, 8 * size);
        labels.push_back(s);
        textColors.push_back(color);
#ifdef PREVIEW_BITMAP_FONT
        int cursor = x;
        for (const unsigned char *p = reinterpret_cast<const unsigned char *>(s); *p; ++p) {
            for (int column = 0; column < 5; ++column) {
                const unsigned char bits = font[*p * 5 + column];
                for (int row = 0; row < 8; ++row)
                    if (bits & (1 << row)) fillRect(cursor + column * size, y + row * size, size, size, color);
            }
            cursor += size * 6;
        }
#else
        std::string escaped;
        for (const char *p = s; *p; ++p) {
            if (*p == '<') escaped += "&lt;";
            else if (*p == '>') escaped += "&gt;";
            else if (*p == '&') escaped += "&amp;";
            else escaped += *p;
        }
        svg += "<text x='" + std::to_string(x) + "' y='" + std::to_string(y + 7 * size) +
               "' font-family='monospace' font-size='" + std::to_string(8 * size) + "' textLength='" +
               std::to_string(std::strlen(s) * 6 * size) + "' lengthAdjust='spacingAndGlyphs' fill='" + rgb(color) + "'>" + escaped + "</text>\n";
#endif
    }
    void save(const std::string &path) {
        std::ofstream f(path);
        f << "<svg xmlns='http://www.w3.org/2000/svg' width='480' height='320' viewBox='0 0 480 320'>" << svg << "</svg>";
    }
};

int main() {
    using namespace monitor;
    Readings r;
    r.valid = Environment | CO2 | Light;
    r.temperature = 23.7f; r.humidity = 48.2f; r.co2 = 824; r.light = 135; r.pressure = 1013.2f;
    History history;
    for (unsigned i = 0; i <= 360; ++i) {
        auto sample = r;
        const double age = static_cast<double>(i) - 360;
        sample.temperature += 7 * std::sin(age / 40.0);
        sample.humidity += 23 * std::sin(age / 30.0);
        sample.co2 += 1000 * std::pow(std::sin(age / 45.0), 2);
        sample.light += 100 * std::sin(age / 50.0);
        sample.pressure += 3 * std::sin(age / 80.0);
        if (i > 130 && i < 145) sample.valid = 0;
        history.append(HistoryPoint::from(i * 60, sample));
    }
    Diagnostics diag;
    diag.freeHeap = 243120; diag.minHeap = 240016; diag.loopUs = 12345;
    diag.renderUs = 25400; diag.maxRenderUs = 41234; diag.maxLoopUs = 60111;
    diag.uptimeSeconds = HistorySeconds; diag.brightness = 540;
    constexpr uint16_t green = 0x4ee9, yellow = 0xff20, red = 0xfa8a, blue = 0x6ddf, gray = 0x8c92;
    for (int page = 0; page < static_cast<int>(Page::Count); ++page) {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav;
        nav.move(page);
        assert(!ui.render(nav, r, history, HistorySeconds, diag) && c.operations == 0);
        nav.setOn(true);
        assert(ui.render(nav, r, history, HistorySeconds, diag));
        if (page == static_cast<int>(Page::Pressure)) {
            assert(std::find(c.labels.begin(), c.labels.end(), "760.0 mmHg") != c.labels.end());
            assert(std::find(c.labels.begin(), c.labels.end(), "6/7") != c.labels.end());
            assert(std::find(c.labels.begin(), c.labels.end(), "No trend") != c.labels.end());
            assert(std::find(c.lineColors.begin(), c.lineColors.end(), blue) != c.lineColors.end());
            assert(std::find(c.lineColors.begin(), c.lineColors.end(), green) != c.lineColors.end());
            assert(std::find(c.lineColors.begin(), c.lineColors.end(), yellow) != c.lineColors.end());
        }
        c.save(".pio/host-tests/screen-" + std::to_string(page) + ".svg");
        const int draws = c.operations;
        assert(!ui.render(nav, r, history, HistorySeconds, diag) && c.operations == draws);
        nav.setOn(false); nav.invalidate();
        assert(!ui.render(nav, r, history, HistorySeconds, diag) && c.operations == draws);
        nav.setOn(true);
        ui.render(nav, r, history, HistorySeconds, diag);
        if (page == 0) assert(c.operations == draws); // unchanged overview needs no pixel writes
        else if (page < static_cast<int>(Page::Service)) assert(c.lines > 0 && c.pixels > 1);
    }
    // Empty, flat, invalid and discontinuous history must be drawable without bogus joins.
    for (int metricPage = 1; metricPage < static_cast<int>(Page::Service); ++metricPage) {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav; nav.move(metricPage); nav.setOn(true);
        History sparse;
        ui.render(nav, Readings{}, sparse, 600, diag);
        auto missing = std::find(c.labels.begin(), c.labels.end(), "N/A");
        assert(missing != c.labels.end() && c.textColors[missing - c.labels.begin()] == gray);
        sparse.append(HistoryPoint::from(0, r));
        sparse.append(HistoryPoint::from(60, Readings{}));
        sparse.append(HistoryPoint::from(120, r));
        sparse.append(HistoryPoint::from(600, r));
        nav.invalidate();
        ui.render(nav, r, sparse, 600, diag);
        assert(c.lines == 0 && c.pixels == 3);
        nav.invalidate();
        ui.render(nav, r, sparse, HistorySeconds + 601, diag);
    }
    auto setValue = [](Readings &sample, Page page, float value) {
        switch (page) {
        case Page::Temperature: sample.temperature = value; break;
        case Page::Humidity: sample.humidity = value; break;
        case Page::CO2: sample.co2 = static_cast<int>(value); break;
        case Page::Pressure: sample.pressure = value; break;
        default: assert(false);
        }
    };
    struct Transition { Page page; float from, to; std::vector<uint16_t> colors; };
    const Transition transitions[] = {
        {Page::Temperature, 16, 28, {red, green, yellow, red}},
        {Page::Temperature, 28, 16, {red, yellow, green, red}},
        {Page::Humidity, 20, 70, {yellow, green, yellow, red}},
        {Page::Humidity, 70, 20, {red, yellow, green, yellow}},
        {Page::CO2, 800, 1800, {green, yellow, red}},
        {Page::CO2, 1800, 800, {red, yellow, green}},
        {Page::Pressure, 800, 1050, {blue}},
        {Page::Temperature, 18, 18, {green}},
        {Page::Temperature, 24, 24, {green}},
        {Page::Temperature, 26, 26, {yellow}},
        {Page::Humidity, 30, 30, {green}},
        {Page::Humidity, 50, 50, {green}},
        {Page::Humidity, 60, 60, {red}},
        {Page::CO2, 1000, 1000, {green}},
        {Page::CO2, 1500, 1500, {yellow}}
    };
    for (const auto &t : transitions) {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav; nav.move(static_cast<int>(t.page)); nav.setOn(true);
        History crossing;
        auto sample = r;
        setValue(sample, t.page, t.from);
        crossing.append(HistoryPoint::from(0, sample));
        setValue(sample, t.page, t.to);
        crossing.append(HistoryPoint::from(60, sample));
        ui.render(nav, sample, crossing, 60, diag);
        assert(c.lineColors == t.colors);
        // Split parts must meet, and progress in both screen axes without jumps.
        for (size_t i = 1; i < c.segments.size(); ++i) {
            assert(c.segments[i-1].x1 == c.segments[i].x0);
            assert(c.segments[i-1].y1 == c.segments[i].y0);
        }
        const auto latest = std::find_if(c.labels.begin(), c.labels.end(), [](const std::string &s) {
            return s.find(" ") != std::string::npos && !s.empty() && s.front() >= '0' && s.front() <= '9';
        });
        assert(latest != c.labels.end() && c.textColors[latest - c.labels.begin()] == t.colors.back());
    }
    // An invalid sample or a missed minute breaks the line even across color zones.
    {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav; nav.move(static_cast<int>(Page::CO2)); nav.setOn(true);
        History sparse;
        auto sample = r;
        sample.co2 = 800; sparse.append(HistoryPoint::from(0, sample));
        sparse.append(HistoryPoint::from(60, Readings{}));
        sample.co2 = 1200; sparse.append(HistoryPoint::from(120, sample));
        sample.co2 = 1800; sparse.append(HistoryPoint::from(300, sample));
        ui.render(nav, sample, sparse, 300, diag);
        assert(c.lines == 0 && c.pixelColors == std::vector<uint16_t>({green, yellow, red}));
    }
    // Pressure colors follow the local 3h change, not the absolute pressure.
    // A ramp begins halfway through the buffer, so the plot contains all bands.
    for (int sign : {1, -1}) {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav; nav.move(static_cast<int>(Page::Pressure)); nav.setOn(true);
        History ramp;
        auto sample = r;
        for (int i = 0; i <= 360; ++i) {
            sample.pressure = 1000 + sign * std::max(0, i - 180) * 0.05f;
            ramp.append(HistoryPoint::from(i * 60, sample));
        }
        ui.render(nav, sample, ramp, 21600, diag);
        for (auto color : {blue, green, yellow, red})
            assert(std::find(c.lineColors.begin(), c.lineColors.end(), color) != c.lineColors.end());
        assert(std::find(c.labels.begin(), c.labels.end(), sign > 0 ? "Rising / rapid" : "Falling / rapid") != c.labels.end());
        assert(std::find(c.labels.begin(), c.labels.end(), "Span 6.75 mmHg") != c.labels.end());
        assert(std::find(c.labels.begin(), c.labels.end(), sign > 0 ?
            "6h available: min 750.1 max 756.8 mmHg" : "6h available: min 743.3 max 750.1 mmHg") != c.labels.end());
        assert(c.lineColors.front() == blue && c.lineColors.back() == red);
        assert(c.segments.front().y0 >= 136 && c.segments.back().y1 <= 258);
        c.save(sign > 0 ? ".pio/host-tests/pressure-rising.svg" : ".pio/host-tests/pressure-falling.svg");
        const int before = c.operations;
        nav.setOn(false); nav.invalidate();
        assert(!ui.render(nav, sample, ramp, 21600, diag) && c.operations == before);
        nav.setOn(true);
        ui.render(nav, sample, ramp, 21700, diag);
        assert(c.labels.end() != std::find(c.labels.begin(), c.labels.end(), "3h: insufficient data"));
    }
    {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav; nav.move(static_cast<int>(Page::Pressure)); nav.setOn(true);
        History flat;
        for (unsigned i = 0; i <= 184; ++i) flat.append(HistoryPoint::from(i * 60, r));
        ui.render(nav, r, flat, 184 * 60, diag);
        assert(std::find(c.labels.begin(), c.labels.end(), "3h +0.00 mmHg") != c.labels.end());
        assert(std::find(c.labels.begin(), c.labels.end(), "Steady") != c.labels.end());
        assert(std::find(c.labels.begin(), c.labels.end(), "Span 0.00 mmHg") != c.labels.end());
        assert(c.lineColors.back() == blue); // first trend needs a valid preceding trend
        flat.append(HistoryPoint::from(185 * 60, r));
        nav.invalidate(); ui.render(nav, r, flat, 185 * 60, diag);
        assert(c.lineColors.back() == green);
    }
    {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav; nav.move(static_cast<int>(Page::Pressure)); nav.setOn(true);
        History expired;
        auto sample = r;
        sample.pressure = 1100; expired.append(HistoryPoint::from(0, sample));
        sample.pressure = 1000; expired.append(HistoryPoint::from(60, sample));
        ui.render(nav, sample, expired, HistorySeconds + 1, diag);
        assert(std::find(c.labels.begin(), c.labels.end(), "6h available: min 750.1 max 750.1 mmHg") != c.labels.end());
        assert(std::find(c.labels.begin(), c.labels.end(), "Span 0.00 mmHg") != c.labels.end());
        assert(c.lines == 0 && c.pixels == 1); // old high pressure is excluded from plot and stats
    }
    std::puts("PASS: seven layouts, guidance and pressure tendency colors, statistics, stale data, render gating, gaps and SVG previews");
}
