#include <MonitorUi.h>
#include <cassert>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

// Recording canvas checks geometry and exports SVG previews using GFX's 6x8 cell size.
struct Canvas {
    int operations = 0, lines = 0, pixels = 0, x = 0, y = 0, size = 1;
    uint16_t color = 0;
    std::string svg;
    std::vector<std::string> labels;
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
    void drawPixel(int a, int b, uint16_t c) { ++pixels; fillRect(a,b,1,1,c); }
    void drawLine(int a, int b, int c, int d, uint16_t col) {
        bounds(a,b,1,1); bounds(c,d,1,1); ++lines;
        svg += "<line x1='" + std::to_string(a) + "' y1='" + std::to_string(b) + "' x2='" +
               std::to_string(c) + "' y2='" + std::to_string(d) + "' stroke='" + rgb(col) + "'/>\n";
    }
    void print(const char *s) {
        bounds(x, y, static_cast<int>(std::strlen(s)) * 6 * size, 8 * size);
        labels.push_back(s);
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
        sample.temperature += 2 * std::sin(i / 40.0);
        sample.humidity += 8 * std::sin(i / 30.0);
        sample.co2 += 300 * std::sin(i / 45.0);
        sample.light += 100 * std::sin(i / 50.0);
        if (i > 130 && i < 145) sample.valid = 0;
        history.append(HistoryPoint::from(i * 60, sample));
    }
    Diagnostics diag;
    diag.freeHeap = 243120; diag.minHeap = 240016; diag.loopUs = 12345;
    diag.renderUs = 25400; diag.maxRenderUs = 41234; diag.maxLoopUs = 60111;
    diag.uptimeSeconds = HistorySeconds; diag.brightness = 540;
    for (int page = 0; page < static_cast<int>(Page::Count); ++page) {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav;
        nav.move(page);
        assert(!ui.render(nav, r, history, HistorySeconds, diag) && c.operations == 0);
        nav.setOn(true);
        assert(ui.render(nav, r, history, HistorySeconds, diag));
        c.save(".pio/host-tests/screen-" + std::to_string(page) + ".svg");
        const int draws = c.operations;
        assert(!ui.render(nav, r, history, HistorySeconds, diag) && c.operations == draws);
        nav.setOn(false); nav.invalidate();
        assert(!ui.render(nav, r, history, HistorySeconds, diag) && c.operations == draws);
        nav.setOn(true);
        ui.render(nav, r, history, HistorySeconds, diag);
        if (page == 0) assert(c.operations == draws); // unchanged overview needs no pixel writes
        else if (page < 5) assert(c.lines > 0 && c.pixels > 1);
    }
    // Empty, flat, invalid and discontinuous history must be drawable without bogus joins.
    for (int metricPage = 1; metricPage <= 4; ++metricPage) {
        Canvas c;
        MonitorUi<Canvas> ui(c);
        Navigation nav; nav.move(metricPage); nav.setOn(true);
        History sparse;
        ui.render(nav, Readings{}, sparse, 600, diag);
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
    std::puts("PASS: six screen layouts, off/clean render gating, graph gaps and SVG previews");
}
