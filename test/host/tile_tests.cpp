#define PROGMEM
#include <MonitorFont.h>
#include <TileCanvas.h>
#include <MonitorUi.h>
#include <array>
#include <cassert>
#include <cstdio>

using namespace monitor;
int main() {
    std::array<uint16_t, 480 * 320> reference{}, panel{};
    std::array<uint16_t, 480 * 16 + 2> tilePixels{};
    TileCanvas direct(font), tiled(font);
    MonitorUi<TileCanvas> directUi(direct), tileUi(tiled);
    History history;
    Readings r;
    r.valid = Environment | CO2 | Light;
    r.temperature = 22.5; r.humidity = 45; r.co2 = 800; r.light = 120; r.pressure = 1013;
    for (size_t i = 0; i < HistoryCapacity; ++i) {
        r.temperature = 15 + (i % 60) / 3.f; r.humidity = 20 + i % 70;
        r.co2 = 600 + i % 50 * 50; r.pressure = 990 + i / 40.f;
        history.append(HistoryPoint::from(i * HistoryPeriodSeconds, r));
    }
    const uint32_t now = (HistoryCapacity - 1) * HistoryPeriodSeconds;
    Diagnostics diagnostics;
    Navigation nav;
    nav.setOn(true);
    unsigned transfers = 0;
    auto frame = [&] {
        auto directNav = nav, tileNav = nav;
        direct.tile(0, 0, 480, 320, reference.data());
        const bool expected = directUi.render(directNav, r, history, now, diagnostics);
        if (tileNav.needsRender()) {
            tiled.plan();
            tileUi.render(tileNav, r, history, now, diagnostics, false);
            const auto bounds = tiled.damage();
            for (int y = bounds.y; y < bounds.y + bounds.height; y += 16) {
                const int rows = std::min(16, bounds.y + bounds.height - y);
                tilePixels.front() = tilePixels.back() = 0xabcd;
                tiled.tile(bounds.x, y, bounds.width, rows, tilePixels.data() + 1);
                tileUi.render(tileNav, r, history, now, diagnostics, false, true);
                for (int row = 0; row < rows; ++row)
                    std::copy_n(tilePixels.data() + 1 + row * bounds.width, bounds.width,
                                panel.data() + (y + row) * 480 + bounds.x);
                assert(tilePixels.front() == 0xabcd && tilePixels.back() == 0xabcd);
                ++transfers;
            }
            tileUi.completeFrame(tileNav, r);
        }
        assert(expected == nav.needsRender());
        assert(panel == reference); // partial updates must preserve labels/backgrounds
        assert(!tileNav.needsRender());
        nav = tileNav;
    };
    frame(); assert(transfers == 20);
    const auto initial = transfers;
    frame(); assert(transfers == initial);
    r.temperature += 1; nav.invalidate(); frame();
    assert(transfers > initial && transfers < initial + 20);
    r.valid &= ~CO2; nav.invalidate(); frame();
    r.valid |= CO2; r.co2 = 1500; nav.invalidate(); frame();
    for (unsigned i = 1; i < static_cast<unsigned>(Page::Count); ++i) {
        nav.move(1); frame();
        r.temperature += .5f; nav.invalidate(); frame();
    }
    nav.setOn(false); nav.invalidate();
    const auto sleeping = transfers;
    frame(); assert(transfers == sleeping);
    nav.setOn(true); frame();
    nav.move(1); frame(); // return to overview from service

    // Explicit clipping and RGB565 wire byte order, including an odd-width tile.
    std::array<uint16_t, 17> guarded;
    guarded.fill(0xcccc);
    tiled.tile(10, 10, 5, 3, guarded.data() + 1);
    tiled.fillRect(-50, -50, 100, 100, 0xf800);
    assert(guarded.front() == 0xcccc && guarded.back() == 0xcccc);
    for (int i = 1; i < 16; ++i) assert(guarded[i] == 0x00f8);
    tiled.drawPixel(9, 10, 0xffff);
    tiled.drawPixel(15, 10, 0xffff);
    assert(guarded.front() == 0xcccc && guarded.back() == 0xcccc);
    tiled.drawLine(10, 10, 14, 12, 0x07e0);
    assert(guarded[1] == 0xe007 && guarded[15] == 0xe007);
    std::puts("PASS: tile/full raster parity for seven pages and incremental updates, sleep/wake, clipping and RGB565 byte order");
}
