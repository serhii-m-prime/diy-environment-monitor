#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>

namespace monitor {
struct PaintBounds {
    int x = 0, y = 0, width = 0, height = 0;
    bool empty() const { return !width || !height; }
};

// Rasterize only one RGB565 tile. The same 6x8 ASCII font and primitive geometry
// are used by the existing UI. Pixel words are stored in SPI wire byte order.
class TileCanvas {
public:
    static constexpr int Width = 480, Height = 320;
    explicit TileCanvas(const uint8_t *bitmapFont) : font_(bitmapFont) {}
    void plan() { planning_ = true; damage_ = PaintBounds{}; }
    PaintBounds damage() const { return damage_; }
    void tile(int x, int y, int width, int height, uint16_t *pixels) {
        planning_ = false; tile_ = {x, y, width, height}; pixels_ = pixels;
    }
    void setTextWrap(bool) {}
    void setCursor(int x, int y) { cursorX_ = x; cursorY_ = y; }
    void setTextSize(int size) { textSize_ = std::max(1, size); }
    void setTextColor(uint16_t color) { textColor_ = color; }
    void fillScreen(uint16_t color) { fillRect(0, 0, Width, Height, color); }
    void fillRect(int x, int y, int width, int height, uint16_t color) {
        if (width <= 0 || height <= 0) return;
        const int x0 = std::max(x, planning_ ? 0 : tile_.x);
        const int y0 = std::max(y, planning_ ? 0 : tile_.y);
        const int x1 = std::min(x + width, planning_ ? Width : tile_.x + tile_.width);
        const int y1 = std::min(y + height, planning_ ? Height : tile_.y + tile_.height);
        if (x1 <= x0 || y1 <= y0) return;
        if (planning_) { include(x0, y0, x1, y1); return; }
        const uint16_t wireColor = static_cast<uint16_t>((color >> 8) | (color << 8));
        for (int row = y0; row < y1; ++row) {
            auto *start = pixels_ + (row - tile_.y) * tile_.width + x0 - tile_.x;
            std::fill(start, start + x1 - x0, wireColor);
        }
    }
    void drawPixel(int x, int y, uint16_t color) { fillRect(x, y, 1, 1, color); }
    void drawFastHLine(int x, int y, int width, uint16_t color) { fillRect(x, y, width, 1, color); }
    void drawFastVLine(int x, int y, int height, uint16_t color) { fillRect(x, y, 1, height, color); }
    void drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
        if (planning_) {
            fillRect(std::min(x0, x1), std::min(y0, y1), std::abs(x1 - x0) + 1,
                     std::abs(y1 - y0) + 1, color);
            return;
        }
        // Match Adafruit GFX's Bresenham tie breaking, including steep lines.
        const bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
        if (steep) { std::swap(x0, y0); std::swap(x1, y1); }
        if (x0 > x1) { std::swap(x0, x1); std::swap(y0, y1); }
        const int dx = x1 - x0, dy = std::abs(y1 - y0), step = y0 < y1 ? 1 : -1;
        int error = dx / 2;
        for (; x0 <= x1; ++x0) {
            if (steep) drawPixel(y0, x0, color); else drawPixel(x0, y0, color);
            error -= dy;
            if (error < 0) { y0 += step; error += dx; }
        }
    }
    void print(const char *text) {
        for (const unsigned char *p = reinterpret_cast<const unsigned char *>(text); *p; ++p) {
            if (*p == '\n') { cursorX_ = 0; cursorY_ += 8 * textSize_; continue; }
            if (*p == '\r') continue;
            for (int column = 0; column < 5; ++column) {
                const uint8_t bits = font_[*p * 5 + column];
                for (int row = 0; row < 8; ++row)
                    if (bits & (1 << row)) fillRect(cursorX_ + column * textSize_,
                        cursorY_ + row * textSize_, textSize_, textSize_, textColor_);
            }
            cursorX_ += 6 * textSize_;
        }
    }
private:
    void include(int x0, int y0, int x1, int y1) {
        if (damage_.empty()) { damage_ = {x0, y0, x1 - x0, y1 - y0}; return; }
        const int right = std::max(damage_.x + damage_.width, x1);
        const int bottom = std::max(damage_.y + damage_.height, y1);
        damage_.x = std::min(damage_.x, x0); damage_.y = std::min(damage_.y, y0);
        damage_.width = right - damage_.x; damage_.height = bottom - damage_.y;
    }
    const uint8_t *font_;
    uint16_t *pixels_ = nullptr;
    PaintBounds damage_{}, tile_{};
    int cursorX_ = 0, cursorY_ = 0, textSize_ = 1;
    uint16_t textColor_ = 0xffff;
    bool planning_ = true;
};
} // namespace monitor
