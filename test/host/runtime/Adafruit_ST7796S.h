#pragma once
#include <SPI.h>
#include <cassert>
#include <cstdint>
constexpr int ST7796S_BGR = 0, ST77XX_INVOFF = 1, ST77XX_DISPOFF = 2, ST77XX_DISPON = 3;
struct Adafruit_ST7796S {
    bool on = false;
    unsigned draws = 0;
    Adafruit_ST7796S(SPIClass*, int, int, int) {}
    void init(int,int,int,int,int) {}
    void setSPISpeed(uint32_t) {}
    void setRotation(int) {}
    void sendCommand(int c) {
        if (c == ST77XX_DISPON) on = true;
        if (c == ST77XX_DISPOFF) on = false;
    }
    void drawing() { assert(on); ++draws; }
    void setTextWrap(bool) {}
    void setCursor(int,int) {}
    void setTextSize(int) {}
    void setTextColor(uint16_t) {}
    void print(const char*) { drawing(); }
    void fillScreen(uint16_t) { drawing(); }
    void fillRect(int,int,int,int,uint16_t) { drawing(); }
    void drawFastHLine(int,int,int,uint16_t) { drawing(); }
    void drawFastVLine(int,int,int,uint16_t) { drawing(); }
    void drawPixel(int,int,uint16_t) { drawing(); }
    void drawLine(int,int,int,int,uint16_t) { drawing(); }
};
