#pragma once
#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <glcdfont.c>
#include <MonitorUi.h>
#include <TileCanvas.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_lcd_panel_io.h>
#include <esp_heap_caps.h>
#include <freertos/semphr.h>

namespace monitor {
class DmaDisplay : public TileCanvas {
public:
    DmaDisplay() : TileCanvas(font) {}
    void begin(int clock, int mosi, int cs, int dc, int reset, uint32_t frequency) {
        done_ = xSemaphoreCreateBinary();
        if (!done_) abort();
        for (auto &buffer : buffers_) {
            buffer = static_cast<uint16_t *>(heap_caps_malloc(Width * TileRows * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
            if (!buffer) abort();
        }
        spi_bus_config_t bus{};
        bus.sclk_io_num = clock; bus.mosi_io_num = mosi; bus.miso_io_num = -1;
        bus.quadwp_io_num = -1; bus.quadhd_io_num = -1;
        bus.max_transfer_sz = Width * TileRows * 2;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
        esp_lcd_panel_io_spi_config_t config{};
        config.cs_gpio_num = cs; config.dc_gpio_num = dc;
        config.spi_mode = 0; config.pclk_hz = frequency;
        config.trans_queue_depth = 2; config.lcd_cmd_bits = 8; config.lcd_param_bits = 8;
        config.on_color_trans_done = completed; config.user_ctx = this;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(reinterpret_cast<esp_lcd_spi_bus_handle_t>(SPI2_HOST), &config, &io_));
        ESP_ERROR_CHECK(gpio_set_direction(static_cast<gpio_num_t>(reset), GPIO_MODE_OUTPUT));
        gpio_set_level(static_cast<gpio_num_t>(reset), 0); delay(20);
        gpio_set_level(static_cast<gpio_num_t>(reset), 1); delay(150);
        // ST7796S settings match Adafruit's MIT-licensed driver, rotation 3/BGR.
        sendCommand(0x01); delay(150);
        parameter(0xf0, 0xc3); parameter(0xf0, 0x96);
        parameter(0xc5, 0x1c); parameter(0x36, 0xe8); parameter(0x3a, 0x55);
        parameter(0xb0, 0x80); parameter(0xb4, 0x00);
        const uint8_t function[] = {0x80, 0x02, 0x3b};
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_, 0xb6, function, sizeof(function)));
        parameter(0xb7, 0xc6); parameter(0xf0, 0x69); parameter(0xf0, 0x3c);
        sendCommand(0x11); delay(150); sendCommand(0x20); sendCommand(0x28);
    }
    void sendCommand(uint8_t command) {
        drain();
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_, command, nullptr, 0));
    }
    bool render(MonitorUi<DmaDisplay> &ui, Navigation &nav, const Readings &r,
                const History &history, uint32_t now, const Diagnostics &diag) {
        if (!nav.needsRender()) return false;
        plan();
        ui.render(nav, r, history, now, diag, false);
        const auto bounds = damage();
        // A complete scene is clipped to the changed area, so transparent text
        // and pixels between changed tiles retain their correct background.
        for (int y = bounds.y; y < bounds.y + bounds.height; y += TileRows) {
            const int rows = std::min(TileRows, bounds.y + bounds.height - y);
            tile(bounds.x, y, bounds.width, rows, buffers_[next_]);
            ui.render(nav, r, history, now, diag, false, true);
            flush(bounds.x, y, bounds.width, rows, buffers_[next_]);
            next_ ^= 1; // rasterize the other buffer while SPI transfers this one
        }
        drain();
        ui.completeFrame(nav, r);
        return true;
    }
    uint32_t transfers() const { return transfers_; }
    uint32_t pixelBytes() const { return pixelBytes_; }
    static constexpr int TileRows = CONFIG_MONITOR_DMA_TILE_ROWS;
    static constexpr uint32_t BufferBytes = Width * TileRows * 2 * 2;
private:
    static bool completed(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t *, void *context) {
        BaseType_t wake = pdFALSE;
        xSemaphoreGiveFromISR(static_cast<DmaDisplay *>(context)->done_, &wake);
        return wake == pdTRUE;
    }
    void parameter(uint8_t command, uint8_t value) {
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_, command, &value, 1));
    }
    void drain() {
        if (pending_) {
            if (xSemaphoreTake(done_, pdMS_TO_TICKS(5000)) != pdTRUE) abort();
            pending_ = false;
        }
    }
    void flush(int x, int y, int width, int rows, const uint16_t *pixels) {
        drain(); // completed buffer can only be reused after this acknowledgement
        const uint8_t columns[] = {uint8_t(x >> 8), uint8_t(x), uint8_t((x + width - 1) >> 8), uint8_t(x + width - 1)};
        const uint8_t lines[] = {uint8_t(y >> 8), uint8_t(y), uint8_t((y + rows - 1) >> 8), uint8_t(y + rows - 1)};
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_, 0x2a, columns, sizeof(columns)));
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_, 0x2b, lines, sizeof(lines)));
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(io_, 0x2c, pixels, width * rows * 2));
        pending_ = true; ++transfers_; pixelBytes_ += width * rows * 2;
    }
    esp_lcd_panel_io_handle_t io_ = nullptr;
    SemaphoreHandle_t done_ = nullptr;
    uint16_t *buffers_[2] = {};
    uint32_t transfers_ = 0, pixelBytes_ = 0;
    uint8_t next_ = 0;
    bool pending_ = false;
};
} // namespace monitor
#endif
