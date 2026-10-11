#pragma once
#include <Arduino.h>

#ifdef ESP_PLATFORM
#include <driver/ledc.h>
#include <esp_check.h>
#include <sdkconfig.h>
#endif

namespace monitor {
inline void configurePwm(int backlightPin, int fanPin) {
#ifdef ESP_PLATFORM
    ledc_timer_config_t timer{};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.timer_num = LEDC_TIMER_0;
    timer.duty_resolution = LEDC_TIMER_12_BIT;
    timer.freq_hz = 5000;
    timer.clk_cfg = LEDC_USE_XTAL_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    timer.timer_num = LEDC_TIMER_1;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t channel{};
    channel.speed_mode = LEDC_LOW_SPEED_MODE;
    channel.channel = LEDC_CHANNEL_0;
    channel.timer_sel = LEDC_TIMER_0;
    channel.gpio_num = backlightPin;
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    channel.channel = LEDC_CHANNEL_1;
    channel.timer_sel = LEDC_TIMER_1;
    channel.gpio_num = fanPin;
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
#else
    ledcSetup(0, 5000, 12); ledcAttachPin(backlightPin, 0); ledcWrite(0, 0);
    ledcSetup(1, 5000, 8); ledcAttachPin(fanPin, 1); ledcWrite(1, 0);
#endif
}
inline void writePwm(int channel, uint16_t duty) {
#ifdef ESP_PLATFORM
    const auto id = static_cast<ledc_channel_t>(channel);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, id, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, id));
#else
    ledcWrite(channel, duty);
#endif
}
} // namespace monitor
