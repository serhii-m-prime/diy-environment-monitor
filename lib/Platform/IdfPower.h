#pragma once
#ifdef ESP_PLATFORM
#include <esp_pm.h>
#include <esp_sleep.h>
#include <esp_check.h>
#include <driver/gpio.h>
#include <sdkconfig.h>

namespace monitor {
class PowerRuntime {
public:
    void begin() {
#if CONFIG_MONITOR_POWER_MANAGEMENT
        esp_pm_config_t config{};
        config.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
        config.min_freq_mhz = CONFIG_MONITOR_MIN_CPU_MHZ;
#if CONFIG_MONITOR_AUTO_LIGHT_SLEEP
        config.light_sleep_enable = true;
#endif
        ESP_ERROR_CHECK(esp_pm_configure(&config));
        ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "display", &display_));
        ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "fan", &fan_));
        ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "sensor_io", &io_));
        ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "render", &render_));
#endif
    }
    void display(bool on) { change(display_, displayHeld_, on); }
    void fan(bool on) { change(fan_, fanHeld_, on); }
    void sensorIo(bool on) { set(io_, on); }
    void rendering(bool on) { set(render_, on); }
    void configureInputWake(int left, int right, int pir) {
#if CONFIG_MONITOR_POWER_MANAGEMENT && CONFIG_MONITOR_AUTO_LIGHT_SLEEP
        for (int pin : {left, right, pir}) {
            ESP_ERROR_CHECK(gpio_sleep_sel_dis(static_cast<gpio_num_t>(pin)));
            inputWake(pin, gpio_get_level(static_cast<gpio_num_t>(pin)) != 0);
        }
        ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
#else
        (void)left; (void)right; (void)pir;
#endif
    }
    void inputWake(int pin, bool high) {
#if CONFIG_MONITOR_POWER_MANAGEMENT && CONFIG_MONITOR_AUTO_LIGHT_SLEEP
        const auto gpio = static_cast<gpio_num_t>(pin);
        // Sleep on the opposite level: held-low buttons or held-high PIR never
        // cause a wake loop, and their release can still wake the debouncer.
        ESP_ERROR_CHECK(gpio_wakeup_enable(gpio, high ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL));
#else
        (void)pin; (void)high;
#endif
    }
private:
    static void set(esp_pm_lock_handle_t lock, bool on) {
        if (lock) ESP_ERROR_CHECK(on ? esp_pm_lock_acquire(lock) : esp_pm_lock_release(lock));
    }
    static void change(esp_pm_lock_handle_t lock, bool &held, bool on) {
        if (on != held) { set(lock, on); held = on; }
    }
    esp_pm_lock_handle_t display_ = nullptr, fan_ = nullptr, io_ = nullptr, render_ = nullptr;
    bool displayHeld_ = false, fanHeld_ = false;
};
class PowerGuard {
public:
    enum class Use { SensorIo, Render };
    PowerGuard(PowerRuntime &runtime, Use use) : runtime_(runtime), use_(use) { set(true); }
    ~PowerGuard() { set(false); }
    PowerGuard(const PowerGuard &) = delete;
    PowerGuard &operator=(const PowerGuard &) = delete;
private:
    void set(bool on) { if (use_ == Use::SensorIo) runtime_.sensorIo(on); else runtime_.rendering(on); }
    PowerRuntime &runtime_;
    Use use_;
};
} // namespace monitor
#endif
