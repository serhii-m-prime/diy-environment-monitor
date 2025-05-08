#pragma once

#include "IEventSensor.h"
#include <Arduino.h>

class GPIOButton : public IEventSensor
{
public:
    GPIOButton(int pin);
    void begin() override;
    void update() override;
    void onEvent(std::function<void()> callback) override;
    void onEventStart(std::function<void()> callback) override;
    void onEventEnd(std::function<void()> callback) override;
    void setDebounceDelay(unsigned long time) override;

private:
    uint8_t _pin;
    std::function<void()> _callback;
    std::function<void()> _startCallback;
    std::function<void()> _endCallback;
    int _lastState;
    int _currentState;
    unsigned long _lastDebounceTime;
    unsigned long _debounceDelay;
};
