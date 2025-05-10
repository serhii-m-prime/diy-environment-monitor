#pragma once

#include <Arduino.h>

class IEventSensor
{
public:
    virtual void begin() = 0;
    virtual void update() = 0;
    virtual void onEvent(std::function<void()> callback) = 0;
    virtual void onEventStart(std::function<void()> callback) = 0;
    virtual void onEventEnd(std::function<void()> callback) = 0;
    virtual void setDebounceDelay(unsigned long time) = 0;
    virtual ~IEventSensor() = default;
};