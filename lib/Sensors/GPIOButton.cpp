#include "GPIOButton.h"
#include <Arduino.h>

    /**
     * @brief Constructor for the GPIOButton class.
     *
     * This constructor initializes the button with a pin number and sets the initial state.
     * @param pin The pin number where the button is connected.
     */
    GPIOButton::GPIOButton(int pin)
    {
        _pin = pin;
        _lastState = HIGH;
        _currentState = HIGH;
        _lastDebounceTime = 0;
        _debounceDelay = 50; // Default debounce delay
    }

    /**
     * @brief Register the button pin as an input with pull-up resistor.
     */
    void GPIOButton::begin()
    {
        pinMode(_pin, INPUT_PULLUP);
    };

    /**
     * @brief Update the button state and call the callback if pressed.
     */
    void GPIOButton::update()
    {
        int reading = digitalRead(_pin);
        if (reading != _lastState)
        {
            if (reading == LOW && _startCallback)
            {
                _startCallback();
            }
            else if (reading == HIGH && _endCallback)
            {
                _endCallback();

            }
            _lastDebounceTime = millis();
            _lastState = reading;
        }
        if ((millis() - _lastDebounceTime) > _debounceDelay)
        {
            if (_currentState != reading)
            {
                _currentState = reading;
                if (_currentState == LOW && _callback)
                {
                    _callback();
                }
            }
        }
    };

    /**
     * @brief Set the callback function to be called when the button is pressed.
     */
    void GPIOButton::onEvent(std::function<void()> callback)
    {
        _callback = callback;
    };


    /**
     * @brief Set the callback function to be called when the button is pressed.
     */
    void GPIOButton::onEventStart(std::function<void()> callback)
    {
        _startCallback = callback;
    };


    /**
     * @brief Set the callback function to be called when the button is pressed.
     */
    void GPIOButton::onEventEnd(std::function<void()> callback)
    {
        _endCallback = callback;
    };

    /**
     * @brief Set the debounce delay for the button.
     */
    void GPIOButton::setDebounceDelay(unsigned long time)
    {
        _debounceDelay = time;
    };