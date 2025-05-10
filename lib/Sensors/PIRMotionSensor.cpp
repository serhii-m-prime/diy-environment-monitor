#include <PIRMotionSensor.h>
#include <Arduino.h>


/**
 * @brief Constructor for the PIRMotionSensor class.
 * 
 * This constructor initializes the PIR motion sensor with a pin number and sets the initial state.
 * @param pin The pin number where the PIR sensor is connected.
 */
PIRMotionSensor::PIRMotionSensor(int pin)
{
    _pin = pin;
    _lastState = LOW;
    _currentState = LOW;
    _lastDebounceTime = 0;
    _debounceDelay = 50; // Default debounce delay
}

/**
 * @brief Register the PIR sensor pin as an input.
 * 
 * This function sets the pin mode for the PIR sensor to INPUT.
 */
void PIRMotionSensor::begin()
{
    pinMode(_pin, INPUT);
}

/**
 * @brief Update the PIR sensor state and call the callback if motion is detected.
 * 
 * This function checks the current state of the PIR sensor and calls the appropriate callback
 * functions based on the state changes.
 */
void PIRMotionSensor::update()
{
    int reading = digitalRead(_pin);
    if (reading != _lastState)
    {
        if (reading == HIGH && _startCallback)
        {
            _startCallback();
        }
        else if (reading == LOW && _endCallback)
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
            if (_currentState == HIGH && _callback)
            {
                _callback();
            }
        }
    }
}

/**
 * @brief Set the callback function to be called when motion is detected.
 * 
 * This function sets the callback function to be called when motion is detected.
 * @param callback The callback function to be called.
 */
void PIRMotionSensor::onEvent(std::function<void()> callback)
{
    _callback = callback;
}

/**
 * * @brief Set the callback function to be called when motion starts.
 */
void PIRMotionSensor::onEventStart(std::function<void()> callback)
{
    _startCallback = callback;
}

/**
 * @brief Set the callback function to be called when motion ends.
 * 
 * This function sets the callback function to be called when motion ends.
 * @param callback The callback function to be called.
 */
void PIRMotionSensor::onEventEnd(std::function<void()> callback)
{
    _endCallback = callback;
}

/**
 * @brief Set the debounce delay for the PIR sensor.
 * 
 * This function sets the debounce delay for the PIR sensor to avoid false triggers.
 * @param time The debounce delay in milliseconds.
 */
void PIRMotionSensor::setDebounceDelay(unsigned long time)
{
    _debounceDelay = time;
}