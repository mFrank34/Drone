#pragma once
#include <Arduino.h>
#include "config.h"

// One PWM channel driving one MOSFET gate. Knows nothing about arming or faults.
class Motor
{
public:
    explicit Motor(uint8_t pin);

    void begin();        // attach PWM and start at zero
    void write(float v); // 0..1, clamped to cfg::MAX_OUT
    void off();

    uint8_t pin() const { return _pin; }
    float last() const { return _last; }

private:
    uint8_t _pin;
    uint8_t _channel = 0;
    float _last = 0.0f;
};
