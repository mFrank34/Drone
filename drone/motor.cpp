#include "motor.h"

Motor::Motor(uint8_t pin) : _pin(pin) {}

void Motor::begin() {
  if (!ledcAttach(_pin, cfg::PWM_FREQ, cfg::PWM_BITS)) {
    Serial.printf("PWM attach failed on pin %u\n", _pin);
  }
  off();
}

void Motor::write(float v)
{
    if (v < 0.0f)
        v = 0.0f;
    if (v > cfg::MAX_OUT)
        v = cfg::MAX_OUT;
    _last = v;
    ledcWrite(_pin, (uint32_t)(v * ((1u << cfg::PWM_BITS) - 1)));
}

void Motor::off() { write(0.0f); }