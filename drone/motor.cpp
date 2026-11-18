#include "motor.h"

Motor::Motor(uint8_t pin) : _pin(pin) {}

void Motor::begin()
{
  static uint8_t nextChannel = 0; // shared across all Motor objects
  _channel = nextChannel++;       // 0..7 for 4 motors is fine

  ledcSetup(_channel, cfg::PWM_FREQ, cfg::PWM_BITS);
  ledcAttachPin(_pin, _channel);
}

void Motor::write(float v)
{
  if (v < 0.0f)
    v = 0.0f;
  if (v > cfg::MAX_OUT)
    v = cfg::MAX_OUT;
  _last = v;
  ledcWrite(_channel, (uint32_t)(v * ((1u << cfg::PWM_BITS) - 1)));
}

void Motor::off() { write(0.0f); }