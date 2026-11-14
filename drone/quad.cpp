#include "quad.h"

Quad::Quad()
    : _m{Motor(cfg::MOTOR_FL), Motor(cfg::MOTOR_FR),
         Motor(cfg::MOTOR_BL), Motor(cfg::MOTOR_BR)} {}

void Quad::begin()
{
    // Every gate low before any PWM is attached
    for (auto &m : _m)
    {
        pinMode(m.pin(), OUTPUT);
        digitalWrite(m.pin(), LOW);
    }
    for (auto &m : _m)
        m.begin();
}

void Quad::write(const float outs[CORNER_COUNT])
{
    for (uint8_t i = 0; i < CORNER_COUNT; i++)
        _m[i].write(_armed ? outs[i] : 0.0f);
}

void Quad::writeOne(Corner id, float v)
{
    _m[id].write(_armed ? v : 0.0f);
}

void Quad::allOff()
{
    for (auto &m : _m)
        m.off();
}

void Quad::arm()
{
    _armed = true;
    _target = _cur = 0.0f;
    _fault = "";
}

void Quad::cut(const char *reason)
{
    allOff();
    _armed = false;
    _target = _cur = 0.0f;
    _fault = reason;
    Serial.printf("CUT: %s\n", reason);
}

void Quad::setTarget(float t)
{
    if (!_armed)
        return;
    _target = fminf(fmaxf(t, 0.0f), cfg::MAX_OUT);
}

void Quad::updateSlew(float dt)
{
    if (!_armed)
    {
        _cur = 0.0f;
        return;
    }
    const float step = cfg::SLEW * dt;
    if (_cur < _target)
        _cur = fminf(_target, _cur + step);
    else if (_cur > _target)
        _cur = fmaxf(_target, _cur - step);
}