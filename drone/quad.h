#pragma once
#include "motor.h"

enum Corner : uint8_t
{
    MOTOR_FL = 0,
    MOTOR_FR = 1,
    MOTOR_BL = 2,
    MOTOR_BR = 3,
    CORNER_COUNT = 4
};

// The group: owns the four motors and the one copy of the safety state.
class Quad
{
public:
    Quad();

    void begin(); // all gates low first, then PWM

    // Output (always forced to zero while disarmed)
    void write(const float outs[CORNER_COUNT]);
    void writeOne(Corner m, float v);
    void allOff();

    // Safety
    void arm();
    void cut(const char *reason);
    bool isArmed() const { return _armed; }
    const char *fault() const { return _fault; }

    // Throttle with slew limiting
    void setTarget(float t);
    void updateSlew(float dt);
    float throttle() const { return _cur; }

    float last(Corner m) const { return _m[m].last(); }

private:
    Motor _m[CORNER_COUNT];
    volatile bool _armed = false;
    float _target = 0.0f;
    float _cur = 0.0f;
    const char *_fault = "";
};