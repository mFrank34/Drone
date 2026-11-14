// motor.h
#pragma once
#include <Arduino.h>
#include "config.h"

enum MotorId : uint8_t
{
    FL = 0,
    FR = 1,
    BL = 2,
    BR = 3,
    MOTOR_COUNT = 4
}