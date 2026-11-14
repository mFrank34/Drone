// config.h

#pragma once
#include <stdint.h>

namespace cfg
{
    /* Motor Pins*/
    constexpr uint8_t MOTOR_FL = 3; // GPIO3 is a strapping pin on the S3
    constexpr uint8_t MOTOR_FR = 11;
    constexpr uint8_t MOTOR_BL = 6;
    constexpr uint8_t MOTOR_BR = 8;

    /* IC2 MTU Sensor */
    constexpr uint8_t SDA_PIN = 12;
    constexpr uint8_t SCL_PIN = 13;
    constexpr uint8_t MPU_ADDR = 0x68;
    constexpr float GX_SIGN = 1.0f;
    constexpr float GY_SIGN = 1.0f;

    /* status LED */
    constexpr uint8_t LED_PIN = 21;

    /* motor config*/
    constexpr uint32_t PWM_FREQ = 20000;
    constexpr uint8_t PWM_BITS = 10; // 1024 steps at 20 kHz
    constexpr float MAX_OUT = 1.0f;
    constexpr float SLEW = 1.0f; // throttle change per second

    /* Safety Shut Off  */
    constexpr uint32_t FAILSAFE_MS = 1000;
    constexpr float MAX_TILT = 60.0f; // degrees

    /* Stablizers */
    constexpr float KP = 0.02f;
    constexpr float KD = 0.004f;
    constexpr float ALPHA = 0.98f;
    constexpr float GYRO_SMOOTH = 0.6f;

    /* Pin check for making sure pins are unique before compiling */
    constexpr bool pinsUnique()
    {
        const uint8_t p[] = {MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR, SDA_PIN, SCL_PIN, LED_PIN};
        for (unsigned i = 0; i < sizeof(p); i++)
            for (unsigned j = i + 1; j < sizeof(p); j++)
                if (p[i] == p[j])
                    return false;
        return true;
    }
    static_assert(pinsUnique(), "pin conflict!");
}