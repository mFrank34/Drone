// config.h

#pragma once
#include <stdint.h>

#pragma once

#ifndef CONFIG_H
#define CONFIG_H

// MAVLink identity
#define SYSTEM_ID 1
#define COMPONENT_ID 1 // MAV_COMP_ID_AUTOPILOT1

// Bluetooth LE name the controller app will see
#define BLE_DEVICE_NAME "Drone"

namespace cfg
{
    /* Motor Pins*/
    constexpr uint8_t MOTOR_FL = 3; // GPIO3 is a strapping pin on the S3
    constexpr uint8_t MOTOR_FR = 8; // 8
    constexpr uint8_t MOTOR_BL = 6;
    constexpr uint8_t MOTOR_BR = 11; // 11

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

    /* Stabiliser gains */
    constexpr float KP = 0.01f;
    constexpr float KD = 0.002f;
    constexpr float KI = 0.02f;
    constexpr float I_LIMIT = 0.10f; // max correction the I term can add
    constexpr float ALPHA = 0.98f;
    constexpr float GYRO_SMOOTH = 0.6f;

    /* Stabiliser Pushes Back  */
    constexpr float ROLL_SIGN = 1.0f;
    constexpr float PITCH_SIGN = 1.0f;
    constexpr float KP_TEST = 0.02f; // reuse KP/KD from before if you prefer
    constexpr float KD_TEST = 0.004f;

    constexpr uint32_t LOOP_MS = 2; // 500 Hz

    /* MAVLink / BLE controller input */
    constexpr float MAV_THR_CAP = 0.5f;    // full stick = this fraction of throttle (old web slider stopped at 50%)
    constexpr int MAV_Z_MIN = 0;           // MANUAL_CONTROL.z at zero throttle: 0 for apps sending 0..1000, -1000 for apps sending -1000..1000
    constexpr float ARM_MAX_THR = 0.05f;   // refuse to arm if the stick throttle is above this
    constexpr uint32_t ARM_INPUT_MS = 500; // ...or if no stick input arrived within this long
    constexpr uint32_t BLE_PASSKEY = 0;    // 0 = open link; 1..999999 = central must pair with this passkey

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

#endif