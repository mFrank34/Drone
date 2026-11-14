#pragma once
#include <Arduino.h>
#include <atomic>
#include "quad.h"

struct Cmd
{
    std::atomic<bool> stop{false}, arm{false}, stab{false};
    std::atomic<float> thr{0.0f};
    std::atomic<uint8_t> sel{0};         // 0 = ALL, 1..4 = FL FR BL BR
    std::atomic<uint32_t> lastSeenMs{0}; // any web request refreshes this
};

struct Telemetry
{
    float roll = 0, pitch = 0, gx = 0, gy = 0, thr = 0;
    float out[CORNER_COUNT] = {0, 0, 0, 0};
    bool armed = false;
    const char *fault = ""; // always a string literal
    uint32_t workMaxUs = 0, periodMaxUs = 0, rate = 0, i2cErrors = 0;
};

inline Cmd cmd;
inline Telemetry tele;
inline portMUX_TYPE teleMux = portMUX_INITIALIZER_UNLOCKED;