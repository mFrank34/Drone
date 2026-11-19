// mix.h
#pragma once
#include <Arduino.h>  // uint8_t
#include "shared.h"   // CORNER_COUNT, CORNER_FL...
#include "attitude.h" // Correction

namespace mix
{
    void stabilised(float base, Correction c, float outs[CORNER_COUNT]);
    void all(float base, float outs[CORNER_COUNT]);
    void single(float base, uint8_t sel, float outs[CORNER_COUNT]);
}