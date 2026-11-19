#pragma once
#include <Arduino.h>
#include "shared.h"

inline void Status(uint32_t periodMs = 1000)
{
    static uint32_t last = 0;
    if (millis() - last < periodMs)
        return;
    last = millis();

    Telemetry t;
    portENTER_CRITICAL(&teleMux);
    t = tele;
    portEXIT_CRITICAL(&teleMux);

    Serial.printf("armed %d %s | roll %.1f pitch %.1f | rate %lu period max %lu us\n",
                  t.armed, t.fault, t.roll, t.pitch,
                  (unsigned long)t.rate, (unsigned long)t.periodMaxUs);
}