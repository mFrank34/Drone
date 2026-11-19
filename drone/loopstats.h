#pragma once
#include <Arduino.h>

class LoopStats
{
public:
    explicit LoopStats(uint32_t windowMs = 100) : windowMs_(windowMs) {}

    float begin(uint32_t nowUs);

    void end(uint32_t nowUs);

    void noteError();

    bool windowDue(uint32_t nowMs) const;

    uint32_t workMaxUs() const;
    uint32_t periodMaxUs() const;
    uint32_t rate() const;
    uint32_t errors() const;

    void resetWindow(uint32_t nowMs);

private:
    uint32_t windowMs_;
    uint32_t lastStart_ = micros(), workStart_ = 0, windowStart_ = millis();
    uint32_t workMax_ = 0, periodMax_ = 0, iters_ = 0, errs_ = 0;
};