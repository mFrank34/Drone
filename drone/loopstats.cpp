#include "loopstats.h"

float LoopStats::begin(uint32_t nowUs)
{
    workStart_ = nowUs;
    const uint32_t period = nowUs - lastStart_;
    lastStart_ = nowUs;
    if (iters_ > 0 && period > periodMax_)
        periodMax_ = period;
    return fminf(period * 1e-6f, 0.05f);
}

void LoopStats::end(uint32_t nowUs)
{
    const uint32_t work = nowUs - workStart_;
    if (work > workMax_)
        workMax_ = work;
    iters_++;
}

void LoopStats::noteError()
{
    errs_++;
}

bool LoopStats::windowDue(uint32_t nowMs) const
{
    return nowMs - windowStart_ >= windowMs_;
}

uint32_t LoopStats::workMaxUs() const
{
    return workMax_;
}

uint32_t LoopStats::periodMaxUs() const
{
    return periodMax_;
}

uint32_t LoopStats::rate() const
{
    return iters_ * 1000 / windowMs_;
}

uint32_t LoopStats::errors() const
{
    return errs_;
}

void LoopStats::resetWindow(uint32_t nowMs)
{
    windowStart_ = nowMs;
    workMax_ = periodMax_ = iters_ = errs_ = 0;
}