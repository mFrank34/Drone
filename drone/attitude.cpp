// attitude.cpp
#include "attitude.h"
#include "shared.h" // cfg::

Correction AttitudePid::update(const Attitude &a, float dt)
{
    const float lim = cfg::I_LIMIT / cfg::KI; // anti-windup clamp

    iRoll_ = constrain(iRoll_ + a.roll * dt, -lim, lim);
    iPitch_ = constrain(iPitch_ + a.pitch * dt, -lim, lim);

    Correction c;
    c.roll = cfg::ROLL_SIGN * (cfg::KP * a.roll + cfg::KD * a.gxf + cfg::KI * iRoll_);
    c.pitch = cfg::PITCH_SIGN * (cfg::KP * a.pitch + cfg::KD * a.gyf + cfg::KI * iPitch_);
    return c;
}

void AttitudePid::reset()
{
    iRoll_ = iPitch_ = 0;
}