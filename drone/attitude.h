// attitude.h
#pragma once
#include <Arduino.h>

struct Attitude
{
    float roll, pitch, gxf, gyf;
};

struct Correction
{
    float roll, pitch;
};

class AttitudePid
{
public:
    Correction update(const Attitude &a, float dt);
    void reset();

private:
    float iRoll_ = 0, iPitch_ = 0;
};