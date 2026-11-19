#pragma once
#include <Arduino.h>

class Commands
{
public:
    void poll();

private:
    void handleLine(const char *line);

    char buf_[16];
    uint8_t n_ = 0;
};