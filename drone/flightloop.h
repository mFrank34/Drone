#pragma once
#include <Arduino.h>
#include "imu.h"
#include "quad.h"
#include "attitude.h"
#include "loopstats.h"
#include "shared.h"

class FlightLoop
{
public:
    FlightLoop(Imu &imu, Quad &quad) : imu_(imu), quad_(quad) {}

    // creates the FreeRTOS task
    void start(uint8_t core = 1, uint8_t priority = 20);

private:
    static void taskEntry(void *self); // FreeRTOS needs a plain function
    void run();                        // the actual loop

    void handleCommands();
    void computeOutputs(float base, float dt, bool imuOk, float outs[CORNER_COUNT]);
    void publishTelemetry();

    Imu &imu_;
    Quad &quad_;
    AttitudePid pid_;
    LoopStats stats_;
};