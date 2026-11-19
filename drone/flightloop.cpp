#include "flightloop.h"
#include "mix.h"

void FlightLoop::start(uint8_t core, uint8_t priority)
{
    xTaskCreatePinnedToCore(taskEntry, "flight", 4096, this, priority, nullptr, core);
}

void FlightLoop::taskEntry(void *self)
{
    static_cast<FlightLoop *>(self)->run();
}

void FlightLoop::run()
{
    TickType_t wake = xTaskGetTickCount();
    stats_ = LoopStats(); // restart timers now, not at global construction time

    for (;;)
    {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::LOOP_MS));
        const float dt = stats_.begin(micros());

        handleCommands();

        const bool imuOk = imu_.update(dt);
        if (!imuOk)
            stats_.noteError();

        quad_.setTarget(cmd.thr);
        quad_.updateSlew(dt);

        float outs[CORNER_COUNT];
        computeOutputs(quad_.throttle(), dt, imuOk, outs);
        quad_.write(outs);

        stats_.end(micros());

        if (stats_.windowDue(millis()))
        {
            publishTelemetry();
            stats_.resetWindow(millis());
        }
    }
}

void FlightLoop::handleCommands()
{
    if (cmd.stop.exchange(false))
    {
        quad_.cut("STOP");
        pid_.reset();
    }
    if (cmd.arm.exchange(false) && !quad_.isArmed())
    {
        quad_.arm();
        pid_.reset();
    }
}

void FlightLoop::computeOutputs(float base, float dt, bool imuOk, float outs[CORNER_COUNT])
{
    for (uint8_t i = 0; i < CORNER_COUNT; i++)
        outs[i] = 0;

    const uint8_t sel = cmd.sel;
    const bool flying = quad_.isArmed() && base > 0.02f;
    const bool stabilise = flying && sel == 0 && cmd.stab && imuOk;

    if (!stabilise)
        pid_.reset();
    if (!flying)
        return;

    if (stabilise)
    {
        Attitude a{imu_.roll, imu_.pitch, imu_.gxf, imu_.gyf};
        mix::stabilised(base, pid_.update(a, dt), outs);
    }
    else if (sel == 0)
        mix::all(base, outs);
    else
        mix::single(base, sel, outs);
}

void FlightLoop::publishTelemetry()
{
    portENTER_CRITICAL(&teleMux);
    tele.roll = imu_.roll;
    tele.pitch = imu_.pitch;
    tele.gx = imu_.gx;
    tele.gy = imu_.gy;
    tele.thr = quad_.throttle();
    tele.armed = quad_.isArmed();
    tele.fault = quad_.fault();
    for (uint8_t i = 0; i < CORNER_COUNT; i++)
        tele.out[i] = quad_.last((Corner)i);
    tele.workMaxUs = stats_.workMaxUs();
    tele.periodMaxUs = stats_.periodMaxUs();
    tele.rate = stats_.rate();
    tele.i2cErrors += stats_.errors();
    portEXIT_CRITICAL(&teleMux);
}