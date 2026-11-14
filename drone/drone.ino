#include <Arduino.h>
#include "imu.h"
#include "quad.h"
#include "shared.h"
#include "web.h"

Imu imu;
Quad quad;

static void controlTask(void *)
{
  TickType_t wake = xTaskGetTickCount();
  uint32_t lastStart = micros();
  uint32_t workMax = 0, periodMax = 0, iters = 0, errs = 0;
  uint32_t windowStart = millis();

  for (;;)
  {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::LOOP_MS));
    uint32_t start = micros();
    uint32_t period = start - lastStart;
    float dt = fminf(period * 1e-6f, 0.05f);
    lastStart = start;
    if (iters > 0 && period > periodMax)
      periodMax = period;

    // ---- commands and failsafes ----
    if (cmd.stop.exchange(false))
      quad.cut("STOP");
    if (cmd.arm.exchange(false) && !quad.isArmed())
    {
      quad.arm();
      cmd.lastSeenMs = millis(); // start the link timer fresh
    }
    if (quad.isArmed() && millis() - cmd.lastSeenMs > cfg::FAILSAFE_MS)
      quad.cut("link lost");

    // ---- sensing ----
    bool imuOk = imu.update(dt);
    if (!imuOk)
    {
      errs++;
      if (quad.isArmed() && cmd.stab)
        quad.cut("sensor error");
    }
    if (quad.isArmed() && cmd.stab &&
        (fabsf(imu.roll) > cfg::MAX_TILT || fabsf(imu.pitch) > cfg::MAX_TILT))
      quad.cut("tilt cutoff");

    // ---- throttle and mixer ----
    quad.setTarget(cmd.thr);
    quad.updateSlew(dt);
    float base = quad.throttle();
    float outs[CORNER_COUNT] = {0, 0, 0, 0};
    uint8_t sel = cmd.sel;

    if (quad.isArmed() && base > 0.02f)
    {
      if (sel == 0 && cmd.stab && imuOk)
      {
        float r = cfg::ROLL_SIGN * (cfg::KP * imu.roll + cfg::KD * imu.gxf);
        float p = cfg::PITCH_SIGN * (cfg::KP * imu.pitch + cfg::KD * imu.gyf);
        outs[CORNER_FL] = base - r + p;
        outs[CORNER_FR] = base + r + p;
        outs[CORNER_BL] = base - r - p;
        outs[CORNER_BR] = base + r - p;
      }
      else if (sel == 0)
      {
        for (auto &o : outs)
          o = base;
      }
      else
      {
        outs[sel - 1] = base; // 1..4 = FL FR BL BR
      }
    }
    quad.write(outs);

    uint32_t work = micros() - start;
    if (work > workMax)
      workMax = work;
    iters++;

    // ---- publish telemetry every 100 ms ----
    if (millis() - windowStart >= 100)
    {
      windowStart = millis();
      portENTER_CRITICAL(&teleMux);
      tele.roll = imu.roll;
      tele.pitch = imu.pitch;
      tele.gx = imu.gx;
      tele.gy = imu.gy;
      tele.thr = quad.throttle();
      tele.armed = quad.isArmed();
      tele.fault = quad.fault();
      for (uint8_t i = 0; i < CORNER_COUNT; i++)
        tele.out[i] = quad.last((Corner)i);
      tele.workMaxUs = workMax;
      tele.periodMaxUs = periodMax;
      tele.rate = iters * 10;
      tele.i2cErrors += errs;
      portEXIT_CRITICAL(&teleMux);
      workMax = periodMax = iters = errs = 0;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  quad.begin(); // gates low, disarmed
  Serial.println("calibrating, keep flat and still...");
  if (!imu.begin())
  {
    Serial.println("IMU failed");
    while (true)
      delay(1000);
  }
  cmd.lastSeenMs = millis();
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 20, nullptr, 1);
  webBegin();
  Serial.println("ready (motors disarmed)");
}

void loop()
{
  Telemetry t;
  portENTER_CRITICAL(&teleMux);
  t = tele;
  portEXIT_CRITICAL(&teleMux);
  Serial.printf("armed %d %s | roll %.1f pitch %.1f | rate %lu period max %lu us\n",
                t.armed, t.fault, t.roll, t.pitch,
                (unsigned long)t.rate, (unsigned long)t.periodMaxUs);
  delay(1000);
} 