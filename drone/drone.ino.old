#include <Arduino.h>
#include "imu.h"
#include "quad.h"
#include "shared.h"

Imu imu;
Quad quad;

// Integral of tilt error (deg*s). Only runs while flying with the stabiliser on.
static float iRoll = 0, iPitch = 0;

static void resetIntegral() { iRoll = iPitch = 0; }

// Fills outs[] for one control step. base = slewed throttle (0..1).
static void mix(float base, float dt, bool imuOk, float outs[CORNER_COUNT])
{
  for (uint8_t i = 0; i < CORNER_COUNT; i++)
    outs[i] = 0;

  const uint8_t sel = cmd.sel; // 0 = ALL, 1..4 = FL FR BL BR
  const bool flying = quad.isArmed() && base > 0.02f;
  const bool stabilise = flying && sel == 0 && cmd.stab && imuOk;

  if (!stabilise)
    resetIntegral(); // no stale integral when it starts stabilising again

  if (!flying)
    return;

  if (stabilise)
  {
    const float lim = cfg::I_LIMIT / cfg::KI; // anti-windup clamp
    iRoll = constrain(iRoll + imu.roll * dt, -lim, lim);
    iPitch = constrain(iPitch + imu.pitch * dt, -lim, lim);

    const float r = cfg::ROLL_SIGN * (cfg::KP * imu.roll + cfg::KD * imu.gxf + cfg::KI * iRoll);
    const float p = cfg::PITCH_SIGN * (cfg::KP * imu.pitch + cfg::KD * imu.gyf + cfg::KI * iPitch);

    outs[CORNER_FL] = base - r + p;
    outs[CORNER_FR] = base + r + p;
    outs[CORNER_BL] = base - r - p;
    outs[CORNER_BR] = base + r - p;
  }
  else if (sel == 0)
  {
    for (uint8_t i = 0; i < CORNER_COUNT; i++)
      outs[i] = base; // all motors equal, no stabiliser
  }
  else
  {
    outs[sel - 1] = base; // single-motor test
  }
}

static void controlTask(void *)
{
  TickType_t wake = xTaskGetTickCount();
  uint32_t lastStart = micros();
  uint32_t workMax = 0, periodMax = 0, iters = 0, errs = 0;
  uint32_t windowStart = millis();

  for (;;)
  {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::LOOP_MS));
    const uint32_t start = micros();
    const uint32_t period = start - lastStart;
    const float dt = fminf(period * 1e-6f, 0.05f);
    lastStart = start;
    if (iters > 0 && period > periodMax)
      periodMax = period;

    // ---- commands and failsafes ----
    if (cmd.stop.exchange(false))
    {
      quad.cut("STOP");
      resetIntegral();
    }
    if (cmd.arm.exchange(false) && !quad.isArmed())
    {
      quad.arm();
      resetIntegral();
      cmd.lastSeenMs = millis(); // start the link timer fresh
    }
    if (quad.isArmed() && millis() - cmd.lastSeenMs > cfg::FAILSAFE_MS)
    {
      quad.cut("link lost");
      resetIntegral();
    }

    // ---- sensing ----
    const bool imuOk = imu.update(dt);
    if (!imuOk)
    {
      errs++;
      if (quad.isArmed() && cmd.stab)
      {
        quad.cut("sensor error");
        resetIntegral();
      }
    }
    if (quad.isArmed() && cmd.stab &&
        (fabsf(imu.roll) > cfg::MAX_TILT || fabsf(imu.pitch) > cfg::MAX_TILT))
    {
      quad.cut("tilt cutoff");
      resetIntegral();
    }

    // ---- throttle, mixer, motors ----
    quad.setTarget(cmd.thr);
    quad.updateSlew(dt);

    float outs[CORNER_COUNT];
    mix(quad.throttle(), dt, imuOk, outs);
    quad.write(outs); // forces zero if disarmed

    const uint32_t work = micros() - start;
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

// ---- USB serial commands (replaces the MAVLink/BLE link) ----
// a = arm, s = stop, b = toggle stabiliser, m<0-4> = motor select,
// t<0-100> = throttle %. Any valid command refreshes the failsafe timer.
static void handleSerial()
{
  static char buf[16];
  static uint8_t n = 0;

  while (Serial.available())
  {
    char c = Serial.read();
    if (c != '\n' && c != '\r')
    {
      if (n < sizeof(buf) - 1)
        buf[n++] = c;
      continue;
    }
    if (n == 0)
      continue;
    buf[n] = 0;
    n = 0;

    switch (buf[0])
    {
    case 'a':
      cmd.arm = true;
      break;
    case 's':
      cmd.stop = true;
      cmd.thr = 0;
      break;
    case 'b':
      cmd.stab = !cmd.stab;
      Serial.printf("stab %d\n", (int)cmd.stab);
      break;
    case 'm':
      cmd.sel = constrain(atoi(buf + 1), 0, 4);
      break;
    case 't':
      cmd.thr = constrain(atoi(buf + 1), 0, 100) / 100.0f;
      break;
    default:
      Serial.println("?");
      continue;
    }
    cmd.lastSeenMs = millis(); // any valid command counts as "link alive"
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  quad.begin(); // gates low, disarmed

  Serial.println("calibrating, keep flat and still...");
  bool imuOk = imu.begin();
  if (!imuOk)
    Serial.println("IMU failed - stabiliser unavailable");

  cmd.lastSeenMs = millis();
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 20, nullptr, 1);
  Serial.println("ready (motors disarmed)");
  Serial.println("cmds: a=arm  s=stop  b=stab on/off  m<0-4>=motor  t<0-100>=throttle %");
}

void loop()
{
  static uint32_t lastPrint = 0;
  handleSerial();

  if (millis() - lastPrint >= 1000)
  {
    lastPrint = millis();
    Telemetry t;
    portENTER_CRITICAL(&teleMux);
    t = tele;
    portEXIT_CRITICAL(&teleMux);
    Serial.printf("armed %d %s | roll %.1f pitch %.1f | rate %lu period max %lu us\n",
                  t.armed, t.fault, t.roll, t.pitch,
                  (unsigned long)t.rate, (unsigned long)t.periodMaxUs);
  }
  delay(5);
}