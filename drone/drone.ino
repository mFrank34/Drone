#include <Arduino.h>
#include "imu.h"
#include "quad.h"

Imu imu;
Quad quad;

// ---- telemetry shared between the control task and loop() ----
struct Telemetry
{
  float roll = 0, pitch = 0, gx = 0, gy = 0;
  float out[CORNER_COUNT] = {0, 0, 0, 0}; // add this
  uint32_t workMaxUs = 0;
  uint32_t periodMaxUs = 0;
  uint32_t iters = 0;
  uint32_t i2cErrors = 0;
};

static Telemetry tele;
static portMUX_TYPE teleMux = portMUX_INITIALIZER_UNLOCKED;

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

    if (!imu.update(dt))
      errs++;

    quad.updateSlew(dt);
    float base = quad.throttle(); // 0 until armed + target set
    float r = cfg::ROLL_SIGN * (cfg::KP * imu.roll + cfg::KD * imu.gxf);
    float p = cfg::PITCH_SIGN * (cfg::KP * imu.pitch + cfg::KD * imu.gyf);

    float outs[CORNER_COUNT];
    outs[CORNER_FL] = base - r + p;
    outs[CORNER_FR] = base + r + p;
    outs[CORNER_BL] = base - r - p;
    outs[CORNER_BR] = base + r - p;

    if (fabsf(imu.roll) > cfg::MAX_TILT || fabsf(imu.pitch) > cfg::MAX_TILT)
      quad.cut("tilt cutoff");
    quad.write(outs);

    uint32_t work = micros() - start;
    if (work > workMax)
      workMax = work;
    iters++;

    // publish once per second, then reset the window
    if (millis() - windowStart >= 1000)
    {
      windowStart = millis();
      portENTER_CRITICAL(&teleMux);
      tele.roll = imu.roll;
      tele.pitch = imu.pitch;
      tele.gx = imu.gx;
      tele.gy = imu.gy;
      tele.workMaxUs = workMax;
      tele.periodMaxUs = periodMax;
      tele.iters = iters;
      tele.i2cErrors += errs;
      portEXIT_CRITICAL(&teleMux);
      workMax = periodMax = iters = errs = 0;
      for (uint8_t i = 0; i < CORNER_COUNT; i++)
        tele.out[i] = quad.last((Corner)i);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  quad.begin(); // gates low, PWM attached, disarmed
  quad.arm();
  quad.setTarget(0.3f);

  Serial.println("calibrating, keep flat and still...");
  if (!imu.begin())
  {
    Serial.println("IMU failed");
    while (true)
      delay(1000);
  }
  Serial.println("ready (motors armed...)");

  // The control task is the only thing that touches I2C and the motors from here on.
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 20, nullptr, 1);
}

void loop()
{
  Telemetry t;
  portENTER_CRITICAL(&teleMux);
  t = tele;
  portEXIT_CRITICAL(&teleMux);

  Serial.printf("roll %6.1f pitch %6.1f | FL %3.0f%% FR %3.0f%% BL %3.0f%% BR %3.0f%% | rate %lu Hz work %lu us period %lu us i2c %lu\n",
                t.roll, t.pitch,
                t.out[CORNER_FL] * 100, t.out[CORNER_FR] * 100,
                t.out[CORNER_BL] * 100, t.out[CORNER_BR] * 100,
                (unsigned long)t.iters, (unsigned long)t.workMaxUs,
                (unsigned long)t.periodMaxUs, (unsigned long)t.i2cErrors);
  Serial.printf("roll %6.1f  pitch %6.1f\n", imu.roll, imu.pitch);
  delay(1000);
}