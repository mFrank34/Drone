#include <Arduino.h>
#include "imu.h"

Imu imu;

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("calibrating, keep flat and still...");
  if (!imu.begin())
  {
    Serial.println("IMU failed");
    while (true)
      delay(1000);
  }
  Serial.println("ready");
}

void loop()
{
  static uint32_t last = micros();
  uint32_t now = micros();
  float dt = fminf((now - last) * 1e-6f, 0.05f);
  last = now;

  if (!imu.update(dt))
    Serial.println("I2C error");

  static uint32_t print = 0;
  if (millis() - print > 100)
  {
    print = millis();
    Serial.printf("roll %6.1f  pitch %6.1f  gx %7.1f  gy %7.1f\n",
                  imu.roll, imu.pitch, imu.gx, imu.gy);
  }
  delay(2);
}