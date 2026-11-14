// imu.h
#pragma once
#include <Arduino.h>
#include "config.h"

class Imu
{
public:
    bool begin();          // init + calibrate; false if sensor missing or moved
    bool update(float dt); // read + filter; false on I2C error

    float roll = 0, pitch = 0;    // degrees
    float gx = 0, gy = 0, gz = 0; // deg/s, bias-corrected
    float gxf = 0, gyf = 0;       // smoothed, for the D term
    float ax = 0, ay = 0, az = 0; // g

private:
    bool readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
    static void accelAngles(float ax, float ay, float az, float &r, float &p);
    bool calibrate(int n = 100, int tries = 3);

    float _roll0 = 0, _pitch0 = 0, _gx0 = 0, _gy0 = 0, _gz0 = 0;
};