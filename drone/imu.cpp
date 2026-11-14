// imu.cpp
#include "imu.h"
#include <Wire.h>

static bool regWrite(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(cfg::MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Imu::begin()
{
    Wire.begin(cfg::SDA_PIN, cfg::SCL_PIN, 400000);
    if (!regWrite(0x6B, 0x00))
        return false; // wake
    delay(100);
    if (!regWrite(0x1A, 0x03))
        return false; // low-pass (try 1 or 2 later)
    return calibrate();
}

bool Imu::readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
    Wire.beginTransmission(cfg::MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((int)cfg::MPU_ADDR, 14) != 14)
        return false;
    int16_t v[7];
    for (int i = 0; i < 7; i++)
    {
        uint8_t hi = Wire.read(), lo = Wire.read();
        v[i] = (int16_t)((hi << 8) | lo);
    }
    ax = v[0] / 16384.0f;
    ay = v[1] / 16384.0f;
    az = v[2] / 16384.0f;
    // v[3] is temperature, skipped
    gx = v[4] / 131.0f;
    gy = v[5] / 131.0f;
    gz = v[6] / 131.0f;
    return true;
}

void Imu::accelAngles(float ax, float ay, float az, float &r, float &p)
{
    r = atan2f(ay, az) * RAD_TO_DEG;
    p = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

bool Imu::calibrate(int n, int tries)
{
    for (int t = 0; t < tries; t++)
    {
        double sr = 0, sp = 0, sgx = 0, sgy = 0, sgz = 0;
        bool moved = false;
        for (int i = 0; i < n; i++)
        {
            float ax, ay, az, gx, gy, gz;
            if (!readRaw(ax, ay, az, gx, gy, gz))
                return false;
            if (fabsf(gx) > 10 || fabsf(gy) > 10 || fabsf(gz) > 10)
                moved = true;
            float r, p;
            accelAngles(ax, ay, az, r, p);
            sr += r;
            sp += p;
            sgx += gx;
            sgy += gy;
            sgz += gz;
            delay(10);
        }
        _roll0 = sr / n;
        _pitch0 = sp / n;
        _gx0 = sgx / n;
        _gy0 = sgy / n;
        _gz0 = sgz / n;
        if (!moved)
            return true;
        delay(1000);
    }
    return false;
}

bool Imu::update(float dt)
{
    float rax, ray, raz, rgx, rgy, rgz;
    if (!readRaw(rax, ray, raz, rgx, rgy, rgz))
        return false;

    gx = (rgx - _gx0) * cfg::GX_SIGN;
    gy = (rgy - _gy0) * cfg::GY_SIGN;
    gz = rgz - _gz0;
    gxf = cfg::GYRO_SMOOTH * gxf + (1 - cfg::GYRO_SMOOTH) * gx;
    gyf = cfg::GYRO_SMOOTH * gyf + (1 - cfg::GYRO_SMOOTH) * gy;

    float ar, ap;
    accelAngles(rax, ray, raz, ar, ap);
    roll = cfg::ALPHA * (roll + gx * dt) + (1 - cfg::ALPHA) * (ar - _roll0);
    pitch = cfg::ALPHA * (pitch + gy * dt) + (1 - cfg::ALPHA) * (ap - _pitch0);
    ax = rax;
    ay = ray;
    az = raz;
    return true;
}