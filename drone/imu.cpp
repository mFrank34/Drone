// imu.cpp
#include "imu.h"
#include <Wire.h>

static constexpr uint8_t REG_WHO_AM_I = 0x0F;
static constexpr uint8_t REG_CTRL1_XL = 0x10;
static constexpr uint8_t REG_CTRL2_G = 0x11;
static constexpr uint8_t REG_CTRL3_C = 0x12;
static constexpr uint8_t REG_OUT_G = 0x22;

static constexpr float ACC_LSB_PER_G = 16384.0f; // +-2 g
static constexpr float GYR_LSB_PER_DPS = 57.14f; // +-500 dps

static bool regWrite(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(cfg::MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool regRead(uint8_t reg, uint8_t &val)
{
    Wire.beginTransmission(cfg::MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((int)cfg::MPU_ADDR, 1) != 1)
        return false;
    val = Wire.read();
    return true;
}

bool Imu::begin()
{
    Wire.begin(cfg::SDA_PIN, cfg::SCL_PIN, 400000);

    uint8_t id = 0;
    if (!regRead(REG_WHO_AM_I, id) || id != 0x69)
        return false; // not an LSM6DS3 at this address

    if (!regWrite(REG_CTRL3_C, 0x01)) // software reset
        return false;
    delay(50);
    if (!regWrite(REG_CTRL3_C, 0x44)) // BDU + register auto-increment
        return false;
    if (!regWrite(REG_CTRL1_XL, 0x60)) // accel 416 Hz, +-2 g
        return false;
    if (!regWrite(REG_CTRL2_G, 0x64)) // gyro 416 Hz, 500 dps
        return false;
    delay(100);

    float a, b, c, d, e, f; // discard settling samples
    for (int i = 0; i < 10; i++)
    {
        readRaw(a, b, c, d, e, f);
        delay(10);
    }
    return calibrate();
}

bool Imu::readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
    Wire.beginTransmission(cfg::MPU_ADDR);
    Wire.write(REG_OUT_G);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((int)cfg::MPU_ADDR, 12) != 12)
        return false;
    int16_t v[6];
    for (int i = 0; i < 6; i++)
    {
        uint8_t lo = Wire.read(), hi = Wire.read(); // little-endian
        v[i] = (int16_t)((hi << 8) | lo);
    }
    gx = v[0] / GYR_LSB_PER_DPS;
    gy = v[1] / GYR_LSB_PER_DPS;
    gz = v[2] / GYR_LSB_PER_DPS;
    ax = v[3] / ACC_LSB_PER_G;
    ay = v[4] / ACC_LSB_PER_G;
    az = v[5] / ACC_LSB_PER_G;
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