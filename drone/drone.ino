#include <Arduino.h>
#include "imu.h"
#include "quad.h"
#include "shared.h"
#include "flightloop.h"
#include "commands.h"
#include "status.h"

Imu imu;
Quad quad;
FlightLoop flight(imu, quad);
Commands serialCmds;

void setup()
{
    Serial.begin(115200);
    delay(500);
    quad.begin(); // gates low, disarmed

    Serial.println("calibrating, keep flat and still...");
    if (!imu.begin())
        Serial.println("IMU failed - stabiliser unavailable");

    flight.start();
    Serial.println("ready (motors disarmed)");
    Serial.println("cmds: a=arm  s=stop  b=stab on/off  m<0-4>=motor  t<0-100>=throttle %");
}

void loop()
{
    serialCmds.poll();
    Status();
    delay(5);
}