#include <Arduino.h>

#include "gamepad.h"

drone::Gamepad pad;

void setup()
{
    Serial.begin(115200);
    delay(1500); // let USB CDC reconnect so you don't miss the banner
    BP32.enableNewBluetoothConnections(true);
    BP32.forgetBluetoothKeys();
    Serial.println("Booted, waiting for controller...");
    pad.begin();
}

void loop()
{
    pad.update();

    static uint32_t lastBeat = 0;
    if (millis() - lastBeat > 2000)
    {
        lastBeat = millis();
        Serial.println("alive");
    }

    ControllerPtr ctl = pad.get(0);
    if (ctl && ctl->isConnected() && ctl->hasData())
        Serial.printf("LX:%d LY:%d RX:%d RY:%d LT:%d RT:%d\n",
                      ctl->axisX(), ctl->axisY(), ctl->axisRX(), ctl->axisRY(),
                      ctl->brake(), ctl->throttle());

    delay(10);
}