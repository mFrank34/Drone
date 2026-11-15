#pragma once
#include <Arduino.h>

// A byte pipe over BLE using the Nordic UART Service (NUS), which is what
// BLE-capable MAVLink apps (e.g. recent QGroundControl) look for by default.
// It knows nothing about MAVLink; it just moves bytes.

bool bleSerialBegin(const char *name);              // false if init failed
bool bleSerialConnected();                          // a central is connected
size_t bleSerialRead(uint8_t *buf, size_t max);     // non-blocking, returns bytes copied
void bleSerialWrite(const uint8_t *data, size_t n); // non-blocking-ish; chunks to the MTU, drops if the link is jammed
