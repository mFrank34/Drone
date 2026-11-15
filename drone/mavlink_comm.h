#pragma once

// MAVLink v2 over BLE. Call mavBegin() once from setup().
//
//   controller app --BLE--> MANUAL_CONTROL / COMMAND_LONG --> cmd  (read by the control loop)
//   tele (written by the control loop) --> HEARTBEAT / ATTITUDE / VFR_HUD --BLE--> app

void mavBegin(); // starts BLE and a low-priority task that runs mavPoll()
void mavPoll();  // one non-blocking step: parse input, send due telemetry
