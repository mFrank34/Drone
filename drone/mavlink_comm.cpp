#include "mavlink_comm.h"
#include "ble_serial.h"
#include "shared.h"
#include "config.h"

// MAVLink headers are vendored in drone/mavlink/ so no library install is needed.
#include "mavlink/common/mavlink.h"

// ---------------------------------------------------------------------------
// state owned by this file (only ever touched from the MAVLink task)
// ---------------------------------------------------------------------------
static uint32_t tHeartbeat = 0, tAttitude = 0, tHud = 0;
static bool wasConnected = false;
static uint32_t lastManualMs = 0; // 0 = no stick input seen on this link
static float lastStick = 1.0f;    // stick throttle 0..1; assume "high" until proven low
static uint16_t prevButtons = 0;
static const char *lastFault = "";

// ---------------------------------------------------------------------------
// sending
// ---------------------------------------------------------------------------
static void send(const mavlink_message_t &m)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
    bleSerialWrite(buf, n);
}

static void sendHeartbeat(const Telemetry &t)
{
    mavlink_heartbeat_t hb = {};
    hb.type = MAV_TYPE_QUADROTOR;
    hb.autopilot = MAV_AUTOPILOT_GENERIC;
    hb.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (t.armed)
        hb.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    if (cmd.stab)
        hb.base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    hb.custom_mode = cmd.stab ? 1 : 0; // 0 = direct throttle, 1 = stabilised
    hb.system_status = t.armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    mavlink_message_t m;
    mavlink_msg_heartbeat_encode(SYSTEM_ID, COMPONENT_ID, &m, &hb);
    send(m);
}

static void sendAttitude(const Telemetry &t)
{
    mavlink_attitude_t a = {};
    a.time_boot_ms = millis();
    a.roll = t.roll * DEG_TO_RAD;
    a.pitch = t.pitch * DEG_TO_RAD;
    a.yaw = 0; // no heading source on this airframe
    a.rollspeed = t.gx * DEG_TO_RAD;
    a.pitchspeed = t.gy * DEG_TO_RAD;
    a.yawspeed = 0;
    mavlink_message_t m;
    mavlink_msg_attitude_encode(SYSTEM_ID, COMPONENT_ID, &m, &a);
    send(m);
}

static void sendHud(const Telemetry &t)
{
    mavlink_vfr_hud_t v = {};
    v.throttle = (uint16_t)(constrain(t.thr, 0.0f, 1.0f) * 100.0f + 0.5f);
    mavlink_message_t m;
    mavlink_msg_vfr_hud_encode(SYSTEM_ID, COMPONENT_ID, &m, &v);
    send(m);
}

static void sendText(uint8_t severity, const char *prefix, const char *text)
{
    mavlink_statustext_t s = {};
    s.severity = severity;
    snprintf(s.text, sizeof s.text, "%s%s", prefix, text);
    mavlink_message_t m;
    mavlink_msg_statustext_encode(SYSTEM_ID, COMPONENT_ID, &m, &s);
    send(m);
}

static void sendAck(const mavlink_message_t &req, uint16_t command, uint8_t result)
{
    mavlink_command_ack_t a = {};
    a.command = command;
    a.result = result;
    a.target_system = req.sysid;
    a.target_component = req.compid;
    mavlink_message_t m;
    mavlink_msg_command_ack_encode(SYSTEM_ID, COMPONENT_ID, &m, &a);
    send(m);
}

// One dummy parameter. Ground stations ask for the parameter list on connect
// and some wait on it; answering with a short list keeps them from stalling.
static void sendParam()
{
    mavlink_param_value_t p = {};
    strncpy(p.param_id, "DRONE_VER", sizeof p.param_id);
    p.param_value = 1.0f;
    p.param_type = MAV_PARAM_TYPE_REAL32;
    p.param_count = 1;
    p.param_index = 0;
    mavlink_message_t m;
    mavlink_msg_param_value_encode(SYSTEM_ID, COMPONENT_ID, &m, &p);
    send(m);
}

// ---------------------------------------------------------------------------
// receiving
// ---------------------------------------------------------------------------
static bool forUs(uint8_t targetSystem) { return targetSystem == 0 || targetSystem == SYSTEM_ID; }

static uint8_t tryArm()
{
    Telemetry t;
    portENTER_CRITICAL(&teleMux);
    t = tele;
    portEXIT_CRITICAL(&teleMux);
    if (t.armed)
        return MAV_RESULT_ACCEPTED;

    const uint32_t now = millis();
    if (lastManualMs == 0 || now - lastManualMs > cfg::ARM_INPUT_MS)
    {
        sendText(MAV_SEVERITY_WARNING, "Arm refused: ", "no stick input");
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }
    if (lastStick > cfg::ARM_MAX_THR)
    {
        sendText(MAV_SEVERITY_WARNING, "Arm refused: ", "throttle not low");
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }
    cmd.thr = 0.0f;
    cmd.arm = true; // the control loop does the actual arm() and restarts the link timer
    return MAV_RESULT_ACCEPTED;
}

static void onCommandLong(const mavlink_message_t &m)
{
    mavlink_command_long_t c;
    mavlink_msg_command_long_decode(&m, &c);
    if (!forUs(c.target_system))
        return;

    uint8_t result = MAV_RESULT_UNSUPPORTED;
    if (c.command == MAV_CMD_COMPONENT_ARM_DISARM)
    {
        if (c.param1 > 0.5f)
            result = tryArm();
        else
        {
            cmd.thr = 0.0f;
            cmd.stop = true; // disarm is always allowed, force flag or not
            result = MAV_RESULT_ACCEPTED;
        }
    }
    sendAck(m, c.command, result);
}

static void onManualControl(const mavlink_message_t &m)
{
    mavlink_manual_control_t mc;
    mavlink_msg_manual_control_decode(&m, &mc);
    if (!forUs(mc.target))
        return;

    const uint32_t now = millis();
    lastManualMs = now;
    cmd.lastSeenMs = now; // this, and only this, keeps the failsafe timer alive

    float stick = 0.0f; // an invalid axis (INT16_MAX) means "no thrust"
    if (mc.z != INT16_MAX)
    {
        const float zmin = (float)cfg::MAV_Z_MIN;
        stick = ((float)mc.z - zmin) / (1000.0f - zmin);
    }
    stick = constrain(stick, 0.0f, 1.0f);
    lastStick = stick;
    cmd.thr = stick * cfg::MAV_THR_CAP;

    // Button 1 toggles the stabiliser on its rising edge.
    const uint16_t pressed = mc.buttons & (uint16_t)~prevButtons;
    if (pressed & 0x0001)
        cmd.stab = !cmd.stab;
    prevButtons = mc.buttons;
}

static void handle(const mavlink_message_t &m)
{
    switch (m.msgid)
    {
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        onManualControl(m);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        onCommandLong(m);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        sendParam();
        break;
    default: // heartbeats from the app etc: nothing to do
        break;
    }
}

// ---------------------------------------------------------------------------
// link edges
// ---------------------------------------------------------------------------
static void onLinkUp()
{
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    lastManualMs = 0;
    lastStick = 1.0f;
    prevButtons = 0;
    tHeartbeat = tAttitude = tHud = 0;
}

static void onLinkDown()
{
    cmd.thr = 0.0f;
    cmd.stop = true; // don't wait out the 1 s timer when we *know* the app is gone
    lastManualMs = 0;
}

// ---------------------------------------------------------------------------
// public
// ---------------------------------------------------------------------------
void mavPoll()
{
    const uint32_t now = millis();
    const bool conn = bleSerialConnected();
    if (conn && !wasConnected)
        onLinkUp();
    if (!conn && wasConnected)
        onLinkDown();
    wasConnected = conn;

    uint8_t rx[64];
    size_t n;
    while ((n = bleSerialRead(rx, sizeof rx)) > 0)
    {
        for (size_t i = 0; i < n; i++)
        {
            mavlink_message_t m;
            mavlink_status_t st;
            if (mavlink_parse_char(MAVLINK_COMM_0, rx[i], &m, &st))
                handle(m);
        }
    }

    if (!conn)
        return;

    Telemetry t;
    portENTER_CRITICAL(&teleMux);
    t = tele;
    portEXIT_CRITICAL(&teleMux);

    if (now - tHeartbeat >= 1000)
    {
        tHeartbeat = now;
        sendHeartbeat(t);
    }
    if (now - tAttitude >= 50) // 20 Hz
    {
        tAttitude = now;
        sendAttitude(t);
    }
    if (now - tHud >= 200) // 5 Hz
    {
        tHud = now;
        sendHud(t);
    }
    if (t.fault != lastFault) // faults are always string literals, so comparing pointers is enough
    {
        lastFault = t.fault;
        if (t.fault && t.fault[0])
            sendText(MAV_SEVERITY_WARNING, "Motors cut: ", t.fault);
    }
}

static void mavTask(void *)
{
    for (;;)
    {
        mavPoll();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void mavBegin()
{
    if (!bleSerialBegin(BLE_DEVICE_NAME))
    {
        Serial.println("BLE init failed - no controller link");
        return;
    }
    Serial.printf("BLE up: connect your MAVLink app to '%s'\n", BLE_DEVICE_NAME);
    // Core 0 with the radio stack; the control loop owns core 1.
    xTaskCreatePinnedToCore(mavTask, "mavlink", 4096, nullptr, 3, nullptr, 0);
}
