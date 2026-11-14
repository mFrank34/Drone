#include "web.h"
#include <WiFi.h>
#include <WebServer.h>
#include "shared.h"
#include "page.h"

static WebServer server(80);

static void seen() { cmd.lastSeenMs = millis(); }

static void hData()
{
    seen();
    Telemetry t;
    portENTER_CRITICAL(&teleMux);
    t = tele;
    portEXIT_CRITICAL(&teleMux);
    char b[400];
    snprintf(b, sizeof b,
             "{\"armed\":%s,\"fault\":\"%s\",\"thr\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,"
             "\"gx\":%.1f,\"gy\":%.1f,\"FL\":%.0f,\"FR\":%.0f,\"BL\":%.0f,\"BR\":%.0f,"
             "\"rate\":%lu,\"periodMaxUs\":%lu}",
             t.armed ? "true" : "false", t.fault, t.thr * 100, t.roll, t.pitch, t.gx, t.gy,
             t.out[CORNER_FL] * 100, t.out[CORNER_FR] * 100,
             t.out[CORNER_BL] * 100, t.out[CORNER_BR] * 100,
             (unsigned long)t.rate, (unsigned long)t.periodMaxUs);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", b);
}

static void hSet()
{
    seen();
    cmd.thr = constrain(server.arg("thr").toFloat() / 100.0f, 0.0f, cfg::MAX_OUT);
    cmd.sel = constrain(server.arg("sel").toInt(), 0, 4);
    cmd.stab = server.arg("stab") == "1";
    server.send(200, "application/json", "{\"ok\":1}");
}

static void hArm()
{
    seen();
    if (server.method() != HTTP_POST)
    {
        server.send(405, "text/plain", "POST only");
        return;
    }
    cmd.thr = 0.0f;
    cmd.arm = true;
    server.send(200, "application/json", "{\"ok\":1}");
}

static void hStop()
{ // any method, so it is as easy to hit as possible
    seen();
    cmd.stop = true;
    cmd.thr = 0.0f;
    server.send(200, "application/json", "{\"ok\":1}");
}

static void hRoot()
{
    seen();
    server.send_P(200, "text/html", PAGE);
}

static void serverTask(void *)
{
    for (;;)
    {
        server.handleClient();
        vTaskDelay(1);
    }
}

void webBegin()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP("drone-imu", "dronedrone1", 1, 0, 4);
    WiFi.setTxPower(WIFI_POWER_8_5dBm); // same level that worked in MicroPython
    Serial.printf("AP up: join 'drone-imu', open http://%s\n", WiFi.softAPIP().toString().c_str());

    server.on("/", hRoot);
    server.on("/data", hData);
    server.on("/set", hSet);
    server.on("/arm", hArm);
    server.on("/stop", hStop);
    server.begin();
    xTaskCreatePinnedToCore(serverTask, "web", 6144, nullptr, 2, nullptr, 0);
}