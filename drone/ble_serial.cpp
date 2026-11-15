#include "ble_serial.h"
#include "config.h"
#include <NimBLEDevice.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"

// Written against NimBLE-Arduino 2.x (h2zero).
// Nordic UART naming is from the *peripheral's* point of view:
//   RX char (…0002) = the app writes to it, we read
//   TX char (…0003) = we notify on it, the app reads
static const char *NUS_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static NimBLECharacteristic *txChar = nullptr;
static StreamBufferHandle_t rxBuf = nullptr; // BLE host task -> MAVLink task
static std::atomic<bool> connected{false};
static std::atomic<uint16_t> chunk{20}; // ATT payload = MTU - 3; 20 until the app negotiates more

class ServerCb : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override
    {
        chunk = 20;
        connected = true;
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override
    {
        connected = false;
        NimBLEDevice::startAdvertising(); // be findable again straight away
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo &) override
    {
        chunk = (mtu > 3) ? (uint16_t)min<int>(mtu - 3, 244) : 20;
    }
};

class RxCb : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override
    {
        const auto v = c->getValue();
        // Never block the BLE host task. If we're full we drop bytes and the
        // MAVLink parser resyncs on the next frame.
        xStreamBufferSend(rxBuf, v.data(), v.size(), 0);
    }
};

bool bleSerialBegin(const char *name)
{
    rxBuf = xStreamBufferCreate(1024, 1);
    if (!rxBuf)
        return false;

    NimBLEDevice::init(name);
    NimBLEDevice::setMTU(247);

    uint32_t rxProps = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
    if (cfg::BLE_PASSKEY != 0)
    {
        // Bond with a fixed passkey. The app/phone will ask the user to type it.
        NimBLEDevice::setSecurityAuth(true, true, true);
        NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
        NimBLEDevice::setSecurityPasskey(cfg::BLE_PASSKEY);
        rxProps |= NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN;
    }

    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCb());

    NimBLEService *svc = server->createService(NUS_SERVICE);
    txChar = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic *rx = svc->createCharacteristic(NUS_RX, rxProps);
    rx->setCallbacks(new RxCb());
    svc->start();

    // 128-bit UUID + flags fills the advertising packet, so the name goes in the scan response.
    NimBLEAdvertisementData adv, scan;
    adv.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    adv.addServiceUUID(NUS_SERVICE);
    scan.setName(name);
    NimBLEAdvertising *a = NimBLEDevice::getAdvertising();
    a->setAdvertisementData(adv);
    a->setScanResponseData(scan);
    return a->start();
}

bool bleSerialConnected() { return connected; }

size_t bleSerialRead(uint8_t *buf, size_t max)
{
    if (!rxBuf)
        return 0;
    return xStreamBufferReceive(rxBuf, buf, max, 0);
}

void bleSerialWrite(const uint8_t *data, size_t n)
{
    if (!connected || !txChar)
        return;
    while (n)
    {
        const size_t k = min<size_t>(n, chunk);
        if (!txChar->notify(data, k))
        {
            vTaskDelay(pdMS_TO_TICKS(3)); // controller's buffers full; give it a moment, then give up
            if (!txChar->notify(data, k))
                return; // telemetry is lossy by design; a torn frame fails CRC and is skipped
        }
        data += k;
        n -= k;
    }
}
