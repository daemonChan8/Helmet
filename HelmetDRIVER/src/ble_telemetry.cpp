#include "ble_telemetry.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
    constexpr char SERVICE_UUID[] = "7f1e0001-5e11-4a61-9f0a-000000000001";
    constexpr char RIDER_TELEMETRY_UUID[] = "7f1e0002-5e11-4a61-9f0a-000000000001";
    constexpr char PASSENGER_TELEMETRY_UUID[] = "7f1e0003-5e11-4a61-9f0a-000000000001";

    BLECharacteristic* rider_characteristic = nullptr;
    BLECharacteristic* passenger_characteristic = nullptr;

    uint8_t last_battery = 255;
    uint8_t last_wear = 255;
    uint8_t last_gps = 255;
    uint8_t last_passenger_batt = 255;
    uint8_t last_passenger_wear = 255;

    bool advertising_started = false;
    bool client_connected = false;
    uint32_t last_notify_ms = 0;

    // Set independently once the central writes each characteristic's CCCD.
    // onConnect() only means the link is up, not that notifications are ready.
    bool rider_notifications_enabled = false;
    bool passenger_notifications_enabled = false;

    SemaphoreHandle_t telemetry_mutex = nullptr;

    // Pending flags: values changed while we weren't allowed to notify yet
    // (not connected, or connected but not yet subscribed). Cleared once we
    // catch up with a real notify.
    bool rider_pending = false;
    bool passenger_pending = false;

    // 255 is the "never received a value yet" sentinel -> render as N/A.
    // An actual 0 reading is real data (0% battery, no wear, no gps fix)
    // and is rendered as 0, not N/A.
    void format_field(char* out, size_t outsz, uint8_t val)
    {
        if (val == 255) {
            snprintf(out, outsz, "N/A");
        } else {
            snprintf(out, outsz, "%u", val);
        }
    }

    bool can_notify(bool characteristic_enabled)
    {
        return client_connected && characteristic_enabled;
    }

    void publish_rider_values()
    {
        if (!rider_characteristic) return;
        char batt_s[5], wear_s[5], gps_s[5];
        format_field(batt_s, sizeof(batt_s), last_battery);
        format_field(wear_s, sizeof(wear_s), last_wear);
        format_field(gps_s, sizeof(gps_s), last_gps);

        char telemetry[20];
        snprintf(telemetry, sizeof(telemetry), "b=%s,w=%s,g=%s",
                 batt_s, wear_s, gps_s);
        rider_characteristic->setValue(telemetry);

        if (!can_notify(rider_notifications_enabled)) {
            rider_pending = true;
            return;
        }
        Serial.printf("[BLE] rider notify: %s\n", telemetry);
        rider_characteristic->notify();
        rider_pending = false;
    }

    void publish_passenger_values()
    {
        if (!passenger_characteristic) return;
        char batt_s[5], wear_s[5];
        format_field(batt_s, sizeof(batt_s), last_passenger_batt);
        format_field(wear_s, sizeof(wear_s), last_passenger_wear);

        char telemetry[14];
        snprintf(telemetry, sizeof(telemetry), "b=%s,w=%s",
                 batt_s, wear_s);
        passenger_characteristic->setValue(telemetry);

        if (!can_notify(passenger_notifications_enabled)) {
            passenger_pending = true;
            return;
        }
        Serial.printf("[BLE] passenger notify: %s\n", telemetry);
        passenger_characteristic->notify();
        passenger_pending = false;
    }

    // Fires once notifications actually become usable (post-connect AND
    // post-CCCD-subscribe), so the client gets a fresh snapshot instead of
    // waiting up to 1s for the next periodic tick.
    void flush_pending_if_ready()
    {
        if (!client_connected) return;
        if (rider_pending && rider_notifications_enabled) publish_rider_values();
        if (passenger_pending && passenger_notifications_enabled) publish_passenger_values();
    }

    class TelemetryServerCallbacks : public BLEServerCallbacks {
        void onConnect(BLEServer*) override
        {
            xSemaphoreTakeRecursive(telemetry_mutex, portMAX_DELAY);
            client_connected = true;
            rider_notifications_enabled = false;
            passenger_notifications_enabled = false;
            last_notify_ms = 0;
            // Mark both as pending so whichever arrives first (a CCCD
            // subscribe event, or the next update/periodic tick) flushes a
            // fresh snapshot rather than notifying blind before the client
            // is actually ready to receive.
            rider_pending = true;
            passenger_pending = true;
            Serial.println("[BLE] Telemetry client connected");
            xSemaphoreGiveRecursive(telemetry_mutex);
        }

        void onDisconnect(BLEServer*) override
        {
            xSemaphoreTakeRecursive(telemetry_mutex, portMAX_DELAY);
            client_connected = false;
            rider_notifications_enabled = false;
            passenger_notifications_enabled = false;
            Serial.println("[BLE] Telemetry client disconnected");
            xSemaphoreGiveRecursive(telemetry_mutex);
            BLEDevice::startAdvertising();
        }
    };

    // Watches the CCCD (BLE2902) writes on each characteristic so we know
    // the exact moment notifications become usable, instead of guessing
    // based on onConnect() timing.
    class NotifyEnableCallbacks : public BLEDescriptorCallbacks {
        bool* enabled;

    public:
        explicit NotifyEnableCallbacks(bool* enabled) : enabled(enabled) {}

    private:
        void onWrite(BLEDescriptor* descriptor) override
        {
            BLE2902* cccd = (BLE2902*)descriptor;
            xSemaphoreTakeRecursive(telemetry_mutex, portMAX_DELAY);
            *enabled = cccd->getNotifications();
            if (*enabled) {
                Serial.println("[BLE] Client subscribed - notifications enabled");
                flush_pending_if_ready();
            }
            xSemaphoreGiveRecursive(telemetry_mutex);
        }
    };
}

void ble_telemetry_init(const char* device_name)
{
    telemetry_mutex = xSemaphoreCreateRecursiveMutex();
    if (!telemetry_mutex) {
        Serial.println("[BLE] Failed to create telemetry mutex");
        return;
    }
    BLEDevice::init(device_name);
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new TelemetryServerCallbacks());
    BLEService* service = server->createService(SERVICE_UUID);

    rider_characteristic = service->createCharacteristic(
        RIDER_TELEMETRY_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    BLE2902* rider_cccd = new BLE2902();
    rider_cccd->setCallbacks(new NotifyEnableCallbacks(&rider_notifications_enabled));
    rider_characteristic->addDescriptor(rider_cccd);

    passenger_characteristic = service->createCharacteristic(
        PASSENGER_TELEMETRY_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    BLE2902* passenger_cccd = new BLE2902();
    passenger_cccd->setCallbacks(new NotifyEnableCallbacks(&passenger_notifications_enabled));
    passenger_characteristic->addDescriptor(passenger_cccd);

    service->start();
    Serial.println("[BLE] Telemetry service ready (rider + passenger characteristics)");
}

void ble_telemetry_start_advertising()
{
    if (advertising_started) return;
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();
    advertising_started = true;
    Serial.println("[BLE] Telemetry advertising started");
}

void ble_telemetry_update(uint8_t battery_percent, uint8_t wear_detected,
                          uint8_t gps_valid)
{
    if (!rider_characteristic) return;
    if (battery_percent > 100) battery_percent = 100;
    gps_valid = gps_valid ? 1 : 0;
    wear_detected = wear_detected ? 1 : 0;

    uint32_t now = millis();
    xSemaphoreTakeRecursive(telemetry_mutex, portMAX_DELAY);
    bool periodic_notify = can_notify(rider_notifications_enabled) && (now - last_notify_ms >= 1000);
    bool changed = battery_percent != last_battery ||
                   wear_detected != last_wear || gps_valid != last_gps;

    // Always record the latest values, even if we can't notify right now,
    // so a value that changes while disconnected/unsubscribed isn't lost
    // and gets published as soon as notifications become usable.
    if (changed) {
        last_battery = battery_percent;
        last_wear = wear_detected;
        last_gps = gps_valid;
    }

    if ((changed && can_notify(rider_notifications_enabled)) || periodic_notify) {
        publish_rider_values();
    } else if (changed) {
        rider_pending = true; // recorded but not yet notified
    }

    if (periodic_notify) last_notify_ms = now;
    xSemaphoreGiveRecursive(telemetry_mutex);
}

void ble_telemetry_set_passenger(uint8_t passenger_battery, uint8_t passenger_wear)
{
    if (!passenger_characteristic) return;
    xSemaphoreTakeRecursive(telemetry_mutex, portMAX_DELAY);
    if (passenger_battery != 255 && passenger_battery > 100) passenger_battery = 100;
    if (passenger_wear != 255) passenger_wear = passenger_wear ? 1 : 0;

    bool changed = passenger_battery != last_passenger_batt ||
                   passenger_wear != last_passenger_wear;
    if (!changed) {
        xSemaphoreGiveRecursive(telemetry_mutex);
        return;
    }

    last_passenger_batt = passenger_battery;
    last_passenger_wear = passenger_wear;
    publish_passenger_values(); // single notify for both fields (or marks pending)
    xSemaphoreGiveRecursive(telemetry_mutex);
}