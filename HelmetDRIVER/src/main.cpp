#include <Arduino.h>

#include "pins.h"
#include "bt_audio.h"
#include "BatteryRGBIndicator.h"
#include "buckle_switch.h"
#include "emergency_sms.h"
#include "ble_telemetry.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("\n[SETUP] HelmetRIDER Driver");

    batteryLedBegin();
    bt_audio_init("Helmet-RIDER");
    ble_telemetry_init("Helmet-RIDER");
    emergency_sms_init();
}

void loop()
{
    batteryLedUpdate();
    double latitude = 0.0;
    double longitude = 0.0;
    uint32_t gps_age_ms = 0;
    uint8_t gps_valid = emergency_sms_get_gps_location(latitude, longitude, gps_age_ms) ? 1 : 0;
    ble_telemetry_update(batteryGetPercent(), bt_audio_get_remote_wear(),
                         gps_valid);
    bt_audio_process();
    if (bt_audio_is_connected()) {
        ble_telemetry_start_advertising();
    }
    buckle_switch_check();
    emergency_sms_process();

    delay(5);
}