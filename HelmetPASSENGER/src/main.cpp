#include <Arduino.h>

#include "pins.h"
#include "BatteryRGBIndicator.h"
#include "buckle_switch.h"
#include "emergency_sms.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("\n[SETUP] HelmetRIDER Driver");

    batteryLedBegin();
    emergency_sms_init();
}

void loop()
{
    batteryLedUpdate();
    double latitude = 0.0;
    double longitude = 0.0;
    uint32_t gps_age_ms = 0;
    uint8_t gps_valid = emergency_sms_get_gps_location(latitude, longitude, gps_age_ms) ? 1 : 0;
    buckle_switch_check();
    emergency_sms_process();

    delay(5);
}