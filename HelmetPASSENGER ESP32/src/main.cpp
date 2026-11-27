#include <Arduino.h>

#include "link_manager.h"
#include "wear_detector.h"
#include "buckle_switch.h"


void setup()
{
    Serial.begin(115200);
    Serial.println("\n[SETUP] HelmetPASSENGER ESP32");

    link_manager_init();
}

void loop()
{
    wearDetectorUpdate();
    buckle_switch_check();
    link_manager_process();

    static uint8_t lastWear = 255;
    static uint32_t lastTelemetryMs = 0;
    uint8_t wear = isWearDetected() ? 1 : 0;

    if (wear != lastWear || millis() - lastTelemetryMs >= 1000) {
        lastWear = wear;
        lastTelemetryMs = millis();
        uint8_t pass_pct = link_manager_get_passenger_battery_pct();
        link_manager_publish_telemetry(pass_pct, wear);
    }

    delay(5);
}
