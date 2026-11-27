#include "buckle_switch.h"
#include "pins.h"
#include "BatteryRGBIndicator.h"
#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

namespace {
    constexpr uint32_t DEBOUNCE_MS = 50;
    constexpr uint32_t CRITICAL_SLEEP_MS = 5000;

    bool          pin_initialized       = false;

    int           last_buckle_raw       = HIGH;
    unsigned long buckle_last_change_ms = 0;

    int           last_charge_raw       = LOW;
    unsigned long charge_last_change_ms = 0;

    bool          was_critical          = false;
    unsigned long critical_since_ms    = 0;

    void prepare_for_sleep() {
        rtc_gpio_pullup_dis(GPIO_NUM_15);
        rtc_gpio_pulldown_en(GPIO_NUM_15);

        const uint64_t wakeMask =
            (1ULL << PIN_BUCKLE_SWITCH) |
            (1ULL << PIN_CHARGE_SENSE);

        esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    void request_esp32_sleep() {
        Serial2.println("SLEEP");
        Serial2.flush();
    }
}

void buckle_switch_check() {
    if (!pin_initialized) {
        pinMode(PIN_BUCKLE_SWITCH, INPUT_PULLDOWN);
        // PIN_CHARGE_SENSE pinMode is set once in batteryLedBegin(),
        // which runs before this in setup() - not repeated here.

        last_buckle_raw       = digitalRead(PIN_BUCKLE_SWITCH);
        buckle_last_change_ms = millis();

        last_charge_raw       = isChargerPresent() ? HIGH : LOW;
        charge_last_change_ms = millis();

        pin_initialized = true;
    }

    uint32_t now = millis();

    int buckleRaw = digitalRead(PIN_BUCKLE_SWITCH);
    if (buckleRaw != last_buckle_raw) {
        last_buckle_raw       = buckleRaw;
        buckle_last_change_ms = now;
        return;
    }
    bool buckleStable = (now - buckle_last_change_ms >= DEBOUNCE_MS);

    int chargeRaw = isChargerPresent() ? HIGH : LOW;
    if (chargeRaw != last_charge_raw) {
        last_charge_raw       = chargeRaw;
        charge_last_change_ms = now;
        return;
    }
    bool chargeStable = (now - charge_last_change_ms >= DEBOUNCE_MS);

    if (!buckleStable || !chargeStable)
        return;

    if (batteryIsCritical()) {
        if (!was_critical) {
            was_critical = true;
            critical_since_ms = now;
        } else if (now - critical_since_ms >= CRITICAL_SLEEP_MS) {
            Serial.println("[BuckleSwitch] Critical low battery - coordinating deep sleep");
            Serial.flush();
            request_esp32_sleep();
            prepare_for_sleep();
            delay(50);
            esp_deep_sleep_start();
        }
    } else {
        was_critical = false;
    }

    // Sleep when buckle is open and charger is absent.
    if (buckleRaw == LOW && chargeRaw == LOW) {
        Serial.println("[BuckleSwitch] Buckle open, no charger - coordinating deep sleep");
        Serial.flush();

        request_esp32_sleep();
        prepare_for_sleep();
        delay(50);
        esp_deep_sleep_start();
    }
}