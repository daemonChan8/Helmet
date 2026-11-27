#include "buckle_switch.h"
#include "pins.h"
#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

namespace {
    constexpr uint32_t DEBOUNCE_MS = 50;

    bool          pin_initialized       = false;

    int           last_buckle_raw       = HIGH;
    unsigned long buckle_last_change_ms = 0;

    void prepare_for_sleep() {
        const gpio_num_t buckle_pin = (gpio_num_t)PIN_BUCKLE_SWITCH;
        rtc_gpio_init(buckle_pin);
        rtc_gpio_set_direction(buckle_pin, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_dis(buckle_pin);
        rtc_gpio_pulldown_en(buckle_pin);
        esp_sleep_enable_ext0_wakeup(buckle_pin, 1);
    }
}

void buckle_switch_init() {
    if (!pin_initialized) {
        pinMode(PIN_BUCKLE_SWITCH, INPUT_PULLDOWN);
        pinMode(PIN_SPK_SD, OUTPUT);
        digitalWrite(PIN_SPK_SD, HIGH);
        last_buckle_raw       = digitalRead(PIN_BUCKLE_SWITCH);
        buckle_last_change_ms = millis();
        pin_initialized = true;
    }
}

void buckle_switch_check() {
    buckle_switch_init();
    uint32_t now = millis();

    int buckleRaw = digitalRead(PIN_BUCKLE_SWITCH);
    if (buckleRaw != last_buckle_raw) {
        last_buckle_raw       = buckleRaw;
        buckle_last_change_ms = now;
        return;
    }
    bool buckleStable = (now - buckle_last_change_ms >= DEBOUNCE_MS);

    if (!buckleStable)
        return;

    if (buckleRaw == LOW) {
        Serial.println("[BuckleSwitch] Buckle open - entering deep sleep");
        Serial.flush();

        prepare_for_sleep();
        delay(50);
        esp_deep_sleep_start();
    }
}