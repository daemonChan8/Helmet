#pragma once
#include <stdint.h>

void ble_telemetry_init(const char* device_name);
void ble_telemetry_start_advertising();
void ble_telemetry_update(uint8_t battery_percent, uint8_t wear_detected,
                          uint8_t gps_valid);
void ble_telemetry_set_passenger(uint8_t passenger_battery, uint8_t passenger_wear);