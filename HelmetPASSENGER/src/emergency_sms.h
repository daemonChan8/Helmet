#pragma once
#include <stdint.h>

void emergency_sms_init();
void emergency_sms_process();
void emergency_sms_trigger_remote();

bool emergency_sms_busy();
uint8_t emergency_sms_get_gps_status();
uint8_t emergency_sms_get_network_status();
bool emergency_sms_get_gps_location(double& lat, double& lon, uint32_t& age_ms);