#pragma once
#include <cstdint>

void batteryLedBegin();
void batteryLedUpdate();

uint8_t batteryGetPercent();
bool    batteryIsCritical();
bool    batteryIsChargerConnected();
bool    isChargerPresent();
