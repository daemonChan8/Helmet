#include "wear_detector.h"
#include "pins.h"

#include <Arduino.h>

static bool          s_initialized   = false;
static bool          s_wearDetected  = false;
static unsigned long s_lastAboveMs   = 0;
static unsigned long s_lastDebugMs   = 0;
static int           s_fsrFiltered   = 0;

static void wearDetectorInit()
{
    pinMode(PIN_FSR, INPUT);
    s_fsrFiltered = analogRead(PIN_FSR);
    s_initialized = true;
}

void wearDetectorUpdate()
{
    if (!s_initialized) {
        wearDetectorInit();
    }

    int fsrValue = analogRead(PIN_FSR);
    s_fsrFiltered = (s_fsrFiltered * 3 + fsrValue) / 4;

    if (millis() - s_lastDebugMs >= 2000) {
        Serial.printf("[WearDetector] raw=%d filtered=%d detected=%d\n", fsrValue, s_fsrFiltered, s_wearDetected);
        s_lastDebugMs = millis();
    }

    if (s_fsrFiltered >= FSR_THRESHOLD) {
        s_wearDetected = true;
        s_lastAboveMs  = millis();
    } else if (s_wearDetected) {
        if (millis() - s_lastAboveMs >= WEAR_HOLD_TIME_MS) {
            s_wearDetected = false;
        }
    }
}

bool isWearDetected()
{
    return s_wearDetected;
}
