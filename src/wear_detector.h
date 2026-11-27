#pragma once
#include <Arduino.h>

// Call this once per loop(). Handles its own first-call init internally,
// so no separate init() call is required in setup().
void wearDetectorUpdate();

// Returns true if the helmet is currently considered worn
// (either FSR is above threshold, or within the 3s hold window after removal).
bool isWearDetected();
