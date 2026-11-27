#pragma once

// Call once from setup() before starting audio tasks.
void buckle_switch_init();

// Call this every loop() iteration.
// Debounces the buckle pin; if confirmed LOW, puts the board into
// deep sleep and configures wake-on-HIGH via GPIO33 EXT0 wake.
void buckle_switch_check();