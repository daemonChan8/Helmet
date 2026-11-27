#pragma once

// =============================================================================
// Pin Definitions — ESP32-WROOM-32 board used for HelmetRIDER ESP32
// =============================================================================

// --- I2S Audio ---
#define PIN_I2S_BCLK            18      // Bit clock
#define PIN_I2S_LRCLK           19      // Word select
#define PIN_I2S_DOUT            23      // MAX98357A DIN (ESP32 → speaker)
#define PIN_I2S_DIN              5       // INMP441 SD (mic → ESP32)

// Speaker amplifier enable pin
#define PIN_SPK_SD               2

// --- UART relay to HelmetDRIVER MAIN board ---
#define PIN_UART_RX             16      // Serial2 RX <- HelmetRIDER TX
#define PIN_UART_TX             17      // Serial2 TX -> HelmetRIDER RX
#define UART_BAUD_RATE          921600

// --- Sensing / input ---
#define PIN_FSR                 39      //BROWN ADC-capable FSR input
#define FSR_THRESHOLD           4000
#define WEAR_HOLD_TIME_MS       3000UL

#define PIN_BTN_MULTIFUNCTION   13      //GREEN mode / volume button
#define PIN_BTN_EMERGENCY       14      //RED dedicated remote emergency trigger -> sends SOS packet to driver main
#define PIN_BUCKLE_SWITCH       33      //PURPLE buckle switch input 

// --- Remote battery-state LED ---
// Mirrors the Driver board's 5-tier battery display (blue/red/critical-
// blink/charging-chase/full-both). Driven by a 1-byte state code received
// over UART from the Driver (LilyGO) board - see bt_audio.cpp's
// remote_battery_led_update(). Only red/blue channels are used, matching
// the original scheme (no green).
#define RGB_RED_PIN              25
#define RGB_BLUE_PIN             26

// --- Reserved safe GPIOs for future expansion ---
// 27, 32 are left free to avoid unnecessary overlap with the
// current BT+I2S+UART+sensor+battery-LED role of this target.