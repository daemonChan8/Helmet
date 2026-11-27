#pragma once

// =============================================================================
// Pin Definitions — LilyGO TSIM7600G-H R2 + Peripheral Map
// =============================================================================

// --- Battery LED / gauge (main board-owned, passenger-style behavior) ---
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22

// Single local charge-state LED (red, plain on/off — not RGB). This board no
// longer displays the full battery-level color scheme locally; that now
// lives entirely on the HelmetRIDER ESP32's RGB LED, driven by the 1-byte
// state code this board sends over UART (see PIN_UART_TX below and
// BattUartState in battery_led.cpp). This LED only shows charging (blink)
// vs full (solid) vs neither (off) — it does NOT show normal battery level.
// Repurposed from the old RGB_RED_PIN, same physical pin.
#define PIN_CHG_LED              18

// --- Driver main board local UART relay to/from HelmetRIDER ESP32 ---
#define PIN_UART_RX             13      // UART2 RX ← HelmetRIDER ESP32 TX
#define PIN_UART_TX             14      // UART2 TX → HelmetRIDER ESP32 RX (A2DP audio relay path)
#define UART_BAUD_RATE          921600

// --- Driver main board modem / GSM (SMS only — GPS now lives on the NEO-M8N below) ---
#define EMERGENCY_UNIT_LABEL    "DRIVER"

#define MODEM_RX_PIN            26   // ESP32 RX  <- SIM7600 TX
#define MODEM_TX_PIN            27   // ESP32 TX  -> SIM7600 RX
#define MODEM_PWRKEY_PIN        4
#define MODEM_FLIGHT_PIN        25   // MUST be HIGH to exit flight mode / enable RF
#define MODEM_DTR_PIN           32   // MUST be LOW to keep the modem from sleeping
#define MODEM_BAUD               115200

#define PIN_BUCKLE_SWITCH       15      // local buckle input on driver main; wake on HIGH

// --- External NEO-M8N GPS module (replaces the SIM7600's integrated GNSS) ---
// All 3 of the ESP32's hardware UARTs are already committed (UART0 = USB/
// console Serial, UART1 = Serial1 modem AT, UART2 = Serial2 relay from the
// HelmetRIDER board), so the NEO-M8N is read over a bit-banged SoftwareSerial
// (EspSoftwareSerial library) instead. 9600 baud is the NEO-M8N's power-on
// default and is comfortably within SoftwareSerial's reliable range.
//
// NOTE: GPIO16/17 are deliberately avoided — on WROVER-based LilyGO variants
// those are wired internally to PSRAM and aren't usable as GPIO. 23/19 below
// are common free candidates on this board family, but please verify against
// your unit's schematic/silkscreen before wiring — this board has several
// pins already committed to the SD card interface and other peripherals.
#define GPS_RX_PIN               23   // ESP32 RX  <- NEO-M8N TX
#define GPS_TX_PIN               19   // ESP32 TX  -> NEO-M8N RX (not required for basic NMEA readout)
#define GPS_BAUD                 9600

#define PIN_CHARGE_SENSE   39