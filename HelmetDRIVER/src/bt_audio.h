#pragma once

#include <stdint.h>

// =============================================================================
// Bluetooth A2DP Audio Module
// Streams audio from a paired phone, resamples 44.1 kHz → 32 kHz,
// compresses with IMA ADPCM, and forwards via UART to HelmetRIDER driver.
// =============================================================================

/**
 * Initialise Bluetooth A2DP Sink with UART audio forwarding.
 * Audio is received from phone, compressed to ADPCM, and sent via UART.
 * No local I2S speaker output.
 *
 * @param device_name  BT name that will appear on the phone.
 */
void bt_audio_init(const char* device_name = "Helmet-RIDER");

/**
 * Adjust playback volume (for any future local monitoring).
 *
 * @param volume  0–127 (A2DP range).
 */
void bt_audio_set_volume(uint8_t volume);

/**
 * @return true if a phone is currently connected and streaming.
 */
bool bt_audio_is_connected();
uint8_t bt_audio_get_remote_wear();

/**
 * Call once per loop iteration to handle connection status reporting.
 */
void bt_audio_process();
