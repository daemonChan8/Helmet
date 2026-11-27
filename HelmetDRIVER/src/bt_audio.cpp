#include "bt_audio.h"
#include "pins.h"
#include "emergency_sms.h"
#include "espnow_audio.h"
#include "ble_telemetry.h"

#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"

// -----------------------------------------------------------------------------
// Noise-shaped ADPCM encode wrapper.
//
// Plain adpcm_encode() quantizes each sample and throws away the rounding
// error. That error is highly correlated with the signal, which is what
// gives compressed ADPCM its characteristic "gritty" texture -- it's baked
// in at encode time and no amount of pre-filtering removes it.
//
// This wrapper carries the leftover quantization error forward and folds it
// into the *next* sample before encoding (first-order error feedback / noise
// shaping). Errors don't vanish, but they stop accumulating as correlated
// distortion and get pushed toward a less audible noise floor instead.
//
// `shape_error` must be a persistent, per-stream state variable -- each
// independent encoder (BT capture, UART->ESP-NOW relay, mic path, etc.)
// needs its own, or streams will contaminate each other.
// -----------------------------------------------------------------------------
static inline uint8_t adpcm_encode_shaped(int16_t sample, adpcm_state_t* s, float* shape_error)
{
    float ideal = (float)sample + *shape_error;
    if (ideal >  32767.0f) ideal =  32767.0f;
    if (ideal < -32768.0f) ideal = -32768.0f;
    int16_t fed_in = (int16_t)ideal;

    uint8_t nib = adpcm_encode(fed_in, s);

    // Leftover error = what we wanted to encode vs. where the predictor
    // actually landed after quantization.
    *shape_error = ideal - (float)s->predictor;
    return nib;
}

// =============================================================================
// UART Audio Packet Structure (2 Mbaud to HelmetRIDER Driver)
// =============================================================================
typedef struct {
    uint8_t  sync;               // Sync byte: 0xAA
    uint16_t seq;                // Sequence number (big-endian)
    int16_t  predictor;          // Encoder state at packet start (big-endian)
    uint8_t  step_index;         // Encoder step index
    uint16_t payload_len;        // ADPCM payload size in bytes (big-endian)
    uint8_t  data[2048];         // ADPCM data (4096 samples = 2048 bytes)
    uint8_t  crc;                // Simple CRC8
} uart_audio_packet_t;

#define UART_SYNC_BYTE      0xAA
#define UART_AUDIO_FRAME_SIZE  (4096)   // mono 32kHz samples per packet = 2048 bytes ADPCM

static BluetoothA2DPSink* s_sink = nullptr;
static HardwareSerial*    s_uart = nullptr;
static volatile uint8_t s_remote_wear = 0;

// --- UART queue and sequencing ---
typedef struct {
    int16_t  predictor;
    uint8_t  step_index;
    uint16_t adpcm_bytes;
    uint8_t  data[2048];
} uart_queue_item_t;

#define UART_QUEUE_DEPTH   20

static QueueHandle_t s_uart_queue = nullptr;
static uint16_t      s_seq_num    = 0;

// Last known-good passenger battery/wear parsed off UART, used to fill in
// a field that's missing/invalid on a given TEL line so the combined BLE
// setter (which always takes both fields together) doesn't clobber the
// other field's last published value with an unrelated "unknown"/default.
// 255 = not known yet (matches the BLE layer's own N/A sentinel).
static uint8_t s_last_uart_pass_batt = 255;
static uint8_t s_last_uart_pass_wear = 255;

static void uart_control_task(void* /*param*/)
{
    char line[16] = {};
    size_t length = 0;

    while (true) {
        if (!s_uart || !s_uart->available()) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        char c = (char)s_uart->read();

        if (c == '\n' || c == '\r') {
            line[length] = '\0';
            if (strcmp(line, "SOS") == 0) {
                Serial.println("[SOS] Remote emergency trigger received");
                emergency_sms_trigger_remote();
            } else {
                unsigned int rider_batt = 0, rider_wear = 0, pass_batt = 255, pass_wear = 0;
                int n = sscanf(line, "TEL,%u,%u,%u,%u", &rider_batt, &rider_wear, &pass_batt, &pass_wear);
                Serial.printf("[BLE] UART recv: %s -> rider=%u wear=%u pass_batt=%u pass_wear=%u\n",
                              line, rider_batt, rider_wear, pass_batt, pass_wear);
                if (n >= 2) {
                    s_remote_wear = rider_wear ? 1 : 0;
                }
                if (n == 4) {
                    // Validate wear: fall back to the last known-good value
                    // if this line's reading is missing/invalid, rather than
                    // dropping the update (and thereby also skipping a valid
                    // battery reading on the same line).
                    uint8_t wear_to_send = s_last_uart_pass_wear;
                    if (pass_wear <= 1) {
                        wear_to_send = (uint8_t)pass_wear;
                        s_last_uart_pass_wear = wear_to_send;
                    } else if (pass_wear == 255) {
                        Serial.println("[BLE] UART recv: passenger wear unknown");
                    } else {
                        Serial.printf("[BLE] UART recv: passenger wear invalid=%u\n", pass_wear);
                    }

                    // Validate battery the same way. pass_batt==255 is the
                    // sentinel for "unknown"; do not clamp it, just fall
                    // back to the last known-good reading.
                    uint8_t batt_to_send = s_last_uart_pass_batt;
                    if (pass_batt <= 100) {
                        batt_to_send = (uint8_t)pass_batt;
                        s_last_uart_pass_batt = batt_to_send;
                    } else if (pass_batt == 255) {
                        Serial.println("[BLE] UART recv: passenger battery unknown");
                    } else {
                        Serial.printf("[BLE] UART recv: passenger battery invalid=%u\n", pass_batt);
                    }

                    // Single combined notify (battery + wear together) --
                    // ble_telemetry_set_passenger() only actually notifies
                    // if something changed from what's already published.
                    ble_telemetry_set_passenger(batt_to_send, wear_to_send);
                }
            }
            length = 0;
        } else if (length < sizeof(line) - 1) {
            line[length++] = c;
        } else {
            length = 0;
        }
    }
}

// =============================================================================
// CRC8 Calculation
// =============================================================================
static uint8_t crc8_calc(const uint8_t* data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc << 1) ^ (crc & 0x80 ? 0x07 : 0x00);
        }
    }
    return crc;
}

// =============================================================================
// UART Transmission Task
// =============================================================================
static void uart_send_task(void* param)
{
    uart_queue_item_t item;
    uart_audio_packet_t pkt;

    while (true) {
        if (xQueueReceive(s_uart_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Build UART packet
        pkt.sync        = UART_SYNC_BYTE;
        pkt.seq         = __bswap16(s_seq_num++);    // big-endian
        pkt.predictor   = __bswap16((uint16_t)item.predictor);
        pkt.step_index  = item.step_index;
        pkt.payload_len = __bswap16(item.adpcm_bytes);
        memcpy(pkt.data, item.data, item.adpcm_bytes);

        // Calculate CRC8 over header + payload (excluding sync and crc fields)
        uint16_t crc_len = 2 + 2 + 1 + 2 + item.adpcm_bytes;  // seq + predictor + step_index + len + data
        uint8_t crc_buf[crc_len];
        memcpy(crc_buf + 0, &pkt.seq, 2);
        memcpy(crc_buf + 2, &pkt.predictor, 2);
        memcpy(crc_buf + 4, &pkt.step_index, 1);
        memcpy(crc_buf + 5, &pkt.payload_len, 2);
        memcpy(crc_buf + 7, pkt.data, item.adpcm_bytes);
        pkt.crc = crc8_calc(crc_buf, crc_len);

        // Send packet: sync + header + data + crc
        if (s_uart) {
            s_uart->write(pkt.sync);
            s_uart->write((uint8_t*)&pkt.seq, 2);
            s_uart->write((uint8_t*)&pkt.predictor, 2);
            s_uart->write(pkt.step_index);
            s_uart->write((uint8_t*)&pkt.payload_len, 2);
            s_uart->write(pkt.data, item.adpcm_bytes);
            s_uart->write(pkt.crc);
        }
    }
}

// =============================================================================
// A2DP Audio Callback
// =============================================================================
// Decimates 44100 Hz stereo -> 32000 Hz mono, then IMA ADPCM encodes (4:1).
// Result: ~8 UART packets/sec with low latency.
// Encoder state is saved at the start of each packet for error recovery.
//
// A cascaded two-stage one-pole low-pass filter runs at the full 44100 Hz
// rate on every sample, BEFORE the decimation decision, so energy above
// ~16kHz (the new Nyquist after downsampling to 32000 Hz) is attenuated
// before we throw samples away. A single one-pole stage only rolls off at
// 6dB/octave, which isn't steep enough to fully suppress aliasing; cascading
// two stages doubles that to 12dB/octave. Without this filtering, decimation
// aliases high-frequency content back down into the audible range as
// harsh/sharp artifacts.
//
// The ADPCM encode itself uses noise shaping (adpcm_encode_shaped) so that
// quantization error is fed forward into the next sample instead of just
// being discarded -- this is what keeps the compressed signal from sounding
// gritty on top of whatever aliasing filtering already handles.
static void a2dp_stream_reader(const uint8_t* data, uint32_t length)
{
    if (!s_uart_queue) return;

    const int16_t* src    = (const int16_t*)data;
    uint32_t       frames = length / 4;           // L(int16) + R(int16) per frame

    static adpcm_state_t    enc          = {0, 0};   // persistent encoder state
    static float             shape_error = 0.0f;      // persistent noise-shaping state
    static uart_queue_item_t pkt;                     // packet being built
    static uint16_t          n_samples   = 0;         // mono samples packed so far
    static bool              pkt_open    = false;

    // Stride to downsample: 44100 / 32000 = 1.378125
    static uint32_t downsample_phase = 0;
    const uint32_t  DOWNSAMPLE_NUM   = 44100;
    const uint32_t  DOWNSAMPLE_DEN   = 32000;

    // Cascaded anti-alias low-pass state (two stages -> 12dB/octave rolloff)
    static float lpf_state1 = 0.0f;
    static float lpf_state2 = 0.0f;
    // Combined -3dB point for a two-stage cascaded one-pole filter at
    // fs=44100 Hz, solved for ~16 kHz (the post-decimation Nyquist for the
    // 32000 Hz target). alpha=0.25 was ~10x too low (~1.3 kHz), which wiped
    // out midrange/treble before ADPCM encoding and removed the masking
    // that normally hides quantization noise. If aliasing/brightness
    // reappears, nudge down in small steps (e.g. 0.85) rather than back
    // toward the old value.
    const float  LPF_ALPHA  = 0.9f;   // lower = darker/safer, higher = brighter/riskier

    for (uint32_t f = 0; f < frames; f++) {
        // Mix L+R to mono FIRST, filter every sample regardless of
        // whether we're going to keep it after decimation
        int16_t mono_raw = (int16_t)(((int32_t)src[f*2] + (int32_t)src[f*2+1]) >> 1);

        lpf_state1 += LPF_ALPHA * ((float)mono_raw - lpf_state1);
        lpf_state2 += LPF_ALPHA * (lpf_state1 - lpf_state2);   // second stage, steeper cutoff

        downsample_phase += DOWNSAMPLE_DEN;
        if (downsample_phase < DOWNSAMPLE_NUM) continue;
        downsample_phase -= DOWNSAMPLE_NUM;

        int16_t mono = (int16_t)lpf_state2;   // use the twice-filtered value, not mono_raw

        if (!pkt_open) {
            pkt.predictor  = (int16_t)enc.predictor;
            pkt.step_index = (uint8_t)enc.step_index;
            n_samples      = 0;
            pkt_open       = true;
        }

        // Encode sample to ADPCM nibble (noise-shaped)
        uint8_t  nibble   = adpcm_encode_shaped(mono, &enc, &shape_error);
        uint16_t byte_idx = n_samples >> 1;
        if (n_samples & 1)
            pkt.data[byte_idx] |=  (nibble << 4);  // high nibble
        else
            pkt.data[byte_idx]  =   nibble;         // low nibble
        n_samples++;

        // When we've encoded 4096 samples (2048 bytes ADPCM), send packet
        if (n_samples >= UART_AUDIO_FRAME_SIZE) {
            pkt.adpcm_bytes = UART_AUDIO_FRAME_SIZE >> 1;  // 4096 samples -> 2048 bytes
            xQueueSend(s_uart_queue, &pkt, 0);     // non-blocking; drop if full
            pkt_open = false;
        }
    }
}

// =============================================================================
// UART Initialization
// =============================================================================
static void uart_audio_init()
{
    s_uart = &Serial2;
    s_uart->setRxBufferSize(4096);
    s_uart->begin(UART_BAUD_RATE, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    Serial.println("[UART] Audio forwarding initialized");
}

// =============================================================================
// Bluetooth GAP Event Callback
// =============================================================================
static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param)
{
    switch(event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                Serial.println("[BT] Authentication complete");
            } else {
                Serial.printf("[BT] Authentication failed: %d\n", param->auth_cmpl.stat);
            }
            break;
            
        case ESP_BT_GAP_PIN_REQ_EVT:
            Serial.println("[BT] PIN request (entering 0000)");
            esp_bt_gap_pin_reply((uint8_t*)param->pin_req.bda, true, 4, (uint8_t*)"0000");
            break;
            
        case ESP_BT_GAP_CFM_REQ_EVT:
            Serial.println("[BT] Confirmation request");
            esp_bt_gap_ssp_confirm_reply((uint8_t*)param->cfm_req.bda, true);
            break;
            
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            Serial.printf("[BT] Passkey: %06d\n", param->key_notif.passkey);
            break;
            
        default:
            break;
    }
}

// =============================================================================
// Public API
// =============================================================================

void bt_audio_init(const char* device_name)
{
    // Allocate FreeRTOS queue
    s_uart_queue = xQueueCreate(UART_QUEUE_DEPTH, sizeof(uart_queue_item_t));

    Serial.println("[BT] Initialising Bluetooth A2DP Sink...");

    static BluetoothA2DPSink sink;
    s_sink = &sink;

    // Register GAP callback for Bluetooth events
    esp_bt_gap_register_callback(gap_callback);

    // Configure Bluetooth for discoverability
    // Set device name
    esp_bt_dev_set_device_name(device_name);
    
    // Set discoverable mode: Connectable & Discoverable
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // No local I2S playback -- audio goes only to UART.
    // false = data is NOT written to i2s, only forwarded via UART.
    sink.set_stream_reader(a2dp_stream_reader, false);
    // Keep BLE memory/controller support enabled alongside Classic A2DP.
    sink.set_default_bt_mode(ESP_BT_MODE_BTDM);
    sink.set_auto_reconnect(false);
    sink.set_volume(80);
    
    // Start Bluetooth A2DP sink
    sink.start(device_name);

    // Disable BT modem sleep so the controller never powers down the RF.
    esp_bt_sleep_disable();

    Serial.printf("[BT] Ready — device discoverable as \"%s\"\n", device_name);
    Serial.println("[BT] Waiting for phone to pair and connect...");

    // Initialize UART for audio forwarding to HelmetRIDER driver
    uart_audio_init();

    // Pin the send task to core 0 (protocol core) to keep it off the Arduino loop.
    xTaskCreatePinnedToCore(uart_send_task, "uart_tx", 8192, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(uart_control_task, "uart_ctrl", 2048, nullptr, 4, nullptr, 0);
}

void bt_audio_set_volume(uint8_t volume)
{
    if (s_sink) s_sink->set_volume(volume);
}

bool bt_audio_is_connected()
{
    return s_sink && s_sink->is_connected();
}

uint8_t bt_audio_get_remote_wear()
{
    return s_remote_wear;
}

void bt_audio_process()
{
    static bool last_state = false;
    bool connected = bt_audio_is_connected();

    if (connected != last_state) {
        Serial.printf("[BT] %s\n", connected ? "Phone connected" : "Phone disconnected");
        last_state = connected;
    }
}