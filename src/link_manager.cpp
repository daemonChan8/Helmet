#include "link_manager.h"
#include "espnow_audio.h"
#include "pins.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_coexist.h>
#include <math.h>
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const uint8_t s_passenger_mac[6] = { 0xB4, 0xBF, 0xE9, 0xDE, 0xE4, 0x40 };

#define UART_SYNC_BYTE          0xAA
#define UART_MAX_PACKET_SIZE    (2048 + 16)
#define UART_CTRL_MAGIC         0xAB
#define UART_CTRL_BATTERY_STATE 0x03

enum RemoteBatteryState : uint8_t {
    REMOTE_BATT_BLUE     = 0,
    REMOTE_BATT_RED      = 1,
    REMOTE_BATT_CRITICAL = 2,
    REMOTE_BATT_CHARGING = 3,
    REMOTE_BATT_FULL     = 4,
};

static volatile uint8_t s_remote_battery_state = REMOTE_BATT_BLUE;

static void remote_battery_led_update()
{
    static uint32_t last_ms = 0;
    static bool phase = false;
    uint32_t now = millis();
    uint8_t state = s_remote_battery_state;

    if (now - last_ms >= 500) {
        last_ms = now;
        phase = !phase;
    }

    bool red = false;
    bool blue = false;
    switch (state) {
        case REMOTE_BATT_RED:
            red = true;
            break;
        case REMOTE_BATT_CRITICAL:
            red = phase;
            break;
        case REMOTE_BATT_CHARGING:
            red = phase;
            blue = !phase;
            break;
        case REMOTE_BATT_FULL:
            red = true;
            blue = true;
            break;
        case REMOTE_BATT_BLUE:
        default:
            blue = true;
            break;
    }

    digitalWrite(RGB_RED_PIN, red ? HIGH : LOW);
    digitalWrite(RGB_BLUE_PIN, blue ? HIGH : LOW);
}

static uint8_t crc8_calc(const uint8_t* data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc << 1) ^ (crc & 0x80 ? 0x07 : 0x00);
    }
    return crc;
}

// -----------------------------------------------------------------------------
// Noise-shaped ADPCM encode wrapper (see bt_audio.cpp for the full rationale).
// Carries quantization error forward into the next sample instead of just
// discarding it, so distortion gets shaped into a less audible noise floor
// rather than accumulating as correlated grit.
//
// `shape_error` must be a persistent, per-stream state variable. This board
// runs two independent encode streams (UART->ESP-NOW relay, and the local
// intercom mic), so each gets its own shape_error -- never share one across
// streams or they'll contaminate each other.
// -----------------------------------------------------------------------------
static inline uint8_t adpcm_encode_shaped(int16_t sample, adpcm_state_t* s, float* shape_error)
{
    float ideal = (float)sample + *shape_error;
    if (ideal >  32767.0f) ideal =  32767.0f;
    if (ideal < -32768.0f) ideal = -32768.0f;
    int16_t fed_in = (int16_t)ideal;

    uint8_t nib = adpcm_encode(fed_in, s);

    *shape_error = ideal - (float)s->predictor;
    return nib;
}

#define LINK_I2S_PORT   I2S_NUM_0

static void i2s_init_duplex()
{
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    cfg.sample_rate          = ESPNOW_AUDIO_SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 6;
    cfg.dma_buf_len          = 128;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;

    if (i2s_driver_install(LINK_I2S_PORT, &cfg, 0, nullptr) != ESP_OK) {
        Serial.println("[LINK] I2S driver install failed");
        return;
    }
    i2s_pin_config_t pins = {};
    pins.bck_io_num   = PIN_I2S_BCLK;
    pins.ws_io_num    = PIN_I2S_LRCLK;
    pins.data_out_num = PIN_I2S_DOUT;
    pins.data_in_num  = PIN_I2S_DIN;
    if (i2s_set_pin(LINK_I2S_PORT, &pins) != ESP_OK) {
        Serial.println("[LINK] I2S pin config failed");
        return;
    }
    Serial.printf("[LINK] I2S full-duplex ready (%d Hz)\n", ESPNOW_AUDIO_SAMPLE_RATE);
}

static volatile LinkMode s_mode = LINK_MODE_MUSIC;
static volatile uint8_t s_last_battery_pct = 0;
static volatile uint8_t s_last_wear_detected = 0;
static helmet_telemetry_packet_t s_last_passenger_telem = {};
static bool s_have_passenger_telem = false;
static volatile bool s_espnow_ready = false; // set true once ESP-NOW link is up; used by link_manager_publish_telemetry() below
static volatile bool s_emergency_mode = false;
static volatile bool s_emergency_armed = false;
static volatile uint32_t s_remote_armed_ms = 0;

static void broadcast_mode(LinkMode mode);

static constexpr uint8_t EMERGENCY_CLICK_COUNT = 5;
static constexpr uint32_t EMERGENCY_CLICK_WINDOW_MS = 7000;
static constexpr uint32_t EMERGENCY_CLICK_MAX_GAP_MS = 1200;
static constexpr uint32_t EMERGENCY_CONFIRM_WINDOW_MS = 10000;

static void emergency_button_poll()
{
    static bool last_raw = HIGH;
    static bool debounced = HIGH;
    static uint32_t last_change_ms = 0;
    static uint8_t click_count = 0;
    static uint32_t first_click_ms = 0;
    static uint32_t last_click_ms = 0;
    static uint32_t armed_ms = 0;

    bool raw = digitalRead(PIN_BTN_EMERGENCY);
    uint32_t now = millis();
    if (raw != last_raw) last_change_ms = now;
    last_raw = raw;

    if (s_emergency_armed && now - armed_ms >= EMERGENCY_CONFIRM_WINDOW_MS) {
        s_emergency_armed = false;
        click_count = 0;
        Serial.println("[SOS] Arm timeout");
    }

    if ((now - last_change_ms) < 30 || raw == debounced) return;
    debounced = raw;

    if (debounced != LOW) return;

    if (s_emergency_mode) {
        s_emergency_mode = false;
        Serial.println("[SOS] Emergency mode stopped");
        // notify passenger to clear emergency
        {
            espnow_audio_packet_t esp_pkt = {};
            esp_pkt.type = ESPNOW_PKT_CONTROL;
            esp_pkt.seq  = ESPNOW_CTRL_EMERGENCY_OFF;
            esp_pkt.len  = 0;
            for (int i = 0; i < 3; i++) {
                esp_now_send(s_passenger_mac, (uint8_t*)&esp_pkt, ESPNOW_AUDIO_HDR_SIZE);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        return;
    }

    if (s_emergency_armed) {
        s_emergency_armed = false;
        s_emergency_mode = true;
        Serial2.println("SOS");
        Serial.println("[SOS] Confirmed - emergency mode active");
        // notify passenger of emergency
        {
            espnow_audio_packet_t esp_pkt = {};
            esp_pkt.type = ESPNOW_PKT_CONTROL;
            esp_pkt.seq  = ESPNOW_CTRL_EMERGENCY_ON;
            esp_pkt.len  = 0;
            for (int i = 0; i < 3; i++) {
                esp_now_send(s_passenger_mac, (uint8_t*)&esp_pkt, ESPNOW_AUDIO_HDR_SIZE);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        return;
    }

    if (click_count == 0 || now - last_click_ms > EMERGENCY_CLICK_MAX_GAP_MS ||
        now - first_click_ms > EMERGENCY_CLICK_WINDOW_MS) {
        click_count = 1;
        first_click_ms = now;
    } else {
        click_count++;
    }
    last_click_ms = now;

    if (click_count >= EMERGENCY_CLICK_COUNT) {
        s_emergency_armed = true;
        armed_ms = now;
        click_count = 0;
        Serial.println("[SOS] Armed - press once within 10 seconds to confirm");
        link_manager_play_announcement(ANNOUNCE_ARM_CONFIRM);
    }
}

LinkMode link_manager_get_mode() { return s_mode; }

static void send_uart_telemetry_bundle(const helmet_telemetry_bundle_t& bundle)
{
    uint8_t passenger_wear_out = (bundle.passenger_wear_detected == 255)
                                    ? 255
                                    : (bundle.passenger_wear_detected ? 1 : 0);
    Serial.printf("[TEL OUT] UART TEL -> rider=%u wear=%u passenger=%u pass_wear=%u have_passenger=%u\n",
                  bundle.rider_battery_pct,
                  bundle.rider_wear_detected ? 1 : 0,
                  bundle.passenger_battery_pct,
                  passenger_wear_out,
                  s_have_passenger_telem ? 1 : 0);
    Serial2.printf("TEL,%u,%u,%u,%u\n",
                   bundle.rider_battery_pct,
                   bundle.rider_wear_detected ? 1 : 0,
                   bundle.passenger_battery_pct,
                   passenger_wear_out);
}

void link_manager_publish_telemetry(uint8_t battery_pct, uint8_t wear_detected)
{
    s_last_battery_pct = battery_pct;
    s_last_wear_detected = wear_detected;

    helmet_telemetry_bundle_t bundle = {};
    bundle.version = 1;
    bundle.rider_battery_pct = battery_pct;
    bundle.rider_wear_detected = wear_detected;
    bundle.passenger_battery_pct = 255; // sentinel for unknown passenger battery
    bundle.passenger_wear_detected = 255; // sentinel for unknown passenger wear

    if (s_have_passenger_telem) {
        bundle.passenger_battery_pct = s_last_passenger_telem.battery_pct;
        bundle.passenger_wear_detected = s_last_passenger_telem.wear_detected;
    }

    send_uart_telemetry_bundle(bundle);

    if (!s_espnow_ready) return;

    helmet_telemetry_packet_t pkt = {};
    pkt.version = 1;
    pkt.device_id = 0; // Rider
    pkt.battery_pct = battery_pct;
    pkt.wear_detected = wear_detected;

    espnow_audio_packet_t esp_pkt = {};
    esp_pkt.type = ESPNOW_PKT_TELEMETRY;
    esp_pkt.len = sizeof(pkt);
    memcpy(esp_pkt.data, &pkt, sizeof(pkt));
    esp_now_send(s_passenger_mac, (uint8_t*)&esp_pkt, ESPNOW_AUDIO_HDR_SIZE + sizeof(pkt));
}

typedef struct {
    espnow_pkt_type_t type;
    int16_t  predictor;
    uint8_t  step_index;
    uint16_t adpcm_bytes;
    uint8_t  data[ESPNOW_AUDIO_DATA_SIZE];
} out_pkt_item_t;

#define TX_QUEUE_DEPTH   20
static QueueHandle_t     s_espnow_tx_queue = nullptr;
static SemaphoreHandle_t s_send_sem        = nullptr;
static volatile bool     s_last_send_ok    = true;

typedef struct {
    uint16_t n_samples;
    int16_t  pcm[4096];
} music_pcm_item_t;
#define MUSIC_PCM_QUEUE_DEPTH  2
static QueueHandle_t s_music_pcm_queue = nullptr;

typedef struct {
    int16_t  predictor;
    uint8_t  step_index;
    uint16_t len;
    uint8_t  data[ESPNOW_AUDIO_DATA_SIZE];
} intercom_rx_item_t;
#define INTERCOM_RX_QUEUE_DEPTH  6
static QueueHandle_t s_intercom_rx_queue = nullptr;

// =============================================================================
// Local announcements (SOS arm-confirm / sending-help beeps, volume-level
// feedback, etc.)
//
// These share the SAME speaker/I2S output as music and intercom, so they
// cannot use their own independent I2S write - only speaker_writer_task is
// allowed to call i2s_write() on LINK_I2S_PORT. Instead:
//
//   1. link_manager_play_announcement(id) - called from ANY task (including
//      emergency_sms.cpp's button polling). Just pushes an id onto a tiny
//      request queue and returns immediately - never blocks the caller.
//   2. announce_decode_task - decoupled from real-time audio entirely. Reads
//      the request, decodes/generates the PCM, and chunks it into
//      s_announce_queue. Free to block/take its time; nothing depends on it
//      being fast.
//   3. speaker_writer_task - drains s_announce_queue FIRST, ahead of music/
//      intercom, so announcements interrupt/prioritize over background
//      audio. While an announcement is playing (and briefly after), music/
//      intercom output is ducked rather than muted.
//
// TODO: load_announcement_pcm() below is a PLACEHOLDER. It generates beep
// tones instead of decoding your actual wav files, because the real storage/
// decode mechanism for lib/sound wavs on this board hasn't been confirmed
// yet (embedded array? SPIFFS/LittleFS? SD card?). Once known, replace the
// body of load_announcement_pcm() with real wav decoding - nothing else in
// this file needs to change.
// =============================================================================

#define ANNOUNCE_CHUNK_SAMPLES        1024   // size of each piece handed to speaker_writer_task
#define ANNOUNCE_QUEUE_DEPTH             8   // chunk queue (drained by speaker_writer_task)
#define ANNOUNCE_REQUEST_QUEUE_DEPTH      4  // pending "play this announcement" requests
#define ANNOUNCE_SCRATCH_MAX_SAMPLES   8000  // TODO: size to your longest real wav once known
#define ANNOUNCE_DUCK_HOLD_MS           350  // keep music/intercom ducked this long after last chunk
#define ANNOUNCE_DUCK_GAIN_Q8             48 // ~19% of normal volume while ducked

typedef struct {
    uint16_t n_samples;
    int16_t  pcm[ANNOUNCE_CHUNK_SAMPLES];
} announce_pcm_item_t;

static QueueHandle_t     s_announce_queue         = nullptr; // chunk queue -> speaker_writer_task
static QueueHandle_t     s_announce_request_queue = nullptr; // id queue -> announce_decode_task
static volatile uint32_t s_last_announce_ms       = 0;

// TODO: replace with real wav decoding. This placeholder generates a sine
// beep so you can verify the queueing/ducking pipeline actually works while
// the real wav source is still unconfirmed.
static uint16_t generate_beep(int16_t* out, uint16_t max_samples, float freq_hz, uint16_t duration_ms)
{
    uint32_t n = (uint32_t)((uint64_t)ESPNOW_AUDIO_SAMPLE_RATE * duration_ms / 1000);
    if (n > max_samples) n = max_samples;
    for (uint32_t i = 0; i < n; i++) {
        float t = (float)i / (float)ESPNOW_AUDIO_SAMPLE_RATE;
        out[i] = (int16_t)(6000.0f * sinf(2.0f * (float)M_PI * freq_hz * t));
    }
    return (uint16_t)n;
}

// TODO: replace with real wav decoding, same as generate_beep(). Plays
// `count` short beeps back-to-back (with silent gaps between), used for
// volume-level feedback -- e.g. 1 beep = lowest level, 4 beeps = highest.
// Fixed pitch across all counts so the feedback reads as "how many", not
// "what note", and stays clearly distinct from the arm/sending tones.
static uint16_t generate_beep_sequence(int16_t* out, uint16_t max_samples,
                                        float freq_hz, uint16_t beep_ms,
                                        uint16_t gap_ms, uint8_t count)
{
    uint16_t total = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (i > 0) {
            uint32_t gap_n = (uint32_t)((uint64_t)ESPNOW_AUDIO_SAMPLE_RATE * gap_ms / 1000);
            if (total + gap_n > max_samples) break;
            memset(out + total, 0, gap_n * sizeof(int16_t));
            total += gap_n;
        }
        uint16_t remaining = max_samples - total;
        uint16_t n = generate_beep(out + total, remaining, freq_hz, beep_ms);
        total += n;
        if (n == 0) break; // ran out of scratch space
    }
    return total;
}

// TODO: this is the function to replace with real wav decoding.
static uint16_t load_announcement_pcm(AnnouncementId id, int16_t* out_pcm, uint16_t max_samples)
{
    switch (id) {
        case ANNOUNCE_ARM_CONFIRM: return generate_beep(out_pcm, max_samples, 1200.0f, 150); // short, high beep
        case ANNOUNCE_SENDING:     return generate_beep(out_pcm, max_samples, 700.0f, 400);  // longer, lower beep

        // Volume feedback: N beeps = level N of 4 (1 = quietest, 4 = loudest).
        case ANNOUNCE_VOLUME_1: return generate_beep_sequence(out_pcm, max_samples, 900.0f, 60, 40, 1);
        case ANNOUNCE_VOLUME_2: return generate_beep_sequence(out_pcm, max_samples, 900.0f, 60, 40, 2);
        case ANNOUNCE_VOLUME_3: return generate_beep_sequence(out_pcm, max_samples, 900.0f, 60, 40, 3);
        case ANNOUNCE_VOLUME_4: return generate_beep_sequence(out_pcm, max_samples, 900.0f, 60, 40, 4);

        default: return 0;
    }
}

static void announce_decode_task(void* /*param*/)
{
    static int16_t scratch[ANNOUNCE_SCRATCH_MAX_SAMPLES];
    uint8_t id_raw;

    for (;;) {
        if (xQueueReceive(s_announce_request_queue, &id_raw, portMAX_DELAY) != pdTRUE) continue;
        Serial.printf("[LINK] announce_decode_task received id=%u\n", id_raw);
        AnnouncementId id = (AnnouncementId)id_raw;

        uint16_t total = load_announcement_pcm(id, scratch, ANNOUNCE_SCRATCH_MAX_SAMPLES);
        if (total == 0) {
            Serial.println("[LINK] Announcement produced 0 samples - skipping");
            continue;
        }

        uint16_t offset = 0;
        while (offset < total) {
            announce_pcm_item_t item;
            uint16_t chunk = min((uint16_t)ANNOUNCE_CHUNK_SAMPLES, (uint16_t)(total - offset));
            memcpy(item.pcm, scratch + offset, chunk * sizeof(int16_t));
            item.n_samples = chunk;
            // Fine to block here - this task is decoupled from both button
            // polling and the real-time I2S writer. Blocking just paces
            // decode work against playback, nothing else.
            xQueueSend(s_announce_queue, &item, portMAX_DELAY);
            Serial.printf("[LINK] announce_decode_task queued chunk offset=%u chunk=%u\n", offset, chunk);
            offset += chunk;
        }
    }
}

// Public API (declare in link_manager.h - see note above). Non-blocking:
// safe to call from emergency_sms.cpp's time-sensitive button polling.
void link_manager_play_announcement(AnnouncementId id)
{
    if (!s_announce_request_queue) {
        Serial.println("[LINK] announce request queue not initialized");
        return;
    }
    uint8_t id_raw = (uint8_t)id;
    BaseType_t res = xQueueSend(s_announce_request_queue, &id_raw, pdMS_TO_TICKS(10)); // small wait
    if (res != pdTRUE) {
        Serial.printf("[LINK] announce request queue full - drop id=%u\n", id_raw);
    } else {
        Serial.printf("[LINK] announce request queued id=%u\n", id_raw);
    }
}

static void on_sent(const uint8_t* mac, esp_now_send_status_t status)
{
    s_last_send_ok = (status == ESP_NOW_SEND_SUCCESS);
    xSemaphoreGive(s_send_sem);
}

static void on_recv(const uint8_t* mac, const uint8_t* incoming, int len)
{
    if (len < (int)ESPNOW_AUDIO_HDR_SIZE) return;
    bool from_expected_passenger = (memcmp(mac, s_passenger_mac, 6) == 0);
    const espnow_audio_packet_t* pkt = (const espnow_audio_packet_t*)incoming;

    Serial.printf("[ESP-NOW] recv from %02X:%02X:%02X:%02X:%02X:%02X type=%u len=%d expected=%u\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], pkt->type, len,
                  from_expected_passenger ? 1 : 0);

    if (!from_expected_passenger) {
        // If a telemetry packet arrives from any source, accept it and
        // remember that MAC as the passenger peer.
        if (pkt->type == ESPNOW_PKT_TELEMETRY && len == (int)(ESPNOW_AUDIO_HDR_SIZE + sizeof(helmet_telemetry_packet_t))) {
            memcpy((void*)s_passenger_mac, mac, 6);
            Serial.printf("[ESP-NOW] updated expected passenger MAC to %02X:%02X:%02X:%02X:%02X:%02X\n",
                          s_passenger_mac[0], s_passenger_mac[1], s_passenger_mac[2],
                          s_passenger_mac[3], s_passenger_mac[4], s_passenger_mac[5]);
            from_expected_passenger = true;
        }
    }

    switch (pkt->type) {
        case ESPNOW_PKT_CONTROL:
            // Passenger sends a control request. Driver is authoritative for
            // mode changes, and both boards must respond to emergency toggles.
            if (pkt->seq == ESPNOW_CTRL_MODE_INTERCOM || pkt->seq == ESPNOW_CTRL_MODE_MUSIC) {
                if (s_emergency_armed || s_emergency_mode) break;
                s_mode = (pkt->seq == ESPNOW_CTRL_MODE_INTERCOM) ? LINK_MODE_INTERCOM
                                                                  : LINK_MODE_MUSIC;
                Serial.printf("[LINK] Mode -> %s (from Passenger)\n",
                              s_mode == LINK_MODE_MUSIC ? "MUSIC" : "INTERCOM");
                broadcast_mode(s_mode);
            } else if (pkt->seq == ESPNOW_CTRL_EMERGENCY_ON) {
                if (!s_emergency_mode) {
                    s_emergency_mode = true;
                    Serial.println("[SOS] Remote emergency ON (state mirror only - Passenger triggers "
                                    "its own SMS locally, this board must never write SOS on its "
                                    "behalf)");
                }
            } else if (pkt->seq == ESPNOW_CTRL_EMERGENCY_ARM) {
                // Passenger has armed - stop background audio (mirror local arm)
                if (!s_emergency_armed) {
                    s_emergency_armed = true;
                    s_remote_armed_ms = millis();
                    Serial.println("[SOS] Remote armed - entering armed state");
                    link_manager_play_announcement(ANNOUNCE_ARM_CONFIRM);
                }
            } else if (pkt->seq == ESPNOW_CTRL_EMERGENCY_OFF) {
                if (s_emergency_mode) {
                    s_emergency_mode = false;
                    Serial.println("[SOS] Remote emergency OFF");
                }
            }
            break;

        case ESPNOW_PKT_INTERCOM:
            if (!s_intercom_rx_queue) return;
            if (pkt->len == 0 || pkt->len > ESPNOW_AUDIO_DATA_SIZE) return;
            {
                intercom_rx_item_t item;
                item.predictor  = pkt->predictor;
                item.step_index = pkt->step_index;
                item.len        = pkt->len;
                memcpy(item.data, pkt->data, pkt->len);
                xQueueSend(s_intercom_rx_queue, &item, 0);
            }
            break;

        case ESPNOW_PKT_TELEMETRY:
            if (pkt->len != sizeof(helmet_telemetry_packet_t)) break;
            {
                helmet_telemetry_packet_t telem = {};
                memcpy(&telem, pkt->data, sizeof(telem));
                s_last_passenger_telem = telem;
                s_have_passenger_telem = true;
                Serial.printf("[TEL IN] passenger device=%u batt=%u wear=%u len=%u from_expected=%u\n",
                              telem.device_id, telem.battery_pct,
                              telem.wear_detected, pkt->len,
                              from_expected_passenger ? 1 : 0);
            }
            break;
    }
}

static HardwareSerial* s_uart = nullptr;

static void uart_rx_task(void* /*param*/)
{
    static uint8_t buf[UART_MAX_PACKET_SIZE];
    uint16_t idx = 0;
    uint8_t control_frame[3] = {};
    uint8_t control_idx = 0;

    adpcm_state_t dec_state = {0, 0};
    adpcm_state_t fwd_enc   = {0, 0};
    uint16_t      fwd_seq   = 0;

    // Per-stream shaping state for the relay re-encode, kept separate from
    // the mic and intercom-playback streams elsewhere in this file.
    float fwd_shape_error = 0.0f;

    while (true) {
        if (!s_uart || s_uart->available() == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        uint8_t byte = s_uart->read();

        if (control_idx != 0) {
            control_frame[control_idx++] = byte;
            if (control_idx == sizeof(control_frame)) {
                if (control_frame[1] == UART_CTRL_BATTERY_STATE &&
                    control_frame[2] <= REMOTE_BATT_FULL) {
                    s_remote_battery_state = control_frame[2];
                }
                control_idx = 0;
            }
            continue;
        }

        if (idx == 0 && byte == UART_CTRL_MAGIC) {
            control_frame[0] = byte;
            control_idx = 1;
            continue;
        }

        if (idx == 0 && byte != UART_SYNC_BYTE) continue;

        buf[idx++] = byte;
        if (idx < 8) continue;

        uint16_t payload_len = ((uint16_t)buf[6] << 8) | buf[7];
        if (idx == 8 && payload_len > 2048) {
            Serial.printf("[LINK][UART] bad payload len %u\n", payload_len);
            idx = 0;
            continue;
        }

        uint16_t expected_size = 9 + payload_len;
        if (idx < expected_size) continue;

        uint16_t crc_len  = 7 + payload_len;
        uint8_t  crc_calc = crc8_calc(buf + 1, crc_len);
        uint8_t  crc_recv = buf[expected_size - 1];
        if (crc_calc != crc_recv) {
            Serial.printf("[LINK][UART] CRC mismatch\n");
            idx = 0;
            continue;
        }

        if (s_emergency_armed || s_emergency_mode) {
            idx = 0;
            continue;
        }

        if (s_mode == LINK_MODE_MUSIC) {
            int16_t predictor  = (int16_t)(((uint16_t)buf[3] << 8) | buf[4]);
            uint8_t step_index = buf[5];
            if (step_index > 88) step_index = 88;

            dec_state.predictor  = predictor;
            dec_state.step_index = step_index;

            static music_pcm_item_t pcm_item;
            pcm_item.n_samples = 0;
            for (uint16_t i = 0; i < payload_len; i++) {
                uint8_t b = buf[8 + i];
                int16_t raw0 = adpcm_decode(b & 0x0F, &dec_state);
                int16_t raw1 = adpcm_decode((b >> 4) & 0x0F, &dec_state);
                // Feeds both this board's own speaker (via music_pcm_item_t)
                // and the re-encode below.
                pcm_item.pcm[pcm_item.n_samples++] = raw0;
                pcm_item.pcm[pcm_item.n_samples++] = raw1;
            }

            static music_pcm_item_t discard;
            while (uxQueueMessagesWaiting(s_music_pcm_queue) > 0)
                xQueueReceive(s_music_pcm_queue, &discard, 0);
            xQueueSend(s_music_pcm_queue, &pcm_item, 0);

            uint16_t n = 0;
            while (n < pcm_item.n_samples) {
                out_pkt_item_t out = {};   // zeroed each iteration -- see below
                out.type       = ESPNOW_PKT_MUSIC;
                out.predictor  = (int16_t)fwd_enc.predictor;
                out.step_index = (uint8_t)fwd_enc.step_index;
                uint16_t chunk_samples = min((uint16_t)ESPNOW_AUDIO_SAMPLES_PER_PKT,
                                              (uint16_t)(pcm_item.n_samples - n));
                for (uint16_t s = 0; s < chunk_samples; s++) {
                    uint8_t  nib      = adpcm_encode_shaped(pcm_item.pcm[n + s], &fwd_enc, &fwd_shape_error);
                    uint16_t byte_idx = s >> 1;
                    if (s & 1) out.data[byte_idx] |= (nib << 4);
                    else       out.data[byte_idx]  =  nib;
                }
                out.adpcm_bytes = (chunk_samples + 1) / 2;
                n += chunk_samples;
                (void)fwd_seq;
                xQueueSend(s_espnow_tx_queue, &out, 0);
            }
        }

        idx = 0;
    }
}

#define INTERCOM_MIC_GAIN      2
#define INTERCOM_NOISE_GATE   800
#define INTERCOM_GATE_HOLD_MS 250

static void mic_task(void* /*param*/)
{
    static adpcm_state_t enc = {0, 0};
    static float          shape_error = 0.0f;   // own noise-shaping state, separate from the relay
    static out_pkt_item_t pkt;
    static uint16_t n_samples = 0;
    static bool     pkt_open  = false;
    static int32_t  raw[64];
    static uint32_t gate_hold = 0;
    static const uint32_t GATE_HOLD_SAMPLES =
        (uint32_t)(ESPNOW_AUDIO_SAMPLE_RATE * INTERCOM_GATE_HOLD_MS / 1000);

    while (true) {
        size_t bytes = 0;
        esp_err_t err = i2s_read(LINK_I2S_PORT, raw, sizeof(raw), &bytes, pdMS_TO_TICKS(50));
        if (err != ESP_OK || bytes == 0) continue;
        if (s_emergency_armed || s_emergency_mode || s_mode != LINK_MODE_INTERCOM) continue;

        uint32_t n_frames = bytes / (2u * sizeof(int32_t));
        for (uint32_t i = 0; i < n_frames; i++) {
            int16_t sl = (int16_t)(raw[i * 2]     >> 16);
            int16_t sr = (int16_t)(raw[i * 2 + 1] >> 16);
            int32_t al = sl < 0 ? -(int32_t)sl : (int32_t)sl;
            int32_t ar = sr < 0 ? -(int32_t)sr : (int32_t)sr;
            int16_t sample = (ar >= al) ? sr : sl;

            int32_t abs_s = sample < 0 ? -(int32_t)sample : (int32_t)sample;
            if (abs_s >= INTERCOM_NOISE_GATE) gate_hold = GATE_HOLD_SAMPLES;
            else if (gate_hold > 0)           --gate_hold;
            else                              sample = 0;

            int32_t gained = (int32_t)sample * INTERCOM_MIC_GAIN;
            if (gained >  32767) gained =  32767;
            if (gained < -32768) gained = -32768;
            sample = (int16_t)gained;

            if (!pkt_open) {
                pkt.predictor  = (int16_t)enc.predictor;
                pkt.step_index = (uint8_t)enc.step_index;
                n_samples      = 0;
                pkt_open       = true;
            }

            uint8_t  nibble   = adpcm_encode_shaped(sample, &enc, &shape_error);
            uint16_t byte_idx = n_samples >> 1;
            if (n_samples & 1) pkt.data[byte_idx] |= (nibble << 4);
            else               pkt.data[byte_idx]  =  nibble;
            n_samples++;

            if (n_samples >= ESPNOW_AUDIO_SAMPLES_PER_PKT) {
                pkt.type        = ESPNOW_PKT_INTERCOM;
                pkt.adpcm_bytes = ESPNOW_AUDIO_DATA_SIZE;
                xQueueSend(s_espnow_tx_queue, &pkt, 0);
                pkt_open = false;
            }
        }
    }
}

static void espnow_tx_task(void* /*param*/)
{
    out_pkt_item_t         item;
    espnow_audio_packet_t  pkt;
    uint16_t seq_music    = 0;
    uint16_t seq_intercom = 0;
    uint8_t  fail_streak  = 0;

    while (true) {
        if (xQueueReceive(s_espnow_tx_queue, &item, portMAX_DELAY) != pdTRUE) continue;
        if (!s_espnow_ready) continue;

        pkt.type       = item.type;
        pkt.seq        = (item.type == ESPNOW_PKT_MUSIC) ? seq_music++ : seq_intercom++;
        pkt.len        = item.adpcm_bytes;
        pkt.predictor  = item.predictor;
        pkt.step_index = item.step_index;
        memcpy(pkt.data, item.data, item.adpcm_bytes);
        size_t pkt_len = ESPNOW_AUDIO_HDR_SIZE + item.adpcm_bytes;

        if (fail_streak >= 20) {
            out_pkt_item_t tmp;
            while (xQueueReceive(s_espnow_tx_queue, &tmp, 0) == pdTRUE) {}
            vTaskDelay(pdMS_TO_TICKS(2000));
            fail_streak = 0;
        }

        xSemaphoreTake(s_send_sem, pdMS_TO_TICKS(150));
        esp_err_t ret = esp_now_send(s_passenger_mac, (uint8_t*)&pkt, pkt_len);
        if (ret != ESP_OK) {
            s_last_send_ok = false;
            xSemaphoreGive(s_send_sem);
        }

        uint32_t samples = (uint32_t)item.adpcm_bytes * 2;
        uint32_t pace_ms  = (samples * 1000UL) / ESPNOW_AUDIO_SAMPLE_RATE;
        if (pace_ms < 3) pace_ms = 3;
        vTaskDelay(pdMS_TO_TICKS(pace_ms));

        if (!s_last_send_ok) { if (fail_streak < 255) fail_streak++; }
        else                 { fail_streak = 0; }
    }
}

// =============================================================================
// Speaker volume - short-press cycles through 4 fixed levels (looping).
// Gains are Q8 fixed point (256 = 1.0x) so scaling is just a multiply+shift.
// =============================================================================
static const int32_t VOLUME_GAIN_Q8[4] = { 64, 128, 192, 256 }; // 25% / 50% / 75% / 100%
static const int32_t SOFTWARE_OUTPUT_GAIN_Q8 = 192; // 256 = 1.0x; increase to 320 for +25% gain
static const int32_t EMERGENCY_OUTPUT_GAIN_Q8 = 512; // fixed emergency volume; ignores the volume button
static const int32_t TONE_SMOOTH_Q8 = 192; // a touch softer for speech clarity
static const int32_t LOW_SHELF_GAIN_Q8 = 272; // very mild low-end support only
static const int32_t TREBLE_CUT_Q8 = 160; // neutral treble, no extra harshness shaping
static volatile uint8_t s_volume_idx = 3;  // start at 100%
static int32_t s_tone_state = 0;

// Bright-but-not-sharp shaping for close-fitting helmets: keeps clarity without
// making the audio edgy, gritty, or overly bassy at high volumes.
static inline int16_t apply_tone_shaping(int16_t sample)
{
    int32_t filtered = ((s_tone_state * TONE_SMOOTH_Q8) + ((int32_t)sample * (256 - TONE_SMOOTH_Q8))) >> 8;
    s_tone_state = filtered;

    // Very mild low-end support, but without over-driving bass transients.
    int32_t lifted = ((filtered * LOW_SHELF_GAIN_Q8) >> 8);

    // Slight treble soften so the sound stays present but not sharp.
    int32_t softened = ((lifted * TREBLE_CUT_Q8) >> 8);

    // Light limiter for clean playback at higher volume levels.
    const int32_t peak = 20000;
    if (softened > peak) softened = peak;
    if (softened < -peak) softened = -peak;
    return (int16_t)softened;
}

// Applies the user's volume level, and additionally ducks (further attenuates)
// if `duck` is true - used to lower music/intercom while an announcement is
// playing (or has just finished, see ANNOUNCE_DUCK_HOLD_MS). Announcements
// themselves are never ducked (always called with duck=false).
static inline int16_t apply_output_gain(int16_t sample, bool duck)
{
    sample = apply_tone_shaping(sample);
    int32_t v = ((int32_t)sample * VOLUME_GAIN_Q8[s_volume_idx] * SOFTWARE_OUTPUT_GAIN_Q8) >> 16;
    if (duck) v = (v * ANNOUNCE_DUCK_GAIN_Q8) >> 8;
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

static inline int16_t apply_emergency_gain(int16_t sample)
{
    sample = apply_tone_shaping(sample);
    int32_t v = ((int32_t)sample * EMERGENCY_OUTPUT_GAIN_Q8 * SOFTWARE_OUTPUT_GAIN_Q8) >> 16;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

static void speaker_writer_task(void* /*param*/)
{
    static const int32_t silence[128 * 2] = {};

    static music_pcm_item_t   music_item;
    static intercom_rx_item_t ic_item;
    static announce_pcm_item_t ann_item;
    static int32_t       stereo_buf[4096 * 2];

    while (true) {
        if (s_emergency_mode) {
            static uint32_t heartbeat_sample = 0;
            const uint32_t samples_per_beat = ESPNOW_AUDIO_SAMPLE_RATE;
            const uint32_t tone_samples = ESPNOW_AUDIO_SAMPLE_RATE / 8;

            for (uint16_t j = 0; j < ANNOUNCE_CHUNK_SAMPLES; j++) {
                uint32_t phase = heartbeat_sample % samples_per_beat;
                int16_t sample = 0;
                if (phase < tone_samples) {
                    float t = (float)phase / (float)ESPNOW_AUDIO_SAMPLE_RATE;
                    sample = (int16_t)(5000.0f * sinf(2.0f * (float)M_PI * 700.0f * t));
                }
                int32_t v = (int32_t)apply_emergency_gain(sample) << 16;
                stereo_buf[j * 2] = v;
                stereo_buf[j * 2 + 1] = v;
                heartbeat_sample++;
            }

            size_t w = 0;
            i2s_write(LINK_I2S_PORT, stereo_buf,
                      ANNOUNCE_CHUNK_SAMPLES * 2 * sizeof(int32_t), &w,
                      pdMS_TO_TICKS(100));
            continue;
        }

        // Announcements take priority over music/intercom, and are never
        // ducked themselves - only the background audio is.
        if (xQueueReceive(s_announce_queue, &ann_item, 0) == pdTRUE) {
            Serial.printf("[LINK] speaker_writer got announce chunk n_samples=%u\n", ann_item.n_samples);
            for (uint16_t j = 0; j < ann_item.n_samples; j++) {
                int32_t v = (int32_t)apply_output_gain(ann_item.pcm[j], false) << 16;
                stereo_buf[j * 2]     = v;
                stereo_buf[j * 2 + 1] = v;
            }
            size_t w = 0;
            i2s_write(LINK_I2S_PORT, stereo_buf,
                      ann_item.n_samples * 2 * sizeof(int32_t), &w, pdMS_TO_TICKS(100));
            s_last_announce_ms = millis();
            continue; // re-check for more announcement chunks before falling back
        }

        bool duck = (millis() - s_last_announce_ms) < ANNOUNCE_DUCK_HOLD_MS;
        LinkMode mode = s_mode;

        if (s_emergency_armed) {
            size_t w = 0;
            i2s_write(LINK_I2S_PORT, silence, sizeof(silence), &w, 0);
            vTaskDelay(pdMS_TO_TICKS(4));
            continue;
        }

        if (mode == LINK_MODE_MUSIC) {
            if (xQueueReceive(s_music_pcm_queue, &music_item, pdMS_TO_TICKS(4)) == pdTRUE) {
                for (uint16_t j = 0; j < music_item.n_samples; j++) {
                    int32_t v = (int32_t)apply_output_gain(music_item.pcm[j], duck) << 16;
                    stereo_buf[j * 2]     = v;
                    stereo_buf[j * 2 + 1] = v;
                }
                size_t w = 0;
                i2s_write(LINK_I2S_PORT, stereo_buf,
                          music_item.n_samples * 2 * sizeof(int32_t), &w, pdMS_TO_TICKS(100));
            } else {
                size_t w = 0;
                i2s_write(LINK_I2S_PORT, silence, sizeof(silence), &w, 0);
            }
        } else {
            if (xQueueReceive(s_intercom_rx_queue, &ic_item, pdMS_TO_TICKS(4)) == pdTRUE) {
                adpcm_state_t dec = { ic_item.predictor, ic_item.step_index };
                uint32_t out = 0;
                for (uint16_t i = 0; i < ic_item.len; i++) {
                    int16_t raw0 = adpcm_decode(ic_item.data[i] & 0x0F,        &dec);
                    int16_t raw1 = adpcm_decode((ic_item.data[i] >> 4) & 0x0F, &dec);
                    int16_t s0 = apply_output_gain(raw0, duck);
                    int16_t s1 = apply_output_gain(raw1, duck);
                    int32_t v0 = (int32_t)s0 << 16, v1 = (int32_t)s1 << 16;
                    stereo_buf[out++] = v0; stereo_buf[out++] = v0;
                    stereo_buf[out++] = v1; stereo_buf[out++] = v1;
                }
                intercom_rx_item_t discard;
                while (uxQueueMessagesWaiting(s_intercom_rx_queue) > 2)
                    xQueueReceive(s_intercom_rx_queue, &discard, 0);

                size_t w = 0;
                i2s_write(LINK_I2S_PORT, stereo_buf, out * sizeof(int32_t), &w, pdMS_TO_TICKS(100));
            } else {
                size_t w = 0;
                i2s_write(LINK_I2S_PORT, silence, sizeof(silence), &w, 0);
            }
        }
    }
}

static void broadcast_mode(LinkMode mode)
{
    espnow_audio_packet_t pkt = {};
    pkt.type = ESPNOW_PKT_CONTROL;
    pkt.seq  = (mode == LINK_MODE_MUSIC) ? ESPNOW_CTRL_MODE_MUSIC : ESPNOW_CTRL_MODE_INTERCOM;
    pkt.len  = 0;
    for (int i = 0; i < 3; i++) {
        esp_now_send(s_passenger_mac, (uint8_t*)&pkt, ESPNOW_AUDIO_HDR_SIZE);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

static void set_mode(LinkMode mode)
{
    if (s_mode == mode) return;
    s_mode = mode;
    Serial.printf("[LINK] Mode -> %s\n", mode == LINK_MODE_MUSIC ? "MUSIC" : "INTERCOM");
    broadcast_mode(mode);
}

// =============================================================================
// Button (debounced, active-low)
//   long press (held >= LONG_PRESS_MS)  -> toggle MUSIC/INTERCOM mode
//   short press (released before that)  -> cycle volume, 4 levels, looping
// =============================================================================
#define LONG_PRESS_MS   600

static void button_poll()
{
    static bool     last_reading   = HIGH;
    static bool     debounced      = HIGH;
    static uint32_t last_change_ms = 0;
    static uint32_t press_start_ms = 0;
    static bool     press_active   = false;
    static bool     long_fired     = false;
    const uint32_t  DEBOUNCE_MS    = 40;

    if (s_emergency_armed || s_emergency_mode) return;

    bool reading = digitalRead(PIN_BTN_MULTIFUNCTION);
    uint32_t now = millis();

    if (reading != last_reading) last_change_ms = now;

    if ((now - last_change_ms) > DEBOUNCE_MS && reading != debounced) {
        debounced = reading;
        if (debounced == LOW) {
            // Press started - just arm it; the action fires either while
            // still held (long press) or on release (short press) below.
            press_active   = true;
            long_fired     = false;
            press_start_ms = now;
        } else {
            // Released - only a short press produces an action here; a
            // long press already fired its action while still held.
            if (press_active && !long_fired) {
                s_volume_idx = (uint8_t)((s_volume_idx + 1) % 4);
                Serial.printf("[LINK] Volume -> level %u/4\n", s_volume_idx + 1);
            }
            press_active = false;
        }
    }

    // Long-press fires once, the moment the hold threshold is crossed,
    // without waiting for release - feels more responsive on a mode switch.
    if (press_active && !long_fired && (now - press_start_ms) >= LONG_PRESS_MS) {
        long_fired = true;
        set_mode(s_mode == LINK_MODE_MUSIC ? LINK_MODE_INTERCOM : LINK_MODE_MUSIC);
    }

    last_reading = reading;
}

static void espnow_init()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_AUDIO_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(80);
    esp_coex_preference_set(ESP_COEX_PREFER_BT);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[LINK] ESP-NOW init failed");
        return;
    }
    esp_now_register_send_cb(on_sent);
    esp_now_register_recv_cb(on_recv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, s_passenger_mac, 6);
    peer.channel = ESPNOW_AUDIO_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[LINK] Failed to add Passenger peer");
        return;
    }
    s_espnow_ready = true;
    Serial.printf("[LINK] ESP-NOW ready, driver MAC=%s, expected passenger MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  WiFi.macAddress().c_str(),
                  s_passenger_mac[0], s_passenger_mac[1], s_passenger_mac[2],
                  s_passenger_mac[3], s_passenger_mac[4], s_passenger_mac[5]);
}

void link_manager_init()
{
    pinMode(PIN_BTN_MULTIFUNCTION, INPUT_PULLUP);
    pinMode(PIN_BTN_EMERGENCY, INPUT_PULLUP);
    pinMode(RGB_RED_PIN, OUTPUT);
    pinMode(RGB_BLUE_PIN, OUTPUT);
    digitalWrite(RGB_RED_PIN, LOW);
    digitalWrite(RGB_BLUE_PIN, HIGH);

    s_espnow_tx_queue    = xQueueCreate(TX_QUEUE_DEPTH, sizeof(out_pkt_item_t));
    s_music_pcm_queue    = xQueueCreate(MUSIC_PCM_QUEUE_DEPTH, sizeof(music_pcm_item_t));
    s_intercom_rx_queue  = xQueueCreate(INTERCOM_RX_QUEUE_DEPTH, sizeof(intercom_rx_item_t));
    s_send_sem           = xSemaphoreCreateBinary();
    xSemaphoreGive(s_send_sem);

    s_announce_queue         = xQueueCreate(ANNOUNCE_QUEUE_DEPTH, sizeof(announce_pcm_item_t));
    s_announce_request_queue = xQueueCreate(ANNOUNCE_REQUEST_QUEUE_DEPTH, sizeof(uint8_t));

    i2s_init_duplex();
    espnow_init();

    s_uart = &Serial2;
    s_uart->setRxBufferSize(4096);
    s_uart->begin(UART_BAUD_RATE, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
    Serial.println("[LINK] UART receiver ready");

    xTaskCreatePinnedToCore(speaker_writer_task,  "link_spk",      8192, nullptr, 6, nullptr, 1);
    xTaskCreatePinnedToCore(uart_rx_task,         "link_uart_rx",  8192, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(mic_task,             "link_mic",      4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(espnow_tx_task,       "link_tx",       4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(announce_decode_task, "link_announce", 4096, nullptr, 2, nullptr, 0);

    Serial.printf("[LINK] Ready - mode = %s\n", s_mode == LINK_MODE_MUSIC ? "MUSIC" : "INTERCOM");
}

void link_manager_process()
{
    button_poll();
    emergency_button_poll();
    remote_battery_led_update();

    static uint32_t last_ms = 0;
    static uint32_t last_mode_sync_ms = 0;
    uint32_t now = millis();

    if (now - last_mode_sync_ms >= 1000) {
        last_mode_sync_ms = now;
        broadcast_mode(s_mode);
    }

    if (now - last_ms >= 3000) {
        last_ms = now;
        Serial.printf("[LINK] mode=%s espnow_ready=%d music_q=%u intercom_q=%u tx_q=%u vol=%u/4\n",
            s_mode == LINK_MODE_MUSIC ? "MUSIC" : "INTERCOM",
            (int)s_espnow_ready,
            (unsigned)uxQueueMessagesWaiting(s_music_pcm_queue),
            (unsigned)uxQueueMessagesWaiting(s_intercom_rx_queue),
            (unsigned)uxQueueMessagesWaiting(s_espnow_tx_queue),
            (unsigned)(s_volume_idx + 1));
    }
}