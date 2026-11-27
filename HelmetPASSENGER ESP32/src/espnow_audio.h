//passenger
#pragma once
#include <stdint.h>

#define ESPNOW_AUDIO_CHANNEL          1
#define ESPNOW_AUDIO_HDR_SIZE         8    
#define ESPNOW_AUDIO_DATA_SIZE        242  
#define ESPNOW_AUDIO_SAMPLES_PER_PKT  (ESPNOW_AUDIO_DATA_SIZE * 2)  

#define ESPNOW_AUDIO_SAMPLE_RATE      32000
#define ESPNOW_AUDIO_BITS             16

enum espnow_pkt_type_t : uint8_t {
    ESPNOW_PKT_MUSIC     = 0,
    ESPNOW_PKT_INTERCOM  = 1,
    ESPNOW_PKT_CONTROL   = 2,
    ESPNOW_PKT_TELEMETRY = 3,
};

enum espnow_ctrl_t : uint16_t {
    ESPNOW_CTRL_MODE_MUSIC    = 0,
    ESPNOW_CTRL_MODE_INTERCOM = 1,
    ESPNOW_CTRL_EMERGENCY_ON  = 2,
    ESPNOW_CTRL_EMERGENCY_OFF = 3,
    ESPNOW_CTRL_EMERGENCY_ARM = 6,
    ESPNOW_CTRL_GPS_REQUEST   = 4,
    ESPNOW_CTRL_GPS_RESPONSE  = 5,
};

#pragma pack(push, 1)
typedef struct {
    uint8_t  type;
    uint16_t seq;
    uint16_t len;
    int16_t  predictor;
    uint8_t  step_index;
    uint8_t  data[ESPNOW_AUDIO_DATA_SIZE];
} espnow_audio_packet_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t version;
    uint8_t device_id;
    uint8_t battery_pct;
    uint8_t wear_detected;
} helmet_telemetry_packet_t;
#pragma pack(pop)

static const int16_t adpcm_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34,
    37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143,
    157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494,
    544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552,
    1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428,
    4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635,
    13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

static const int8_t adpcm_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

typedef struct { int32_t predictor; int32_t step_index; } adpcm_state_t;

static inline uint8_t adpcm_encode(int16_t sample, adpcm_state_t* s)
{
    int32_t diff   = (int32_t)sample - s->predictor;
    int32_t step   = adpcm_step_table[s->step_index];
    uint8_t nibble = 0;
    if (diff < 0) { nibble = 8; diff = -diff; }
    int32_t vpdiff = step >> 3;
    if (diff >= step)        { nibble |= 4; diff -= step;       vpdiff += step;       }
    if (diff >= (step >> 1)) { nibble |= 2; diff -= step >> 1;  vpdiff += step >> 1;  }
    if (diff >= (step >> 2)) { nibble |= 1;                     vpdiff += step >> 2;  }
    s->predictor += (nibble & 8) ? -vpdiff : vpdiff;
    if (s->predictor >  32767) s->predictor =  32767;
    if (s->predictor < -32768) s->predictor = -32768;
    s->step_index += adpcm_index_table[nibble];
    if (s->step_index <  0) s->step_index = 0;
    if (s->step_index > 88) s->step_index = 88;
    return nibble & 0x0F;
}

static inline int16_t adpcm_decode(uint8_t nibble, adpcm_state_t* s)
{
    int32_t step = adpcm_step_table[s->step_index];
    int32_t diff = step >> 3;
    if (nibble & 4) diff += step;
    if (nibble & 2) diff += step >> 1;
    if (nibble & 1) diff += step >> 2;
    s->predictor += (nibble & 8) ? -diff : diff;
    if (s->predictor >  32767) s->predictor =  32767;
    if (s->predictor < -32768) s->predictor = -32768;
    s->step_index += adpcm_index_table[nibble & 0x0F];
    if (s->step_index <  0) s->step_index = 0;
    if (s->step_index > 88) s->step_index = 88;
    return (int16_t)s->predictor;
}
