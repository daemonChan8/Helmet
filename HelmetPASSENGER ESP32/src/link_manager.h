#pragma once
#include <stdint.h>

enum LinkMode : uint8_t {
    LINK_MODE_MUSIC    = 0,
    LINK_MODE_INTERCOM = 1,
};

void link_manager_init();

void link_manager_process();

LinkMode link_manager_get_mode();

typedef enum {
    ANNOUNCE_ARM_CONFIRM,
    ANNOUNCE_SENDING,
    ANNOUNCE_VOLUME_1,
    ANNOUNCE_VOLUME_2,
    ANNOUNCE_VOLUME_3,
    ANNOUNCE_VOLUME_4,
} AnnouncementId;

void link_manager_play_announcement(AnnouncementId id);

void link_manager_publish_telemetry(uint8_t battery_pct, uint8_t wear_detected);

// Returns last passenger battery percent received over UART, or 255 if unknown
uint8_t link_manager_get_passenger_battery_pct();