#pragma once

#include "sdkconfig.h"

/*
 * Edit the two peer MACs here. Each board must contain the STA MAC of the
 * other board. The firmware role is selected automatically by chip target:
 * ESP32-S3 = Ground, ESP32 = Air.
 */
#define DRONE_AIR_STA_MAC_BYTES    {0x08, 0xA6, 0xF7, 0xB1, 0x43, 0xC4}
#define DRONE_GROUND_STA_MAC_BYTES {0x1C, 0xDB, 0xD4, 0x4A, 0xF8, 0xE0}
#define DRONE_ESPNOW_CHANNEL       6U
//  esp32s3: MAC:                1c:db:d4:4a:f8:e0
//  esp32: MAC:                08:a6:f7:b1:43:c4
#if CONFIG_IDF_TARGET_ESP32S3
#define DRONE_BRIDGE_IS_GROUND 0
#define DRONE_ESPNOW_PEER_MAC_BYTES DRONE_AIR_STA_MAC_BYTES
#elif CONFIG_IDF_TARGET_ESP32
#define DRONE_BRIDGE_IS_GROUND 0
#define DRONE_ESPNOW_PEER_MAC_BYTES DRONE_GROUND_STA_MAC_BYTES
#else
#error "Drone bridge supports only ESP32-S3 Ground and ESP32 Air targets"
#endif

#if (DRONE_ESPNOW_CHANNEL < 1U) || (DRONE_ESPNOW_CHANNEL > 13U)
#error "DRONE_ESPNOW_CHANNEL must be from 1 to 13"
#endif
