#pragma once

/*
 * Edit both STA MAC addresses here. Ground and Air select their peer
 * explicitly; the firmware role is never inferred from the chip target.
 */
#define DRONE_AIR_STA_MAC_BYTES    {0x08, 0xA6, 0xF7, 0xB1, 0x43, 0xC4}
#define DRONE_GROUND_STA_MAC_BYTES {0x1C, 0xDB, 0xD4, 0x4A, 0xF8, 0xE0}
#define DRONE_ESPNOW_CHANNEL       6U
//  esp32s3: MAC:                1c:db:d4:4a:f8:e0
//  esp32: MAC:                08:a6:f7:b1:43:c4
#if (DRONE_ESPNOW_CHANNEL < 1U) || (DRONE_ESPNOW_CHANNEL > 13U)
#error "DRONE_ESPNOW_CHANNEL must be from 1 to 13"
#endif
