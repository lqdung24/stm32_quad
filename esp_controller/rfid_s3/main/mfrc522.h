#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#define MFRC522_UID_MAX_BYTES 10U
#define MFRC522_MAX_BLOCKS 256U
#define MFRC522_BLOCK_BYTES 16U
#define MFRC522_MAX_SECTORS 40U

/* Only single-size (4-byte) UIDs can be rewritten. Block 0 of a double-size
 * UID card has a different layout and no single BCC byte. */
#define MFRC522_UID_WRITE_BYTES 4U

typedef enum {
    MFRC522_KEY_A = 0,
    MFRC522_KEY_B = 1,
} mfrc522_key_type_t;

typedef struct {
    uint8_t bytes[6];
    mfrc522_key_type_t type;
} mfrc522_sector_key_t;

typedef struct {
    uint8_t uid[MFRC522_UID_MAX_BYTES];
    uint8_t uid_length;
    uint8_t sak;
    uint8_t atqa[2];
} mfrc522_card_t;

typedef struct {
    mfrc522_card_t card;
    uint16_t block_count;
    uint8_t sector_count;
    uint8_t blocks[MFRC522_MAX_BLOCKS][MFRC522_BLOCK_BYTES];
    uint8_t valid[(MFRC522_MAX_BLOCKS + 7U) / 8U];
} mfrc522_snapshot_t;

typedef struct {
    spi_host_device_t host;
    gpio_num_t sclk_io;
    gpio_num_t mosi_io;
    gpio_num_t miso_io;
    gpio_num_t cs_io;
    gpio_num_t rst_io;
    uint32_t clock_hz;
} mfrc522_config_t;

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t rst_io;
} mfrc522_t;

typedef struct {
    uint16_t attempted;
    uint16_t succeeded;
    uint16_t failed;
} mfrc522_write_stats_t;

/* Requested block 0 contents. The BCC is always recomputed by the driver, and
 * every byte that is not selected here is preserved from the target card. */
typedef struct {
    uint8_t uid[MFRC522_UID_WRITE_BYTES];
    uint8_t sak;
    uint8_t atqa[2];
    bool set_sak;
    bool set_atqa;
} mfrc522_uid_write_t;

typedef struct {
    mfrc522_card_t before;      /* card as selected before the write */
    mfrc522_card_t after;       /* card as reselected after the write */
    uint8_t old_block0[MFRC522_BLOCK_BYTES];
    uint8_t new_block0[MFRC522_BLOCK_BYTES];
    bool block0_written;        /* card accepted the block 0 write */
    bool block0_verified;       /* block 0 read back byte-identical */
    bool reselected;            /* card answered again after the write */
    bool uid_matches;           /* reselected UID equals the requested UID */
} mfrc522_uid_write_result_t;

esp_err_t mfrc522_init(mfrc522_t *device, const mfrc522_config_t *config);
esp_err_t mfrc522_scan(mfrc522_t *device, mfrc522_card_t *card);
esp_err_t mfrc522_wait_for_card(mfrc522_t *device, mfrc522_card_t *card);

/* Halt the selected PICC and drop Crypto1. A card left in the ACTIVE state
 * answers neither REQA nor WUPA, so every successful scan must be released
 * before the next scan of the same card can succeed. */
void mfrc522_release(mfrc522_t *device);

/* Rewrite block 0 of a UID-writable (CUID / "magic gen2") MIFARE Classic card
 * using normal sector 0 authentication with keys[0]. Standard cards NAK this
 * write and ESP_ERR_NOT_SUPPORTED is returned. */
esp_err_t mfrc522_write_uid(mfrc522_t *device, const mfrc522_sector_key_t *keys,
                            const mfrc522_uid_write_t *request,
                            mfrc522_uid_write_result_t *result);
esp_err_t mfrc522_read_snapshot(
    mfrc522_t *device, const mfrc522_sector_key_t *keys,
    mfrc522_snapshot_t *snapshot, uint16_t *read_blocks,
    uint8_t *failed_sectors);
esp_err_t mfrc522_write_snapshot(
    mfrc522_t *device, const mfrc522_sector_key_t *keys,
    const mfrc522_snapshot_t *snapshot, mfrc522_card_t *target,
    mfrc522_write_stats_t *stats);

bool mfrc522_snapshot_block_valid(const mfrc522_snapshot_t *snapshot,
                                  uint16_t block);
uint16_t mfrc522_sector_first_block(uint8_t sector);
uint16_t mfrc522_sector_trailer_block(uint8_t sector);
