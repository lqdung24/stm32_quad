#include "mfrc522.h"

#include <string.h>

#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_COMMAND       0x01U
#define REG_COM_IRQ       0x04U
#define REG_DIV_IRQ       0x05U
#define REG_ERROR         0x06U
#define REG_STATUS2       0x08U
#define REG_FIFO_DATA     0x09U
#define REG_FIFO_LEVEL    0x0aU
#define REG_CONTROL       0x0cU
#define REG_BIT_FRAMING   0x0dU
#define REG_MODE          0x11U
#define REG_TX_CONTROL    0x14U
#define REG_TX_ASK        0x15U
#define REG_CRC_RESULT_H  0x21U
#define REG_CRC_RESULT_L  0x22U
#define REG_RFCFG         0x26U
#define REG_TMODE         0x2aU
#define REG_TPRESCALER    0x2bU
#define REG_TRELOAD_H     0x2cU
#define REG_TRELOAD_L     0x2dU
#define REG_VERSION       0x37U

#define CMD_IDLE          0x00U
#define CMD_CALC_CRC      0x03U
#define CMD_TRANSCEIVE    0x0cU
#define CMD_MF_AUTHENT    0x0eU
#define CMD_SOFT_RESET    0x0fU

#define PICC_REQA         0x26U
#define PICC_WUPA         0x52U
#define PICC_SEL_CL1      0x93U
#define PICC_SEL_CL2      0x95U
#define PICC_SEL_CL3      0x97U
#define PICC_MF_READ      0x30U
#define PICC_MF_WRITE     0xa0U
#define PICC_MF_KEY_A     0x60U
#define PICC_MF_KEY_B     0x61U
#define PICC_HALT         0x50U

#define PICC_SAK_CASCADE  0x04U
#define MFRC522_FIFO_SIZE 64U
#define MFRC522_TIMEOUT_US 50000
#define MFRC522_CARD_WAIT_US 10000000

static esp_err_t reg_write(mfrc522_t *device, uint8_t reg, uint8_t value)
{
    const uint8_t tx[2] = {(uint8_t)((reg << 1U) & 0x7eU), value};
    spi_transaction_t transaction = {
        .length = 16U,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(device->spi, &transaction);
}

static esp_err_t reg_read_many(mfrc522_t *device, uint8_t reg,
                               uint8_t *values, size_t count)
{
    if (count == 0U || count > MFRC522_FIFO_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t tx[MFRC522_FIFO_SIZE + 1U] = {0};
    uint8_t rx[MFRC522_FIFO_SIZE + 1U] = {0};
    const uint8_t address = (uint8_t)(0x80U | ((reg << 1U) & 0x7eU));
    for (size_t i = 0; i < count; ++i) {
        tx[i] = address;
    }
    spi_transaction_t transaction = {
        .length = (count + 1U) * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(device->spi, &transaction),
                        "mfrc522", "SPI read failed");
    memcpy(values, &rx[1], count);
    return ESP_OK;
}

static esp_err_t reg_read(mfrc522_t *device, uint8_t reg, uint8_t *value)
{
    return reg_read_many(device, reg, value, 1U);
}

static esp_err_t reg_set_bits(mfrc522_t *device, uint8_t reg, uint8_t mask)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(reg_read(device, reg, &value), "mfrc522", "read");
    return reg_write(device, reg, (uint8_t)(value | mask));
}

static esp_err_t reg_clear_bits(mfrc522_t *device, uint8_t reg, uint8_t mask)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(reg_read(device, reg, &value), "mfrc522", "read");
    return reg_write(device, reg, (uint8_t)(value & (uint8_t)~mask));
}

static esp_err_t fifo_write(mfrc522_t *device, const uint8_t *data,
                            size_t length)
{
    if (length > MFRC522_FIFO_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    for (size_t i = 0; i < length; ++i) {
        ESP_RETURN_ON_ERROR(reg_write(device, REG_FIFO_DATA, data[i]),
                            "mfrc522", "FIFO write failed");
    }
    return ESP_OK;
}

static esp_err_t calculate_crc(mfrc522_t *device, const uint8_t *data,
                               size_t length, uint8_t result[2])
{
    ESP_RETURN_ON_ERROR(reg_write(device, REG_COMMAND, CMD_IDLE), "mfrc522",
                        "idle failed");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_DIV_IRQ, 0x04U), "mfrc522",
                        "clear CRC IRQ failed");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_FIFO_LEVEL, 0x80U), "mfrc522",
                        "flush FIFO failed");
    ESP_RETURN_ON_ERROR(fifo_write(device, data, length), "mfrc522",
                        "load CRC FIFO failed");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_COMMAND, CMD_CALC_CRC),
                        "mfrc522", "start CRC failed");

    const int64_t deadline = esp_timer_get_time() + MFRC522_TIMEOUT_US;
    uint8_t irq = 0U;
    do {
        ESP_RETURN_ON_ERROR(reg_read(device, REG_DIV_IRQ, &irq), "mfrc522",
                            "CRC IRQ read failed");
        if ((irq & 0x04U) != 0U) {
            ESP_RETURN_ON_ERROR(reg_write(device, REG_COMMAND, CMD_IDLE),
                                "mfrc522", "CRC idle failed");
            ESP_RETURN_ON_ERROR(reg_read(device, REG_CRC_RESULT_L, &result[0]),
                                "mfrc522", "CRC low read failed");
            return reg_read(device, REG_CRC_RESULT_H, &result[1]);
        }
    } while (esp_timer_get_time() < deadline);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t communicate(mfrc522_t *device, uint8_t command,
                             uint8_t wait_irq, const uint8_t *tx,
                             size_t tx_length, uint8_t tx_last_bits,
                             uint8_t *rx, size_t *rx_length,
                             uint8_t *rx_valid_bits)
{
    ESP_RETURN_ON_ERROR(reg_write(device, REG_COMMAND, CMD_IDLE), "mfrc522",
                        "idle failed");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_COM_IRQ, 0x7fU), "mfrc522",
                        "clear IRQ failed");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_FIFO_LEVEL, 0x80U), "mfrc522",
                        "flush FIFO failed");
    ESP_RETURN_ON_ERROR(fifo_write(device, tx, tx_length), "mfrc522",
                        "load FIFO failed");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_BIT_FRAMING,
                                  (uint8_t)(tx_last_bits & 0x07U)),
                        "mfrc522", "bit framing failed");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_COMMAND, command), "mfrc522",
                        "command failed");
    if (command == CMD_TRANSCEIVE) {
        ESP_RETURN_ON_ERROR(reg_set_bits(device, REG_BIT_FRAMING, 0x80U),
                            "mfrc522", "start send failed");
    }

    const int64_t deadline = esp_timer_get_time() + MFRC522_TIMEOUT_US;
    uint8_t irq = 0U;
    do {
        ESP_RETURN_ON_ERROR(reg_read(device, REG_COM_IRQ, &irq), "mfrc522",
                            "IRQ read failed");
        if ((irq & wait_irq) != 0U) {
            break;
        }
        if ((irq & 0x01U) != 0U) {
            return ESP_ERR_TIMEOUT;
        }
    } while (esp_timer_get_time() < deadline);
    if ((irq & wait_irq) == 0U) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t error;
    ESP_RETURN_ON_ERROR(reg_read(device, REG_ERROR, &error), "mfrc522",
                        "error register read failed");
    if ((error & 0x1bU) != 0U) {
        return ESP_FAIL;
    }
    if (rx != NULL && rx_length != NULL) {
        uint8_t level;
        ESP_RETURN_ON_ERROR(reg_read(device, REG_FIFO_LEVEL, &level),
                            "mfrc522", "FIFO level read failed");
        if (level > *rx_length) {
            return ESP_ERR_INVALID_SIZE;
        }
        *rx_length = level;
        if (level > 0U) {
            ESP_RETURN_ON_ERROR(reg_read_many(device, REG_FIFO_DATA, rx, level),
                                "mfrc522", "FIFO read failed");
        }
        if (rx_valid_bits != NULL) {
            uint8_t control;
            ESP_RETURN_ON_ERROR(reg_read(device, REG_CONTROL, &control),
                                "mfrc522", "control read failed");
            *rx_valid_bits = (uint8_t)(control & 0x07U);
        }
    }
    return ESP_OK;
}

static esp_err_t transceive(mfrc522_t *device, const uint8_t *tx,
                            size_t tx_length, uint8_t tx_last_bits,
                            uint8_t *rx, size_t *rx_length,
                            uint8_t *rx_valid_bits)
{
    return communicate(device, CMD_TRANSCEIVE, 0x30U, tx, tx_length,
                       tx_last_bits, rx, rx_length, rx_valid_bits);
}

/* A PICC must not answer a valid HLTA command. A receive timeout therefore
 * means that the command completed as expected. When Crypto1 is active the
 * MFRC522 encrypts this frame before transmitting it. */
static esp_err_t halt_card(mfrc522_t *device)
{
    uint8_t frame[4] = {PICC_HALT, 0x00U};
    ESP_RETURN_ON_ERROR(calculate_crc(device, frame, 2U, &frame[2]),
                        "mfrc522", "halt CRC failed");
    const esp_err_t err = transceive(device, frame, sizeof(frame), 0U,
                                     NULL, NULL, NULL);
    return err == ESP_ERR_TIMEOUT ? ESP_OK : ESP_FAIL;
}

static esp_err_t request(mfrc522_t *device, uint8_t command, uint8_t atqa[2])
{
    size_t length = 2U;
    uint8_t valid_bits = 0U;
    ESP_RETURN_ON_ERROR(reg_clear_bits(device, REG_STATUS2, 0x08U),
                        "mfrc522", "stop crypto failed");
    ESP_RETURN_ON_ERROR(transceive(device, &command, 1U, 7U, atqa, &length,
                                   &valid_bits),
                        "mfrc522", "request failed");
    return (length == 2U && valid_bits == 0U) ? ESP_OK : ESP_FAIL;
}

static esp_err_t select_card(mfrc522_t *device, mfrc522_card_t *card)
{
    static const uint8_t selectors[] = {PICC_SEL_CL1, PICC_SEL_CL2,
                                        PICC_SEL_CL3};
    uint8_t uid_length = 0U;
    for (size_t cascade = 0; cascade < 3U; ++cascade) {
        uint8_t anticollision[2] = {selectors[cascade], 0x20U};
        uint8_t response[5];
        size_t response_length = sizeof(response);
        uint8_t valid_bits = 0U;
        ESP_RETURN_ON_ERROR(transceive(device, anticollision,
                                       sizeof(anticollision), 0U, response,
                                       &response_length, &valid_bits),
                            "mfrc522", "anticollision failed");
        if (response_length != 5U || valid_bits != 0U ||
            (uint8_t)(response[0] ^ response[1] ^ response[2] ^ response[3]) !=
                response[4]) {
            return ESP_ERR_INVALID_CRC;
        }

        uint8_t select_frame[9] = {selectors[cascade], 0x70U};
        memcpy(&select_frame[2], response, sizeof(response));
        ESP_RETURN_ON_ERROR(calculate_crc(device, select_frame, 7U,
                                          &select_frame[7]),
                            "mfrc522", "select CRC failed");
        uint8_t sak_response[3];
        response_length = sizeof(sak_response);
        ESP_RETURN_ON_ERROR(transceive(device, select_frame,
                                       sizeof(select_frame), 0U, sak_response,
                                       &response_length, &valid_bits),
                            "mfrc522", "select failed");
        if (response_length != 3U || valid_bits != 0U) {
            return ESP_FAIL;
        }
        uint8_t crc[2];
        ESP_RETURN_ON_ERROR(calculate_crc(device, sak_response, 1U, crc),
                            "mfrc522", "SAK CRC failed");
        if (crc[0] != sak_response[1] || crc[1] != sak_response[2]) {
            return ESP_ERR_INVALID_CRC;
        }

        const size_t start = response[0] == 0x88U ? 1U : 0U;
        const size_t bytes = 4U - start;
        if ((size_t)uid_length + bytes > sizeof(card->uid)) {
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(&card->uid[uid_length], &response[start], bytes);
        uid_length = (uint8_t)(uid_length + bytes);
        card->sak = sak_response[0];
        if ((card->sak & PICC_SAK_CASCADE) == 0U) {
            card->uid_length = uid_length;
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

static bool same_card(const mfrc522_card_t *left,
                      const mfrc522_card_t *right)
{
    return left->uid_length == right->uid_length &&
           memcmp(left->uid, right->uid, left->uid_length) == 0;
}

/* End the encrypted session on both sides, wake the halted PICC, and run the
 * selection sequence again. This deliberately avoids sending a plain-text
 * authentication command while the PICC is still in its Crypto1 session. */
static esp_err_t stop_crypto_and_reselect(mfrc522_t *device,
                                          const mfrc522_card_t *expected)
{
    (void)halt_card(device);
    ESP_RETURN_ON_ERROR(reg_clear_bits(device, REG_STATUS2, 0x08U),
                        "mfrc522", "stop crypto failed");

    mfrc522_card_t selected = {0};
    ESP_RETURN_ON_ERROR(request(device, PICC_WUPA, selected.atqa), "mfrc522",
                        "card wakeup failed");
    ESP_RETURN_ON_ERROR(select_card(device, &selected), "mfrc522",
                        "card reselect failed");
    return same_card(&selected, expected) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static void finish_card_session(mfrc522_t *device)
{
    (void)halt_card(device);
    (void)reg_clear_bits(device, REG_STATUS2, 0x08U);
}

/* Same sequence as stop_crypto_and_reselect() but without an identity check.
 * After a block 0 rewrite the card answers with its new UID, so the caller
 * cannot know beforehand which UID to expect. */
static esp_err_t reselect_any(mfrc522_t *device, mfrc522_card_t *card)
{
    (void)halt_card(device);
    ESP_RETURN_ON_ERROR(reg_clear_bits(device, REG_STATUS2, 0x08U),
                        "mfrc522", "stop crypto failed");
    memset(card, 0, sizeof(*card));
    ESP_RETURN_ON_ERROR(request(device, PICC_WUPA, card->atqa), "mfrc522",
                        "card wakeup failed");
    return select_card(device, card);
}

static esp_err_t card_geometry(uint8_t sak, uint16_t *blocks,
                               uint8_t *sectors)
{
    switch (sak & 0x7fU) {
    case 0x09U:
        *blocks = 20U;
        *sectors = 5U;
        return ESP_OK;
    case 0x08U:
        *blocks = 64U;
        *sectors = 16U;
        return ESP_OK;
    case 0x18U:
        *blocks = 256U;
        *sectors = 40U;
        return ESP_OK;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t authenticate(mfrc522_t *device, uint16_t block,
                              const mfrc522_sector_key_t *key,
                              const mfrc522_card_t *card)
{
    if (block > UINT8_MAX || card->uid_length < 4U) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t frame[12];
    frame[0] = key->type == MFRC522_KEY_B ? PICC_MF_KEY_B : PICC_MF_KEY_A;
    frame[1] = (uint8_t)block;
    memcpy(&frame[2], key->bytes, 6U);
    memcpy(&frame[8], &card->uid[card->uid_length - 4U], 4U);
    ESP_RETURN_ON_ERROR(communicate(device, CMD_MF_AUTHENT, 0x10U, frame,
                                    sizeof(frame), 0U, NULL, NULL, NULL),
                        "mfrc522", "authentication command failed");
    uint8_t status2;
    ESP_RETURN_ON_ERROR(reg_read(device, REG_STATUS2, &status2), "mfrc522",
                        "authentication status failed");
    return (status2 & 0x08U) != 0U ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t read_block(mfrc522_t *device, uint16_t block,
                            uint8_t data[MFRC522_BLOCK_BYTES])
{
    if (block > UINT8_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t frame[4] = {PICC_MF_READ, (uint8_t)block};
    ESP_RETURN_ON_ERROR(calculate_crc(device, frame, 2U, &frame[2]),
                        "mfrc522", "read CRC failed");
    uint8_t response[18];
    size_t response_length = sizeof(response);
    uint8_t valid_bits = 0U;
    ESP_RETURN_ON_ERROR(transceive(device, frame, sizeof(frame), 0U, response,
                                   &response_length, &valid_bits),
                        "mfrc522", "block read failed");
    if (response_length != sizeof(response) || valid_bits != 0U) {
        return ESP_FAIL;
    }
    uint8_t crc[2];
    ESP_RETURN_ON_ERROR(calculate_crc(device, response, 16U, crc), "mfrc522",
                        "data CRC failed");
    if (crc[0] != response[16] || crc[1] != response[17]) {
        return ESP_ERR_INVALID_CRC;
    }
    memcpy(data, response, MFRC522_BLOCK_BYTES);
    return ESP_OK;
}

static bool mifare_ack(const uint8_t *response, size_t length,
                       uint8_t valid_bits)
{
    return length == 1U && valid_bits == 4U && (response[0] & 0x0fU) == 0x0aU;
}

/* Unguarded MIFARE write. Only mfrc522_write_uid may target block 0; every
 * other caller must go through write_block() so a bad snapshot can never
 * overwrite the manufacturer block. */
static esp_err_t write_block_raw(mfrc522_t *device, uint16_t block,
                                 const uint8_t data[MFRC522_BLOCK_BYTES])
{
    if (block > UINT8_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t command[4] = {PICC_MF_WRITE, (uint8_t)block};
    ESP_RETURN_ON_ERROR(calculate_crc(device, command, 2U, &command[2]),
                        "mfrc522", "write command CRC failed");
    uint8_t response[2];
    size_t response_length = sizeof(response);
    uint8_t valid_bits = 0U;
    ESP_RETURN_ON_ERROR(transceive(device, command, sizeof(command), 0U,
                                   response, &response_length, &valid_bits),
                        "mfrc522", "write command failed");
    if (!mifare_ack(response, response_length, valid_bits)) {
        return ESP_FAIL;
    }

    uint8_t frame[18];
    memcpy(frame, data, MFRC522_BLOCK_BYTES);
    ESP_RETURN_ON_ERROR(calculate_crc(device, frame, MFRC522_BLOCK_BYTES,
                                      &frame[MFRC522_BLOCK_BYTES]),
                        "mfrc522", "write data CRC failed");
    response_length = sizeof(response);
    valid_bits = 0U;
    ESP_RETURN_ON_ERROR(transceive(device, frame, sizeof(frame), 0U, response,
                                   &response_length, &valid_bits),
                        "mfrc522", "write data failed");
    return mifare_ack(response, response_length, valid_bits) ? ESP_OK : ESP_FAIL;
}

static esp_err_t write_block(mfrc522_t *device, uint16_t block,
                             const uint8_t data[MFRC522_BLOCK_BYTES])
{
    if (block == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    return write_block_raw(device, block, data);
}

esp_err_t mfrc522_init(mfrc522_t *device, const mfrc522_config_t *config)
{
    if (device == NULL || config == NULL || config->clock_hz == 0U ||
        config->clock_hz > 10000000U) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(device, 0, sizeof(*device));
    device->rst_io = config->rst_io;

    const gpio_config_t reset_config = {
        .pin_bit_mask = 1ULL << config->rst_io,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&reset_config), "mfrc522",
                        "reset GPIO failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(config->rst_io, 0), "mfrc522",
                        "reset low failed");
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_RETURN_ON_ERROR(gpio_set_level(config->rst_io, 1), "mfrc522",
                        "reset high failed");
    vTaskDelay(pdMS_TO_TICKS(50));

    const spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_io,
        .miso_io_num = config->miso_io,
        .sclk_io_num = config->sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(config->host, &bus_config,
                                           SPI_DMA_DISABLED),
                        "mfrc522", "SPI bus init failed");
    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = (int)config->clock_hz,
        .mode = 0,
        .spics_io_num = config->cs_io,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(config->host, &device_config,
                                           &device->spi),
                        "mfrc522", "SPI device init failed");

    ESP_RETURN_ON_ERROR(reg_write(device, REG_COMMAND, CMD_SOFT_RESET),
                        "mfrc522", "soft reset failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(reg_write(device, REG_TMODE, 0x80U), "mfrc522", "init");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_TPRESCALER, 0xa9U), "mfrc522", "init");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_TRELOAD_H, 0x03U), "mfrc522", "init");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_TRELOAD_L, 0xe8U), "mfrc522", "init");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_TX_ASK, 0x40U), "mfrc522", "init");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_MODE, 0x3dU), "mfrc522", "init");
    ESP_RETURN_ON_ERROR(reg_write(device, REG_RFCFG, 0x70U), "mfrc522", "init");
    ESP_RETURN_ON_ERROR(reg_set_bits(device, REG_TX_CONTROL, 0x03U),
                        "mfrc522", "antenna on failed");

    uint8_t version;
    ESP_RETURN_ON_ERROR(reg_read(device, REG_VERSION, &version), "mfrc522",
                        "version read failed");
    return (version == 0x00U || version == 0xffU) ? ESP_ERR_NOT_FOUND : ESP_OK;
}

esp_err_t mfrc522_scan(mfrc522_t *device, mfrc522_card_t *card)
{
    if (device == NULL || card == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(card, 0, sizeof(*card));
    esp_err_t err = request(device, PICC_REQA, card->atqa);
    if (err != ESP_OK) {
        err = request(device, PICC_WUPA, card->atqa);
    }
    if (err != ESP_OK) {
        return err;
    }
    return select_card(device, card);
}

esp_err_t mfrc522_wait_for_card(mfrc522_t *device, mfrc522_card_t *card)
{
    if (device == NULL || card == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const int64_t deadline = esp_timer_get_time() + MFRC522_CARD_WAIT_US;
    esp_err_t err;
    do {
        err = mfrc522_scan(device, card);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    } while (esp_timer_get_time() < deadline);
    return err;
}

void mfrc522_release(mfrc522_t *device)
{
    if (device != NULL) {
        finish_card_session(device);
    }
}

uint16_t mfrc522_sector_first_block(uint8_t sector)
{
    return sector < 32U ? (uint16_t)sector * 4U
                        : (uint16_t)(128U + (uint16_t)(sector - 32U) * 16U);
}

uint16_t mfrc522_sector_trailer_block(uint8_t sector)
{
    return sector < 32U ? (uint16_t)sector * 4U + 3U
                        : (uint16_t)(128U + (uint16_t)(sector - 32U) * 16U + 15U);
}

bool mfrc522_snapshot_block_valid(const mfrc522_snapshot_t *snapshot,
                                  uint16_t block)
{
    return snapshot != NULL && block < snapshot->block_count &&
           (snapshot->valid[block / 8U] & (uint8_t)(1U << (block % 8U))) != 0U;
}

static void snapshot_set_valid(mfrc522_snapshot_t *snapshot, uint16_t block)
{
    snapshot->valid[block / 8U] |= (uint8_t)(1U << (block % 8U));
}

esp_err_t mfrc522_read_snapshot(
    mfrc522_t *device, const mfrc522_sector_key_t *keys,
    mfrc522_snapshot_t *snapshot, uint16_t *read_blocks,
    uint8_t *failed_sectors)
{
    if (device == NULL || keys == NULL || snapshot == NULL ||
        read_blocks == NULL || failed_sectors == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    *read_blocks = 0U;
    *failed_sectors = 0U;
    ESP_RETURN_ON_ERROR(mfrc522_wait_for_card(device, &snapshot->card),
                        "mfrc522", "card scan failed");
    const esp_err_t geometry_err = card_geometry(snapshot->card.sak,
                                                 &snapshot->block_count,
                                                 &snapshot->sector_count);
    if (geometry_err != ESP_OK) {
        finish_card_session(device);
        ESP_LOGE("mfrc522", "unsupported card type");
        return geometry_err;
    }

    for (uint8_t sector = 0U; sector < snapshot->sector_count; ++sector) {
        const uint16_t first = mfrc522_sector_first_block(sector);
        const uint16_t trailer = mfrc522_sector_trailer_block(sector);
        if (authenticate(device, first, &keys[sector], &snapshot->card) != ESP_OK) {
            ++*failed_sectors;
            if (sector + 1U < snapshot->sector_count &&
                stop_crypto_and_reselect(device, &snapshot->card) != ESP_OK) {
                *failed_sectors = (uint8_t)(*failed_sectors +
                    snapshot->sector_count - sector - 1U);
                finish_card_session(device);
                break;
            }
            if (sector + 1U == snapshot->sector_count) {
                finish_card_session(device);
            }
            continue;
        }
        bool sector_failed = false;
        for (uint16_t block = first; block < trailer; ++block) {
            if (block == 0U) {
                continue;
            }
            if (read_block(device, block, snapshot->blocks[block]) == ESP_OK) {
                snapshot_set_valid(snapshot, block);
                ++*read_blocks;
            } else {
                sector_failed = true;
            }
        }
        if (sector_failed) {
            ++*failed_sectors;
        }
        if (sector + 1U < snapshot->sector_count) {
            if (stop_crypto_and_reselect(device, &snapshot->card) != ESP_OK) {
                *failed_sectors = (uint8_t)(*failed_sectors +
                    snapshot->sector_count - sector - 1U);
                finish_card_session(device);
                break;
            }
        } else {
            finish_card_session(device);
        }
    }
    return *read_blocks > 0U ? ESP_OK : ESP_FAIL;
}

esp_err_t mfrc522_write_snapshot(
    mfrc522_t *device, const mfrc522_sector_key_t *keys,
    const mfrc522_snapshot_t *snapshot, mfrc522_card_t *target,
    mfrc522_write_stats_t *stats)
{
    if (device == NULL || keys == NULL || snapshot == NULL || target == NULL ||
        stats == NULL || snapshot->block_count == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(stats, 0, sizeof(*stats));
    ESP_RETURN_ON_ERROR(mfrc522_wait_for_card(device, target), "mfrc522",
                        "target scan failed");
    uint16_t target_blocks;
    uint8_t target_sectors;
    const esp_err_t geometry_err = card_geometry(target->sak, &target_blocks,
                                                 &target_sectors);
    if (geometry_err != ESP_OK) {
        finish_card_session(device);
        ESP_LOGE("mfrc522", "unsupported target card");
        return geometry_err;
    }
    if (target_blocks != snapshot->block_count ||
        target_sectors != snapshot->sector_count) {
        finish_card_session(device);
        return ESP_ERR_INVALID_SIZE;
    }

    bool session_failed = false;
    for (uint8_t sector = 0U; sector < target_sectors; ++sector) {
        const uint16_t first = mfrc522_sector_first_block(sector);
        const uint16_t trailer = mfrc522_sector_trailer_block(sector);
        bool has_data = false;
        for (uint16_t block = first; block < trailer; ++block) {
            has_data |= mfrc522_snapshot_block_valid(snapshot, block);
        }
        if (!has_data) {
            if (sector + 1U == target_sectors) {
                finish_card_session(device);
            }
            continue;
        }
        if (authenticate(device, first, &keys[sector], target) != ESP_OK) {
            for (uint16_t block = first; block < trailer; ++block) {
                if (mfrc522_snapshot_block_valid(snapshot, block)) {
                    ++stats->attempted;
                    ++stats->failed;
                }
            }
            if (sector + 1U < target_sectors &&
                stop_crypto_and_reselect(device, target) != ESP_OK) {
                session_failed = true;
                finish_card_session(device);
                break;
            }
            if (sector + 1U == target_sectors) {
                finish_card_session(device);
            }
            continue;
        }
        for (uint16_t block = first; block < trailer; ++block) {
            if (!mfrc522_snapshot_block_valid(snapshot, block)) {
                continue;
            }
            ++stats->attempted;
            uint8_t verify[MFRC522_BLOCK_BYTES];
            if (write_block(device, block, snapshot->blocks[block]) == ESP_OK &&
                read_block(device, block, verify) == ESP_OK &&
                memcmp(verify, snapshot->blocks[block], sizeof(verify)) == 0) {
                ++stats->succeeded;
            } else {
                ++stats->failed;
            }
        }
        if (sector + 1U < target_sectors) {
            if (stop_crypto_and_reselect(device, target) != ESP_OK) {
                session_failed = true;
                finish_card_session(device);
                break;
            }
        } else {
            finish_card_session(device);
        }
    }
    return !session_failed && stats->failed == 0U && stats->succeeded > 0U
               ? ESP_OK
               : ESP_FAIL;
}

esp_err_t mfrc522_write_uid(mfrc522_t *device, const mfrc522_sector_key_t *keys,
                            const mfrc522_uid_write_t *request_data,
                            mfrc522_uid_write_result_t *result)
{
    if (device == NULL || keys == NULL || request_data == NULL ||
        result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));
    ESP_RETURN_ON_ERROR(mfrc522_wait_for_card(device, &result->before),
                        "mfrc522", "target scan failed");

    /* Reject anything whose block 0 is not a MIFARE Classic manufacturer
     * block. On Ultralight/NTAG, block 0 holds the UID itself and this write
     * would permanently brick the tag. */
    uint16_t blocks;
    uint8_t sectors;
    const esp_err_t geometry_err = card_geometry(result->before.sak, &blocks,
                                                 &sectors);
    if (geometry_err != ESP_OK) {
        finish_card_session(device);
        ESP_LOGE("mfrc522", "unsupported card type");
        return geometry_err;
    }
    if (result->before.uid_length != MFRC522_UID_WRITE_BYTES) {
        finish_card_session(device);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (authenticate(device, 0U, &keys[0], &result->before) != ESP_OK) {
        finish_card_session(device);
        return ESP_ERR_INVALID_STATE;
    }
    /* Preserve SAK, ATQA, and manufacturer bytes unless explicitly overridden,
     * so a card keeps answering exactly as it did before. */
    if (read_block(device, 0U, result->old_block0) != ESP_OK) {
        finish_card_session(device);
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(result->new_block0, result->old_block0, MFRC522_BLOCK_BYTES);
    memcpy(result->new_block0, request_data->uid, MFRC522_UID_WRITE_BYTES);
    result->new_block0[4] = (uint8_t)(request_data->uid[0] ^
                                      request_data->uid[1] ^
                                      request_data->uid[2] ^
                                      request_data->uid[3]);
    if (request_data->set_sak) {
        result->new_block0[5] = request_data->sak;
    }
    if (request_data->set_atqa) {
        result->new_block0[6] = request_data->atqa[0];
        result->new_block0[7] = request_data->atqa[1];
    }

    if (write_block_raw(device, 0U, result->new_block0) != ESP_OK) {
        finish_card_session(device);
        return ESP_ERR_NOT_SUPPORTED;
    }
    result->block0_written = true;

    /* The new UID only takes effect on the next selection. */
    if (reselect_any(device, &result->after) != ESP_OK) {
        finish_card_session(device);
        return ESP_ERR_INVALID_RESPONSE;
    }
    result->reselected = true;
    result->uid_matches =
        result->after.uid_length == MFRC522_UID_WRITE_BYTES &&
        memcmp(result->after.uid, request_data->uid,
               MFRC522_UID_WRITE_BYTES) == 0;

    uint8_t verify[MFRC522_BLOCK_BYTES];
    if (authenticate(device, 0U, &keys[0], &result->after) == ESP_OK &&
        read_block(device, 0U, verify) == ESP_OK) {
        result->block0_verified =
            memcmp(verify, result->new_block0, sizeof(verify)) == 0;
    }
    finish_card_session(device);

    return (result->block0_verified && result->uid_matches) ? ESP_OK : ESP_FAIL;
}
