#include <assert.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "driver/spi_master.h"
#include "driver/rc522_spi.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "picc/rc522_mifare.h"
#include "rc522.h"
#include "rc522_picc.h"

#include "usb_console.h"


#define RFID_SCLK_GPIO GPIO_NUM_10
#define RFID_MOSI_GPIO GPIO_NUM_11
#define RFID_CS_GPIO   GPIO_NUM_12
#define RFID_MISO_GPIO GPIO_NUM_13
#define RFID_RST_GPIO  GPIO_NUM_14

#define CARD_WAIT_MS         10000U
#define RFID_IO_LOCK_WAIT_MS 2000U


static rc522_handle_t s_scanner;

/*
 * Points to the PICC object owned by the rc522 library.
 * NULL means no active card is currently known.
 */
static const rc522_picc_t *s_picc;

/*
 * Protects s_picc.
 */
static SemaphoreHandle_t s_picc_lock;

/*
 * Signaled when a card becomes ACTIVE.
 */
static SemaphoreHandle_t s_card_seen;

/*
 * Shared with rc522_config_t.task_mutex.
 *
 * This serializes our MIFARE commands against the library's
 * internal rc522_polling_task.
 */
static SemaphoreHandle_t s_rc522_io_lock;


/*
 * Per-sector MIFARE keys.
 *
 * Default:
 *   Key A = FF FF FF FF FF FF
 */
static rc522_mifare_key_t
    s_keys[RC522_MIFARE_SECTOR_INDEX_MAX + 1U];


static void reset_keys(void)
{
    for (size_t i = 0;
         i < sizeof(s_keys) / sizeof(s_keys[0]);
         ++i) {

        s_keys[i] = (rc522_mifare_key_t) {
            .type = RC522_MIFARE_KEY_A,
            .value = {
                RC522_MIFARE_KEY_VALUE_DEFAULT
            },
        };
    }
}


static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }

    c = (char)toupper((unsigned char)c);

    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }

    return -1;
}


static bool parse_hex(
    const char *text,
    uint8_t *out,
    size_t length)
{
    if (text == NULL ||
        strlen(text) != length * 2U) {
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        int hi = hex_nibble(text[i * 2U]);
        int lo = hex_nibble(text[i * 2U + 1U]);

        if (hi < 0 || lo < 0) {
            return false;
        }

        out[i] = (uint8_t)((hi << 4) | lo);
    }

    return true;
}


/*
 * Return the currently active PICC.
 *
 * If no card is active, wait up to CARD_WAIT_MS for one.
 */
static const rc522_picc_t *active_picc(void)
{
    const rc522_picc_t *picc;

    xSemaphoreTake(s_picc_lock, portMAX_DELAY);
    picc = s_picc;
    xSemaphoreGive(s_picc_lock);

    if (picc != NULL) {
        return picc;
    }

    (void)usb_console_write(
        "Place a card on the reader...\r\n");

    if (xSemaphoreTake(
            s_card_seen,
            pdMS_TO_TICKS(CARD_WAIT_MS)) != pdTRUE) {

        return NULL;
    }

    xSemaphoreTake(s_picc_lock, portMAX_DELAY);
    picc = s_picc;
    xSemaphoreGive(s_picc_lock);

    return picc;
}


static void print_picc(
    const rc522_picc_t *picc)
{
    (void)usb_console_write("UID=");

    for (uint8_t i = 0;
         i < picc->uid.length;
         ++i) {

        (void)usb_console_printf(
            "%02X",
            (unsigned)picc->uid.value[i]);
    }

    (void)usb_console_printf(
        " SAK=%02X TYPE=%s\r\n",
        (unsigned)picc->sak,
        rc522_picc_type_name(picc->type));
}


/*
 * Only allow ordinary data blocks.
 *
 * Refuse:
 *   - manufacturer block 0
 *   - sector trailers
 */
static esp_err_t data_block(
    const rc522_picc_t *picc,
    uint8_t block,
    uint8_t *sector_index)
{
    rc522_mifare_desc_t desc;
    rc522_mifare_sector_desc_t sector;

    if (!rc522_mifare_type_is_classic_compatible(
            picc->type)) {

        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_RETURN_ON_ERROR(
        rc522_mifare_get_desc(
            picc,
            &desc),
        "rfid",
        "describe card");

    *sector_index =
        rc522_mifare_get_sector_index_by_block_address(
            block);

    if (*sector_index >= desc.number_of_sectors) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(
        rc522_mifare_get_sector_desc(
            *sector_index,
            &sector),
        "rfid",
        "describe sector");

    /*
     * Block 0:
     *   manufacturer block
     *
     * Last block of every sector:
     *   sector trailer
     * 
     * But i wanna rewrite it so i'll change it
     */
    // if (block < sector.block_0_address ||
    //     block >=
    //         sector.block_0_address +
    //         sector.number_of_blocks) {

    //     return ESP_ERR_NOT_ALLOWED;
    // }

    return ESP_OK;
}


static void command_scan(void)
{
    const rc522_picc_t *picc = active_picc();

    if (picc == NULL) {
        (void)usb_console_write(
            "ERR no card detected after 10 seconds\r\n");
        return;
    }

    print_picc(picc);
}


static bool parse_block(
    const char *text,
    uint8_t *block)
{
    char *end = NULL;

    unsigned long n =
        text == NULL
            ? 256UL
            : strtoul(text, &end, 10);

    if (end == text ||
        *end != '\0' ||
        n > UINT8_MAX) {

        return false;
    }

    *block = (uint8_t)n;

    return true;
}


/*
 * Read/write one MIFARE Classic data block.
 *
 * Important:
 *
 * s_rc522_io_lock is the same mutex supplied to
 * rc522_config_t.task_mutex.
 *
 * Therefore:
 *
 *     CLI read/write
 *          vs
 *     rc522_polling_task
 *
 * can never access MFRC522 at the same time.
 */
static void command_rw(
    bool write,
    char *block_text,
    char *hex)
{
    uint8_t block;
    uint8_t sector;

    uint8_t expected[RC522_MIFARE_BLOCK_SIZE];
    uint8_t actual[RC522_MIFARE_BLOCK_SIZE];

    const rc522_picc_t *picc = NULL;

    esp_err_t err = ESP_OK;
    bool authenticated = false;


    /* ---------------------------------------------------------
     * Parse command arguments
     * --------------------------------------------------------- */

    if (!parse_block(block_text, &block)) {
        (void)usb_console_write(
            write
                ? "ERR usage: write <data-block> <32 hex chars / 16 bytes>\r\n"
                : "ERR usage: read <data-block>\r\n");
        return;
    }

    if (write &&
        !parse_hex(
            hex,
            expected,
            sizeof(expected))) {

        (void)usb_console_write(
            "ERR usage: write <data-block> "
            "<32 hex chars / 16 bytes>\r\n");

        return;
    }


    /* ---------------------------------------------------------
     * Wait for a card
     * --------------------------------------------------------- */

    picc = active_picc();

    if (picc == NULL) {
        (void)usb_console_write(
            "ERR no card detected after 10 seconds\r\n");
        return;
    }


    /* ---------------------------------------------------------
     * Lock RC522 hardware.
     *
     * This is the same mutex passed as:
     *
     *     rc522_config_t.task_mutex
     *
     * so the library polling task cannot touch RC522 while
     * auth/read/write is running.
     * --------------------------------------------------------- */

    if (xSemaphoreTake(
            s_rc522_io_lock,
            pdMS_TO_TICKS(RFID_IO_LOCK_WAIT_MS))
        != pdTRUE) {

        (void)usb_console_write(
            "ERR RFID reader busy\r\n");
        return;
    }


    /* ---------------------------------------------------------
     * Re-check the card after acquiring the hardware lock.
     *
     * It may have been removed while waiting for the mutex.
     * --------------------------------------------------------- */

    xSemaphoreTake(
        s_picc_lock,
        portMAX_DELAY);

    picc = s_picc;

    xSemaphoreGive(
        s_picc_lock);

    if (picc == NULL) {
        (void)usb_console_write(
            "ERR card removed\r\n");

        goto cleanup;
    }


    /* ---------------------------------------------------------
     * Validate block and determine sector
     * --------------------------------------------------------- */

    err = data_block(
        picc,
        block,
        &sector);

    if (err != ESP_OK) {
        (void)usb_console_printf(
            "ERR validate block %u: "
            "0x%X (%s)\r\n",
            (unsigned)block,
            (unsigned)err,
            esp_err_to_name(err));

        goto cleanup;
    }


    /* ---------------------------------------------------------
     * Authenticate sector
     * --------------------------------------------------------- */

    err = rc522_mifare_auth(
        s_scanner,
        picc,
        block,
        &s_keys[sector]);

    if (err != ESP_OK) {
        (void)usb_console_printf(
            "ERR auth block %u sector %u: "
            "0x%X (%s)\r\n",
            (unsigned)block,
            (unsigned)sector,
            (unsigned)err,
            esp_err_to_name(err));

        goto cleanup;
    }

    authenticated = true;


    /* ---------------------------------------------------------
     * WRITE
     * --------------------------------------------------------- */

    if (write) {
        err = rc522_mifare_write(
            s_scanner,
            picc,
            block,
            expected);

        if (err != ESP_OK) {
            (void)usb_console_printf(
                "ERR write block %u: "
                "0x%X (%s)\r\n",
                (unsigned)block,
                (unsigned)err,
                esp_err_to_name(err));

            goto cleanup;
        }
    }


    /* ---------------------------------------------------------
     * READ
     *
     * For normal read:
     *     this returns block contents.
     *
     * For write:
     *     this reads the block back for verification.
     * --------------------------------------------------------- */

    err = rc522_mifare_read(
        s_scanner,
        picc,
        block,
        actual);

    if (err != ESP_OK) {
        (void)usb_console_printf(
            write
                ? "ERR verify-read block %u: 0x%X (%s)\r\n"
                : "ERR read block %u: 0x%X (%s)\r\n",
            (unsigned)block,
            (unsigned)err,
            esp_err_to_name(err));

        goto cleanup;
    }


    /* ---------------------------------------------------------
     * Verify write
     * --------------------------------------------------------- */

    if (write &&
        memcmp(
            expected,
            actual,
            RC522_MIFARE_BLOCK_SIZE) != 0) {

        (void)usb_console_printf(
            "ERR verify mismatch block %u\r\n",
            (unsigned)block);

        (void)usb_console_write(
            "EXPECTED=");

        for (size_t i = 0;
             i < RC522_MIFARE_BLOCK_SIZE;
             ++i) {

            (void)usb_console_printf(
                "%02X",
                (unsigned)expected[i]);
        }

        (void)usb_console_write(
            "\r\nACTUAL=");

        for (size_t i = 0;
             i < RC522_MIFARE_BLOCK_SIZE;
             ++i) {

            (void)usb_console_printf(
                "%02X",
                (unsigned)actual[i]);
        }

        (void)usb_console_write("\r\n");

        err = ESP_FAIL;

        goto cleanup;
    }


    /* ---------------------------------------------------------
     * Success output
     * --------------------------------------------------------- */

    (void)usb_console_printf(
        "B%03u=",
        (unsigned)block);

    for (size_t i = 0;
         i < RC522_MIFARE_BLOCK_SIZE;
         ++i) {

        (void)usb_console_printf(
            "%02X",
            (unsigned)actual[i]);
    }

    (void)usb_console_write(
        write
            ? " verified\r\n"
            : "\r\n");


cleanup:

    /* ---------------------------------------------------------
     * Deauthenticate only if authentication succeeded.
     *
     * Deauth failure is reported separately and must not make
     * a successful read look like a read failure.
     * --------------------------------------------------------- */

    if (authenticated) {
        esp_err_t deauth_err =
            rc522_mifare_deauth(
                s_scanner,
                picc);

        if (deauth_err != ESP_OK) {
            (void)usb_console_printf(
                "WARN deauth failed: "
                "0x%X (%s)\r\n",
                (unsigned)deauth_err,
                esp_err_to_name(deauth_err));
        }
    }


    /* Allow rc522 polling task to continue. */
    xSemaphoreGive(
        s_rc522_io_lock);
}

static void command_key(
    char *type,
    char *sector_text,
    char *hex)
{
    uint8_t key[RC522_MIFARE_KEY_SIZE];

    char *end = NULL;

    if (type == NULL ||
        type[1] ||
        sector_text == NULL ||
        !parse_hex(
            hex,
            key,
            sizeof(key))) {

        goto usage;
    }

    char t =
        (char)toupper(
            (unsigned char)type[0]);

    if (t != 'A' &&
        t != 'B') {

        goto usage;
    }

    unsigned first = 0;
    unsigned last =
        RC522_MIFARE_SECTOR_INDEX_MAX;

    if (strcasecmp(
            sector_text,
            "all") != 0) {

        unsigned long n =
            strtoul(
                sector_text,
                &end,
                10);

        if (end == sector_text ||
            *end ||
            n > last) {

            goto usage;
        }

        first = (unsigned)n;
        last = (unsigned)n;
    }

    for (unsigned i = first;
         i <= last;
         ++i) {

        s_keys[i].type =
            t == 'A'
                ? RC522_MIFARE_KEY_A
                : RC522_MIFARE_KEY_B;

        memcpy(
            s_keys[i].value,
            key,
            sizeof(key));
    }

    (void)usb_console_write(
        "OK key configured in RAM\r\n");

    return;

usage:

    (void)usb_console_write(
        "ERR usage: "
        "key <A|B> <all|0..39> <12 hex>\r\n");
}


/*
 * Called from the rc522 polling task.
 *
 * Note that the polling task already owns
 * s_rc522_io_lock when this callback runs.
 */
static void on_picc_state_changed(
    void *arg,
    esp_event_base_t base,
    int32_t event_id,
    void *data)
{
    (void)arg;
    (void)base;
    (void)event_id;

    const rc522_picc_state_changed_event_t *event =
        data;

    xSemaphoreTake(
        s_picc_lock,
        portMAX_DELAY);

    if (event->picc->state ==
            RC522_PICC_STATE_ACTIVE ||
        event->picc->state ==
            RC522_PICC_STATE_ACTIVE_H) {

        s_picc = event->picc;

        /*
         * Binary semaphore.
         *
         * If already signaled, Give simply fails because
         * it is already full; that's fine.
         */
        (void)xSemaphoreGive(
            s_card_seen);
    }
    else if (
        event->old_state >=
            RC522_PICC_STATE_ACTIVE) {

        s_picc = NULL;

        /*
         * Remove a stale "card seen" signal.
         *
         * Otherwise:
         *
         *   card inserted
         *      -> semaphore = 1
         *
         *   card removed
         *      -> s_picc = NULL
         *
         *   scan
         *      -> consumes old semaphore immediately
         *      -> doesn't actually wait 10 seconds.
         */
        (void)xSemaphoreTake(
            s_card_seen,
            0);
    }

    xSemaphoreGive(
        s_picc_lock);
}


static void process_command(
    char *line)
{
    char *save = NULL;

    char *cmd =
        strtok_r(
            line,
            " \t",
            &save);

    if (cmd == NULL) {
        return;
    }

    if (strcasecmp(cmd, "scan") == 0) {

        command_scan();
    }
    else if (
        strcasecmp(cmd, "read") == 0) {

        command_rw(
            false,
            strtok_r(
                NULL,
                " \t",
                &save),
            NULL);
    }
    else if (
        strcasecmp(cmd, "write") == 0) {

        char *b =
            strtok_r(
                NULL,
                " \t",
                &save);

        char *hex =
            strtok_r(
                NULL,
                " \t",
                &save);

        command_rw(
            true,
            b,
            hex);
    }
    else if (
        strcasecmp(cmd, "key") == 0) {

        char *t =
            strtok_r(
                NULL,
                " \t",
                &save);

        char *s =
            strtok_r(
                NULL,
                " \t",
                &save);

        char *k =
            strtok_r(
                NULL,
                " \t",
                &save);

        command_key(
            t,
            s,
            k);
    }
    else if (
        strcasecmp(cmd, "clear") == 0) {

        reset_keys();

        (void)usb_console_write(
            "OK keys reset to FFFFFFFFFFFF\r\n");
    }
    else {
        (void)usb_console_write(
            "Commands: "
            "scan | "
            "read <block> | "
            "write <block> <32 hex> | "
            "key <A|B> <all|0..39> <12 hex> | "
            "clear\r\n");
    }
}


void app_main(void)
{
    ESP_ERROR_CHECK(
        usb_console_init());

    /*
     * App-level PICC state mutex.
     */
    s_picc_lock =
        xSemaphoreCreateMutex();

    /*
     * Card-arrival notification.
     */
    s_card_seen =
        xSemaphoreCreateBinary();

    /*
     * MFRC522 hardware mutex.
     *
     * This mutex is shared with rc522_polling_task.
     */
    s_rc522_io_lock =
        xSemaphoreCreateMutex();

    assert(
        s_picc_lock &&
        s_card_seen &&
        s_rc522_io_lock);

    reset_keys();

    /*
     * Polling is intentional for card lifecycle events.
     *
     * Suppress expected no-response polling logs.
     * Command errors are still reported by this app.
     */
    esp_log_level_set(
        "rc522",
        ESP_LOG_NONE);


    /*
     * SPI transport.
     */
    rc522_driver_handle_t driver;

    const rc522_spi_config_t spi = {
        .host_id = SPI2_HOST,

        .bus_config =
            &(spi_bus_config_t) {
                .miso_io_num =
                    RFID_MISO_GPIO,

                .mosi_io_num =
                    RFID_MOSI_GPIO,

                .sclk_io_num =
                    RFID_SCLK_GPIO,
            },

        .dev_config = {
            .spics_io_num =
                RFID_CS_GPIO,
        },

        .rst_io_num =
            RFID_RST_GPIO,
    };


    ESP_ERROR_CHECK(
        rc522_spi_create(
            &spi,
            &driver));

    ESP_ERROR_CHECK(
        rc522_driver_install(
            driver));


    /*
     * IMPORTANT:
     *
     * Pass the same mutex used by command_rw().
     *
     * rc522_polling_task will automatically acquire
     * this before REQA/WUPA/select/heartbeat.
     */
    const rc522_config_t scanner_config = {
        .driver = driver,
        .task_mutex = s_rc522_io_lock,
    };

    ESP_ERROR_CHECK(
        rc522_create(
            &scanner_config,
            &s_scanner));


    ESP_ERROR_CHECK(
        rc522_register_events(
            s_scanner,
            RC522_EVENT_PICC_STATE_CHANGED,
            on_picc_state_changed,
            NULL));


    ESP_ERROR_CHECK(
        rc522_start(
            s_scanner));


    (void)usb_console_write(
        "\r\n"
        "ESP32-S3 RC522 ready "
        "(abobija/rc522).\r\n"
        "Type help.\r\n"
        "> ");


    char line[128];

    while (true) {

        if (usb_console_readline(
                line,
                sizeof(line)) == ESP_OK) {

            process_command(line);
        }
        else {
            (void)usb_console_write(
                "ERR command too long\r\n");
        }

        (void)usb_console_write("> ");
    }
}