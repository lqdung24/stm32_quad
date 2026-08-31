#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_err.h"
#include "mfrc522.h"
#include "usb_console.h"

#define RFID_SCLK_GPIO GPIO_NUM_10
#define RFID_MOSI_GPIO GPIO_NUM_11
#define RFID_CS_GPIO GPIO_NUM_12
#define RFID_MISO_GPIO GPIO_NUM_13
#define RFID_RST_GPIO GPIO_NUM_14
#define RFID_SPI_HZ 4000000U

static mfrc522_t s_reader;
static mfrc522_snapshot_t s_snapshot;
static bool s_snapshot_ready;
static mfrc522_sector_key_t s_keys[MFRC522_MAX_SECTORS];

static void print_uid(const mfrc522_card_t *card)
{
    (void)usb_console_write("UID=");
    for (uint8_t i = 0U; i < card->uid_length; ++i)
    {
        (void)usb_console_printf("%02X", card->uid[i]);
    }
}

static void print_card(const mfrc522_card_t *card)
{
    print_uid(card);
    (void)usb_console_printf(" SAK=%02X ATQA=%02X%02X\r\n", card->sak,
                             card->atqa[0], card->atqa[1]);
}

static void print_help(void)
{
    (void)usb_console_write(
        "Commands:\r\n"
        "  scan                         read UID/type of card\r\n"
        "  copy                         read UID and all accessible data blocks\r\n"
        "  dump                         print snapshot currently in RAM\r\n"
        "  write                        write/verify data blocks to target card\r\n"
        "  uid <8 hex|snapshot>         rewrite UID of a CUID/gen2 card\r\n"
        "       [sak <hh>] [atqa <hhhh>]\r\n"
        "  key <A|B> <all|0..39> <hex> set 6-byte auth key in RAM\r\n"
        "  clear                        erase snapshot and configured keys\r\n"
        "  help                         show this text\r\n"
        "Notes: write never touches block 0 or sector trailers. UID on standard\r\n"
        "MIFARE Classic cards is immutable; use uid for CUID/gen2 cards only.\r\n"
        "uid recomputes the BCC and keeps SAK/ATQA/manufacturer bytes unless\r\n"
        "overridden.\r\n");
}

static void reset_keys(void)
{
    for (size_t sector = 0U; sector < MFRC522_MAX_SECTORS; ++sector)
    {
        memset(s_keys[sector].bytes, 0xff, sizeof(s_keys[sector].bytes));
        s_keys[sector].type = MFRC522_KEY_A;
    }
}

static int hex_nibble(char value)
{
    if (value >= '0' && value <= '9')
    {
        return value - '0';
    }
    value = (char)toupper((unsigned char)value);
    return (value >= 'A' && value <= 'F') ? value - 'A' + 10 : -1;
}

static bool parse_hex(const char *text, uint8_t *bytes, size_t count)
{
    if (text == NULL || strlen(text) != count * 2U)
    {
        return false;
    }
    for (size_t i = 0U; i < count; ++i)
    {
        const int high = hex_nibble(text[i * 2U]);
        const int low = hex_nibble(text[i * 2U + 1U]);
        if (high < 0 || low < 0)
        {
            return false;
        }
        bytes[i] = (uint8_t)((high << 4) | low);
    }
    return true;
}

static bool parse_key(const char *text, uint8_t key[6])
{
    return parse_hex(text, key, 6U);
}

static void command_scan(void)
{
    mfrc522_card_t card;
    const esp_err_t err = mfrc522_wait_for_card(&s_reader, &card);
    if (err != ESP_OK)
    {
        (void)usb_console_printf(
            "ERR no readable card after 10 seconds (%s)\r\n",
            esp_err_to_name(err));
        return;
    }
    print_card(&card);
    /* A selected card stays ACTIVE and would answer neither REQA nor WUPA on
     * the next scan until it is lifted off the antenna. */
    mfrc522_release(&s_reader);
}

static void command_copy(void)
{
    uint16_t blocks = 0U;
    uint8_t failed_sectors = 0U;
    (void)usb_console_write("Place source card on reader...\r\n");
    const esp_err_t err = mfrc522_read_snapshot(
        &s_reader, s_keys, &s_snapshot, &blocks, &failed_sectors);
    if (err != ESP_OK)
    {
        s_snapshot_ready = false;
        (void)usb_console_printf("ERR copy failed (%s)\r\n",
                                 esp_err_to_name(err));
        return;
    }
    s_snapshot_ready = true;
    (void)usb_console_write("COPIED ");
    print_card(&s_snapshot.card);
    (void)usb_console_printf("blocks=%u/%u failed_sectors=%u RAM-only\r\n",
                             blocks, s_snapshot.block_count,
                             failed_sectors);
}

static void command_dump(void)
{
    if (!s_snapshot_ready)
    {
        (void)usb_console_write("ERR no snapshot; run copy first\r\n");
        return;
    }
    print_card(&s_snapshot.card);
    for (uint8_t sector = 0U; sector < s_snapshot.sector_count; ++sector)
    {
        const uint16_t first = mfrc522_sector_first_block(sector);
        const uint16_t trailer = mfrc522_sector_trailer_block(sector);
        (void)usb_console_printf("SECTOR %u\r\n", sector);
        for (uint16_t block = first; block < trailer; ++block)
        {
            if (!mfrc522_snapshot_block_valid(&s_snapshot, block))
            {
                (void)usb_console_printf("  B%03u: --\r\n", block);
                continue;
            }
            (void)usb_console_printf("  B%03u: ", block);
            for (size_t byte = 0U; byte < MFRC522_BLOCK_BYTES; ++byte)
            {
                (void)usb_console_printf("%02X", s_snapshot.blocks[block][byte]);
            }
            (void)usb_console_write("\r\n");
        }
    }
}

static void command_write(void)
{
    if (!s_snapshot_ready)
    {
        (void)usb_console_write("ERR no snapshot; run copy first\r\n");
        return;
    }
    (void)usb_console_write("Place target card on reader...\r\n");
    mfrc522_card_t target;
    mfrc522_write_stats_t stats;
    const esp_err_t err = mfrc522_write_snapshot(
        &s_reader, s_keys, &s_snapshot, &target, &stats);
    if (target.uid_length > 0U)
    {
        (void)usb_console_write("TARGET ");
        print_card(&target);
        const bool same_uid = target.uid_length == s_snapshot.card.uid_length &&
                              memcmp(target.uid, s_snapshot.card.uid, target.uid_length) == 0;
        (void)usb_console_printf("UID %s; data copy continued. Use uid to "
                                 "rewrite the UID of a CUID/gen2 card\r\n",
                                 same_uid ? "already matches" : "differs");
    }
    (void)usb_console_printf("WRITE attempted=%u verified=%u failed=%u result=%s\r\n",
                             stats.attempted, stats.succeeded, stats.failed,
                             esp_err_to_name(err));
}

static void print_block0(const char *label, const uint8_t block[MFRC522_BLOCK_BYTES])
{
    (void)usb_console_printf("  %s ", label);
    for (size_t byte = 0U; byte < MFRC522_BLOCK_BYTES; ++byte)
    {
        (void)usb_console_printf("%02X", block[byte]);
    }
    (void)usb_console_write("\r\n");
}

static void command_uid(char *uid_text, char *save)
{
    mfrc522_uid_write_t request = {0};

    if (uid_text != NULL && strcasecmp(uid_text, "snapshot") == 0)
    {
        if (!s_snapshot_ready)
        {
            (void)usb_console_write("ERR no snapshot; run copy first\r\n");
            return;
        }
        if (s_snapshot.card.uid_length != MFRC522_UID_WRITE_BYTES)
        {
            (void)usb_console_printf(
                "ERR snapshot UID is %u bytes; only 4-byte UIDs can be "
                "written\r\n",
                s_snapshot.card.uid_length);
            return;
        }
        memcpy(request.uid, s_snapshot.card.uid, MFRC522_UID_WRITE_BYTES);
    }
    else if (!parse_hex(uid_text, request.uid, MFRC522_UID_WRITE_BYTES))
    {
        (void)usb_console_write(
            "ERR usage: uid <8 hex|snapshot> [sak <hh>] [atqa <hhhh>]\r\n");
        return;
    }

    /* Optional overrides; anything not given is preserved from the target. */
    for (char *option = strtok_r(NULL, " \t", &save); option != NULL;
         option = strtok_r(NULL, " \t", &save))
    {
        char *value = strtok_r(NULL, " \t", &save);
        if (strcasecmp(option, "sak") == 0 &&
            parse_hex(value, &request.sak, 1U))
        {
            request.set_sak = true;
        }
        else if (strcasecmp(option, "atqa") == 0 &&
                 parse_hex(value, request.atqa, 2U))
        {
            request.set_atqa = true;
        }
        else
        {
            (void)usb_console_write(
                "ERR usage: uid <8 hex|snapshot> [sak <hh>] [atqa <hhhh>]\r\n");
            return;
        }
    }

    (void)usb_console_write("Place UID-writable (CUID/gen2) card on reader...\r\n");
    mfrc522_uid_write_result_t result;
    const esp_err_t err = mfrc522_write_uid(&s_reader, s_keys, &request,
                                            &result);

    if (result.before.uid_length > 0U)
    {
        (void)usb_console_write("BEFORE ");
        print_card(&result.before);
    }
    if (result.block0_written)
    {
        print_block0("old B000:", result.old_block0);
        print_block0("new B000:", result.new_block0);
    }
    if (result.reselected)
    {
        (void)usb_console_write("AFTER  ");
        print_card(&result.after);
    }

    if (err == ESP_OK)
    {
        (void)usb_console_write("OK UID written and verified\r\n");
        return;
    }

    switch (err)
    {
    case ESP_ERR_NOT_SUPPORTED:
        (void)usb_console_write(
            result.before.uid_length == MFRC522_UID_WRITE_BYTES
                ? "ERR card rejected the block 0 write; not a CUID/gen2 card\r\n"
                : "ERR only 4-byte-UID MIFARE Classic cards are supported\r\n");
        break;
    case ESP_ERR_INVALID_STATE:
        (void)usb_console_write(
            "ERR sector 0 authentication failed; set the right key first\r\n");
        break;
    case ESP_ERR_INVALID_RESPONSE:
        (void)usb_console_printf(
            "ERR card stopped answering after the write (written=%s)\r\n",
            result.block0_written ? "yes" : "no");
        break;
    default:
        (void)usb_console_printf(
            "ERR uid write failed (%s) written=%s reselected=%s uid_match=%s "
            "verified=%s\r\n",
            esp_err_to_name(err), result.block0_written ? "yes" : "no",
            result.reselected ? "yes" : "no",
            result.uid_matches ? "yes" : "no",
            result.block0_verified ? "yes" : "no");
        break;
    }
}

static void command_key(char *type_text, char *sector_text, char *key_text)
{
    uint8_t key[6];
    if (type_text == NULL || sector_text == NULL || key_text == NULL ||
        type_text[1] != '\0' ||
        (toupper((unsigned char)type_text[0]) != 'A' &&
         toupper((unsigned char)type_text[0]) != 'B') ||
        !parse_key(key_text, key))
    {
        (void)usb_console_write("ERR usage: key <A|B> <all|0..39> <12 hex>\r\n");
        return;
    }
    const mfrc522_key_type_t type =
        toupper((unsigned char)type_text[0]) == 'B' ? MFRC522_KEY_B
                                                    : MFRC522_KEY_A;
    unsigned first;
    unsigned last;
    if (strcasecmp(sector_text, "all") == 0)
    {
        first = 0U;
        last = MFRC522_MAX_SECTORS - 1U;
    }
    else
    {
        char *end = NULL;
        const unsigned long parsed = strtoul(sector_text, &end, 10);
        if (end == sector_text || *end != '\0' ||
            parsed >= MFRC522_MAX_SECTORS)
        {
            (void)usb_console_write("ERR sector must be all or 0..39\r\n");
            return;
        }
        first = (unsigned)parsed;
        last = first;
    }
    for (unsigned sector = first; sector <= last; ++sector)
    {
        memcpy(s_keys[sector].bytes, key, sizeof(key));
        s_keys[sector].type = type;
    }
    (void)usb_console_printf("OK key %c set for %s (RAM-only)\r\n",
                             type == MFRC522_KEY_B ? 'B' : 'A', sector_text);
}

static void process_command(char *line)
{
    char *save = NULL;
    char *command = strtok_r(line, " \t", &save);
    if (command == NULL)
    {
        return;
    }
    if (strcasecmp(command, "help") == 0)
    {
        print_help();
    }
    else if (strcasecmp(command, "scan") == 0)
    {
        command_scan();
    }
    else if (strcasecmp(command, "copy") == 0)
    {
        command_copy();
    }
    else if (strcasecmp(command, "dump") == 0)
    {
        command_dump();
    }
    else if (strcasecmp(command, "write") == 0)
    {
        command_write();
    }
    else if (strcasecmp(command, "uid") == 0)
    {
        /* Sequenced deliberately: argument evaluation order is unspecified, so
         * `save` must be read after the UID token has been consumed. */
        char *uid_text = strtok_r(NULL, " \t", &save);
        command_uid(uid_text, save);
    }
    else if (strcasecmp(command, "key") == 0)
    {
        /* Each token must be consumed in a separate statement; as sibling
         * arguments the three calls could run in any order. */
        char *type_text = strtok_r(NULL, " \t", &save);
        char *sector_text = strtok_r(NULL, " \t", &save);
        char *key_text = strtok_r(NULL, " \t", &save);
        command_key(type_text, sector_text, key_text);
    }
    else if (strcasecmp(command, "clear") == 0)
    {
        memset(&s_snapshot, 0, sizeof(s_snapshot));
        s_snapshot_ready = false;
        reset_keys();
        (void)usb_console_write("OK snapshot erased; keys reset to FFFFFFFFFFFF\r\n");
    }
    else
    {
        (void)usb_console_write("ERR unknown command; type help\r\n");
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(usb_console_init());
    reset_keys();

    const mfrc522_config_t config = {
        .host = SPI2_HOST,
        .sclk_io = RFID_SCLK_GPIO,
        .mosi_io = RFID_MOSI_GPIO,
        .miso_io = RFID_MISO_GPIO,
        .cs_io = RFID_CS_GPIO,
        .rst_io = RFID_RST_GPIO,
        .clock_hz = RFID_SPI_HZ,
    };
    const esp_err_t err = mfrc522_init(&s_reader, &config);
    if (err != ESP_OK)
    {
        (void)usb_console_printf("FATAL RC522 init failed: %s\r\n",
                                 esp_err_to_name(err));
        return;
    }

    (void)usb_console_write(
        "\r\nESP32-S3 RC522 copier ready\r\n"
        "SPI: SCK=GPIO10 MOSI=GPIO11 MISO=GPIO13 SS=GPIO12 RST=GPIO14\r\n"
        "Power RC522 from 3.3V only. Type help.\r\n> ");
    char line[128];
    while (true)
    {
        const esp_err_t read_err = usb_console_readline(line, sizeof(line));
        if (read_err == ESP_OK)
        {
            process_command(line);
        }
        else
        {
            (void)usb_console_write("ERR command too long\r\n");
        }
        (void)usb_console_write("> ");
    }
}
