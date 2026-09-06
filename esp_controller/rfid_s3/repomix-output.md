This file is a merged representation of the entire codebase, combined into a single document by Repomix.
The content has been processed where content has been compressed (code blocks are separated by ⋮---- delimiter).

# File Summary

## Purpose
This file contains a packed representation of the entire repository's contents.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.

## File Format
The content is organized as follows:
1. This summary section
2. Repository information
3. Directory structure
4. Repository files (if enabled)
5. Multiple file entries, each consisting of:
  a. A header with the file path (## File: path/to/file)
  b. The full contents of the file in a code block

## Usage Guidelines
- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.

## Notes
- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Content has been compressed - code blocks are separated by ⋮---- delimiter
- Files are sorted by Git change count (files with more changes are at the bottom)

# Directory Structure
````
main/
  CMakeLists.txt
  idf_component.yml
  main.c
  usb_console.c
  usb_console.h
CMakeLists.txt
dependencies.lock
README.md
sdkconfig.defaults
````

# Files

## File: main/CMakeLists.txt
````
idf_component_register(
    SRCS
        "main.c"
        "usb_console.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_usb_serial_jtag
        rc522)
````

## File: main/idf_component.yml
````yaml
dependencies:
  abobija/rc522: "^4.0.0"
````

## File: main/main.c
````c
/*
 * Points to the PICC object owned by the rc522 library.
 * NULL means no active card is currently known.
 */
⋮----
/*
 * Protects s_picc.
 */
⋮----
/*
 * Signaled when a card becomes ACTIVE.
 */
⋮----
/*
 * Shared with rc522_config_t.task_mutex.
 *
 * This serializes our MIFARE commands against the library's
 * internal rc522_polling_task.
 */
⋮----
/*
 * Per-sector MIFARE keys.
 *
 * Default:
 *   Key A = FF FF FF FF FF FF
 */
⋮----
static void reset_keys(void)
⋮----
static int hex_nibble(char c)
⋮----
static bool parse_hex(
⋮----
/*
 * Return the currently active PICC.
 *
 * If no card is active, wait up to CARD_WAIT_MS for one.
 */
static const rc522_picc_t *active_picc(void)
⋮----
static void print_picc(
⋮----
/*
 * Only allow ordinary data blocks.
 *
 * Refuse:
 *   - manufacturer block 0
 *   - sector trailers
 */
static esp_err_t data_block(
⋮----
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
⋮----
//     return ESP_ERR_NOT_ALLOWED;
// }
⋮----
static void command_scan(void)
⋮----
static bool parse_block(
⋮----
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
⋮----
/* ---------------------------------------------------------
     * Parse command arguments
     * --------------------------------------------------------- */
⋮----
/* ---------------------------------------------------------
     * Wait for a card
     * --------------------------------------------------------- */
⋮----
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
⋮----
/* ---------------------------------------------------------
     * Re-check the card after acquiring the hardware lock.
     *
     * It may have been removed while waiting for the mutex.
     * --------------------------------------------------------- */
⋮----
/* ---------------------------------------------------------
     * Validate block and determine sector
     * --------------------------------------------------------- */
⋮----
/* ---------------------------------------------------------
     * Authenticate sector
     * --------------------------------------------------------- */
⋮----
/* ---------------------------------------------------------
     * WRITE
     * --------------------------------------------------------- */
⋮----
/* ---------------------------------------------------------
     * READ
     *
     * For normal read:
     *     this returns block contents.
     *
     * For write:
     *     this reads the block back for verification.
     * --------------------------------------------------------- */
⋮----
/* ---------------------------------------------------------
     * Verify write
     * --------------------------------------------------------- */
⋮----
/* ---------------------------------------------------------
     * Success output
     * --------------------------------------------------------- */
⋮----
/* ---------------------------------------------------------
     * Deauthenticate only if authentication succeeded.
     *
     * Deauth failure is reported separately and must not make
     * a successful read look like a read failure.
     * --------------------------------------------------------- */
⋮----
/* Allow rc522 polling task to continue. */
⋮----
static void command_key(
⋮----
/*
 * Called from the rc522 polling task.
 *
 * Note that the polling task already owns
 * s_rc522_io_lock when this callback runs.
 */
static void on_picc_state_changed(
⋮----
/*
         * Binary semaphore.
         *
         * If already signaled, Give simply fails because
         * it is already full; that's fine.
         */
⋮----
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
⋮----
static void process_command(
⋮----
void app_main(void)
⋮----
/*
     * App-level PICC state mutex.
     */
⋮----
/*
     * Card-arrival notification.
     */
⋮----
/*
     * MFRC522 hardware mutex.
     *
     * This mutex is shared with rc522_polling_task.
     */
⋮----
/*
     * Polling is intentional for card lifecycle events.
     *
     * Suppress expected no-response polling logs.
     * Command errors are still reported by this app.
     */
⋮----
/*
     * SPI transport.
     */
⋮----
/*
     * IMPORTANT:
     *
     * Pass the same mutex used by command_rw().
     *
     * rc522_polling_task will automatically acquire
     * this before REQA/WUPA/select/heartbeat.
     */
````

## File: main/usb_console.c
````c
esp_err_t usb_console_init(void)
⋮----
esp_err_t usb_console_write(const char *text)
⋮----
esp_err_t usb_console_printf(const char *format, ...)
⋮----
esp_err_t usb_console_readline(char *line, size_t capacity)
````

## File: main/usb_console.h
````c
esp_err_t usb_console_init(void);
esp_err_t usb_console_readline(char *line, size_t capacity);
esp_err_t usb_console_write(const char *text);
esp_err_t usb_console_printf(const char *format, ...)
````

## File: CMakeLists.txt
````
cmake_minimum_required(VERSION 3.22)

set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rfid_copy_s3)
````

## File: dependencies.lock
````
dependencies:
  abobija/rc522:
    component_hash: 3302c1e7e19ee75c37e3e7284e86d0f624b56b7c5c55929c4ed1a1bdd28b62c5
    dependencies:
    - name: idf
      require: private
      version: '>=5.0'
    source:
      registry_url: https://components.espressif.com/
      type: service
    version: 4.0.0
  idf:
    source:
      type: idf
    version: 6.0.0
direct_dependencies:
- abobija/rc522
manifest_hash: 0900fd61ad8d8f372907101d4d087f4f139d00be496c05ec781e4c3bad91c47f
target: esp32s3
version: 3.0.0
````

## File: README.md
````markdown
# ESP32-S3 + MFRC522 card reader/copy tool

Standalone ESP-IDF firmware for an ESP32-S3-WROOM-1-N16R8. It uses the
[`abobija/rc522`](https://components.espressif.com/components/abobija/rc522)
ESP-IDF component for event-driven card detection and native MIFARE Classic
authentication, raw block reads, and verified raw block writes through the
ESP32-S3 native USB Serial/JTAG port.

## Wiring

| RC522 pin | ESP32-S3 | Notes |
|---|---:|---|
| 3.3V | 3V3 | Never connect the RC522 to 5 V |
| GND | GND | Common ground |
| SCK | GPIO10 | SPI2 clock |
| MOSI | GPIO11 | SPI2 controller to RC522 |
| SDA / SS | GPIO12 | SPI chip select; this is not I2C SDA |
| MISO | GPIO13 | RC522 to SPI2 controller |
| RST | GPIO14 | Reset |
| IRQ | not connected | Polling is used |

GPIO19/20 are deliberately left free for native USB D-/D+. GPIO0, GPIO3,
GPIO45, and GPIO46 are avoided because they are strapping pins.

## Build, flash, and connect

Use ESP-IDF v5.5 or newer:

```sh
idf.py -C rfid_s3 build
idf.py -C rfid_s3 -p /dev/ttyACM0 flash
idf.py -C rfid_s3 -p /dev/ttyACM0 monitor
```

Do not run a second serial monitor while issuing commands. Any serial terminal
can use the same native USB Serial/JTAG port. The baud-rate setting is ignored
by native USB.

## Commands

- `scan` reads UID, SAK, and ATQA.
- `read 4` authenticates and prints data block 4 as 32 hexadecimal characters.
- `write 4 00112233445566778899AABBCCDDEEFF` authenticates, writes block 4,
  reads it back, and prints the verified value.
- `key A all FFFFFFFFFFFF` sets an authentication key for every sector.
- `key B 7 A0A1A2A3A4A5` sets Key B for sector 7.
- `clear` resets all keys to `FFFFFFFFFFFF`.

Only MIFARE Classic Mini, 1K, and 4K geometry is supported. The firmware does
not guess keys. Block 0 and every sector trailer are intentionally excluded
from reads and writes, so access conditions and manufacturer data cannot be
changed from the console. Commands wait up to 10 seconds for a card.

Use this tool only with cards and systems you own or are authorized to test.
````

## File: sdkconfig.defaults
````
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_COMPILER_OPTIMIZATION_SIZE=y
CONFIG_ESP_CONSOLE_NONE=y
# CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG is not set
CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y
````
