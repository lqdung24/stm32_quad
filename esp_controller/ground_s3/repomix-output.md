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
```
main/
  CMakeLists.txt
  Kconfig.projbuild
  main.c
  usb_transport.c
  usb_transport.h
CMakeLists.txt
dependencies.lock
sdkconfig.defaults
```

# Files

## File: main/CMakeLists.txt
```
idf_component_register(
    SRCS
        "main.c"
        "usb_transport.c"
    INCLUDE_DIRS "."
    REQUIRES
        common
        esp_driver_usb_serial_jtag
        nvs_flash)
```

## File: main/Kconfig.projbuild
```
menu "Drone Ground bridge"
    config DRONE_STATUS_LED_GPIO
        int "WS2812 data GPIO"
        range 0 48
        default 48
endmenu
```

## File: main/main.c
```c
} ground_link_state_t;
⋮----
static uint32_t monotonic_ms(void)
⋮----
static bool decode_control(const uint8_t *packet, size_t length,
⋮----
static void log_espnow_link_config(void)
⋮----
/* Laptop Web Serial -> ESP-NOW. The DroneProtocol packet stays unmodified. */
static void on_usb_packet(const uint8_t *packet, size_t length)
⋮----
/*
 * USB has no reliable connect/disconnect event for the Web Serial client.
 * After its control stream has been quiet for LINK_TIMEOUT_MS, keep the
 * radio link alive with fresh, explicit disarm commands.  Each command must
 * have a new sequence number: STM32 deliberately rejects duplicate sequence
 * numbers and would otherwise let its command watchdog expire.
 */
static void ground_control_keepalive_task(void *arg)
⋮----
/* ESP-NOW -> laptop Web Serial: status and telemetry remain raw packets. */
static void on_espnow_packet(const uint8_t *packet, size_t length)
⋮----
static const char *link_text(bool online)
⋮----
static void ground_link_log_task(void *arg)
⋮----
void app_main(void)
```

## File: main/usb_transport.c
```c
static const char *TAG = "usb_transport"; /* Ground only */
⋮----
static void usb_packet_received(const uint8_t *packet, size_t length, void *context)
⋮----
static void usb_rx_task(void *arg)
⋮----
esp_err_t usb_transport_start(usb_transport_packet_callback_t callback)
⋮----
esp_err_t usb_transport_send_packet(const uint8_t *packet, size_t length)
⋮----
esp_err_t usb_transport_send_debug_line(const char *line)
```

## File: main/usb_transport.h
```c
/* Ground-only native ESP32-S3 USB Serial/JTAG transport. */
esp_err_t usb_transport_start(usb_transport_packet_callback_t callback);
esp_err_t usb_transport_send_packet(const uint8_t *packet, size_t length);
⋮----
/* Zero-delimited text for monitor/debug; protocol readers safely discard it. */
esp_err_t usb_transport_send_debug_line(const char *line);
```

## File: CMakeLists.txt
```
cmake_minimum_required(VERSION 3.22)

# This project always builds for the Ground ESP32-S3.
set(IDF_TARGET "esp32s3")
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../common")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_ground_s3)
```

## File: dependencies.lock
```
dependencies:
  espressif/led_strip:
    component_hash: 28621486f77229aaf81c71f5e15d6fbf36c2949cf11094e07090593e659e7639
    dependencies:
    - name: idf
      require: private
      version: '>=5.0'
    source:
      registry_url: https://components.espressif.com/
      type: service
    version: 3.0.3
  idf:
    source:
      type: idf
    version: 6.0.0
direct_dependencies:
- espressif/led_strip
manifest_hash: 6ed48909b74f563b9aa9ffeb66b67024552396a553c3dc2bdb8fedd0d81081a2
target: esp32s3
version: 3.0.0
```

## File: sdkconfig.defaults
```
CONFIG_COMPILER_OPTIMIZATION_SIZE=y
CONFIG_ESP_WIFI_SOFTAP_SUPPORT=n
CONFIG_ESP_CONSOLE_SECONDARY_NONE=y
# CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG is not set
```
