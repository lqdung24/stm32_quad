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
include/
  bridge_config.h
  espnow_transport.h
  packet_stream.h
  status_led.h
src/
  espnow_transport.c
  packet_stream.c
  status_led.c
CMakeLists.txt
idf_component.yml
```

# Files

## File: include/bridge_config.h
```c
/*
 * Edit both STA MAC addresses here. Ground and Air select their peer
 * explicitly; the firmware role is never inferred from the chip target.
 */
⋮----
//  esp32s3: MAC:                1c:db:d4:4a:f8:e0
//  esp32: MAC:                08:a6:f7:b1:43:c4
```

## File: include/espnow_transport.h
```c
} espnow_transport_stats_t;
⋮----
} espnow_transport_link_info_t;
⋮----
/* ESP-NOW carries a complete, unmodified DroneProtocol packet per message. */
esp_err_t espnow_transport_start(espnow_transport_packet_callback_t callback,
⋮----
esp_err_t espnow_transport_send_packet(const uint8_t *packet, size_t length);
void espnow_transport_get_stats(espnow_transport_stats_t *stats);
esp_err_t espnow_transport_get_link_info(espnow_transport_link_info_t *info);
```

## File: include/packet_stream.h
```c
} packet_stream_t;
⋮----
/* Shared COBS/0x00 framing for byte streams such as USB CDC and UART. */
void packet_stream_init(packet_stream_t *stream, packet_stream_callback_t callback,
⋮----
void packet_stream_feed(packet_stream_t *stream, const uint8_t *bytes,
⋮----
/* Writes a leading and trailing delimiter, so console text cannot corrupt sync. */
size_t packet_stream_encode(const uint8_t *packet, size_t packet_length,
```

## File: include/status_led.h
```c
} status_led_state_t;
⋮----
/* Initialize the status LED (WS2812 on Ground, ordinary GPIO on Air). */
esp_err_t status_led_init(void);
⋮----
/* This function is safe to call from normal FreeRTOS task/event contexts. */
void status_led_set(status_led_state_t state);
```

## File: src/espnow_transport.c
```c
} espnow_packet_t;
⋮----
static void increment_stat(uint32_t *counter)
⋮----
static void espnow_receive_callback(const esp_now_recv_info_t *info,
⋮----
static void espnow_send_callback(const esp_now_send_info_t *info,
⋮----
static void espnow_rx_task(void *arg)
⋮----
esp_err_t espnow_transport_start(espnow_transport_packet_callback_t callback,
⋮----
esp_err_t espnow_transport_send_packet(const uint8_t *packet, size_t length)
⋮----
void espnow_transport_get_stats(espnow_transport_stats_t *stats)
⋮----
esp_err_t espnow_transport_get_link_info(espnow_transport_link_info_t *info)
```

## File: src/packet_stream.c
```c
#include "packet_stream.h" /* shared by the USB and UART transports */
⋮----
void packet_stream_init(packet_stream_t *stream, packet_stream_callback_t callback,
⋮----
void packet_stream_feed(packet_stream_t *stream, const uint8_t *bytes,
⋮----
/* Ignore the rest of an oversized frame until its delimiter. */
⋮----
size_t packet_stream_encode(const uint8_t *packet, size_t packet_length,
```

## File: src/status_led.c
```c
} status_led_color_t;
⋮----
static const char *TAG = "status_led"; /* Air: ordinary GPIO LED. */
⋮----
static const char *TAG = "status_led"; /* Ground: WS2812 RGB LED. */
⋮----
static status_led_color_t status_led_color_for(status_led_state_t state,
⋮----
/* Amber blink: firmware is starting. */
⋮----
/* Blue blink: the ground bridge is ready for a USB controller. */
⋮----
/* Cyan blink: the ESP-NOW bridge is linked; STM32 status is pending. */
⋮----
/* Red is deliberately steady while motors are armed. */
⋮----
/* Two short red flashes repeat for failsafe or STM32 error. */
⋮----
static bool status_led_colors_equal(status_led_color_t left,
⋮----
static bool status_led_is_on(status_led_color_t color)
⋮----
static void status_led_task(void *arg)
⋮----
esp_err_t status_led_init(void)
⋮----
void status_led_set(status_led_state_t state)
```

## File: CMakeLists.txt
```
set(DRONE_PROTOCOL_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../../stm32cube/Components/DroneProtocol")

idf_component_register(
    SRCS
        "src/espnow_transport.c"
        "src/packet_stream.c"
        "src/status_led.c"
        "${DRONE_PROTOCOL_DIR}/Src/dp_protocol.c"
        "${DRONE_PROTOCOL_DIR}/Src/dp_bytes.c"
        "${DRONE_PROTOCOL_DIR}/Src/dp_cobs.c"
    INCLUDE_DIRS
        "include"
        "${DRONE_PROTOCOL_DIR}/Inc"
    REQUIRES
        esp_event
        esp_wifi
        led_strip)
```

## File: idf_component.yml
```yaml
# Dependencies used by the shared status LED implementation.
dependencies:
  espressif/led_strip: "^3.0.0"
```
