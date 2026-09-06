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
  uart_transport.c
  uart_transport.h
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
        "uart_transport.c"
    INCLUDE_DIRS "."
    REQUIRES
        common
        esp_driver_uart
        nvs_flash)
```

## File: main/Kconfig.projbuild
```
menu "Drone Air bridge"
    menu "STM32 UART link"
        config DRONE_UART_PORT
            int "UART controller number"
            range 0 2
            default 1
        config DRONE_UART_TX_GPIO
            int "ESP TX GPIO (connect to STM32 USART1 RX/PA10)"
            range 0 39
            default 17
        config DRONE_UART_RX_GPIO
            int "ESP RX GPIO (connect to STM32 USART1 TX/PA9)"
            range 0 39
            default 18
        config DRONE_UART_BAUD_RATE
            int "UART baud rate"
            range 115200 921600
            default 460800
    endmenu

    config DRONE_STATUS_LED_SINGLE_COLOR
        bool "Use a single-color status LED"
        default y
        help
            Drive the status LED as a normal active-high GPIO output rather
            than a WS2812 RGB LED.

    config DRONE_STATUS_LED_GPIO
        int "Single-color status LED GPIO"
        range 0 39
        default 2
endmenu
```

## File: main/main.c
```c
} air_link_state_t;
⋮----
static uint32_t monotonic_ms(void)
⋮----
static bool decode_control(const uint8_t *packet, size_t length,
⋮----
static void log_espnow_link_config(void)
⋮----
static void update_air_led(const DroneSystemStatus *status)
⋮----
static void remember_ground_control(const DroneControlCommand *command)
⋮----
/* ESP-NOW -> STM32 UART. STM32 owns all motor and failsafe decisions. */
static void on_espnow_packet(const uint8_t *packet, size_t length)
⋮----
/* STM32 UART -> ESP-NOW. Forward protocol packets unchanged to the laptop. */
static void on_uart_packet(const uint8_t *packet, size_t length)
⋮----
static void send_stm32_link_lost_status(const DroneControlCommand *last_control,
⋮----
static void air_link_monitor_task(void *arg)
⋮----
void app_main(void)
```

## File: main/uart_transport.c
```c
static const char *TAG = "uart_transport"; /* Air only */
⋮----
static void uart_packet_received(const uint8_t *packet, size_t length,
⋮----
static void uart_rx_task(void *arg)
⋮----
esp_err_t uart_transport_start(uart_transport_packet_callback_t callback)
⋮----
esp_err_t uart_transport_send_packet(const uint8_t *packet, size_t length)
```

## File: main/uart_transport.h
```c
/* Air-only transport between ESP32 and the STM32 flight controller. */
⋮----
esp_err_t uart_transport_start(uart_transport_packet_callback_t callback);
esp_err_t uart_transport_send_packet(const uint8_t *packet, size_t length);
```

## File: CMakeLists.txt
```
cmake_minimum_required(VERSION 3.22)

# This project always builds for the Air ESP32.
set(IDF_TARGET "esp32")
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../common")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_air_esp32)
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
target: esp32
version: 3.0.0
```

## File: sdkconfig.defaults
```
CONFIG_COMPILER_OPTIMIZATION_SIZE=y
CONFIG_ESP_WIFI_SOFTAP_SUPPORT=n
CONFIG_ESP_CONSOLE_SECONDARY_NONE=y
CONFIG_DRONE_STATUS_LED_SINGLE_COLOR=y
```
