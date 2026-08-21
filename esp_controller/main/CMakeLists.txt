set(drone_bridge_sources
    "main.c"
    "status_led.c"
    "packet_stream.c"
    "espnow_transport.c"
    "uart_transport.c"
    "../../stm32cube/Components/DroneProtocol/Src/dp_protocol.c"
    "../../stm32cube/Components/DroneProtocol/Src/dp_bytes.c"
    "../../stm32cube/Components/DroneProtocol/Src/dp_cobs.c")
set(drone_bridge_requires esp_driver_uart esp_event esp_wifi nvs_flash)

# Native USB Serial/JTAG exists only on ESP32-S3. Checking the target here
# keeps the dependency available while CMake discovers project Kconfig values.
if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND drone_bridge_requires esp_driver_usb_serial_jtag)
endif()

# ESP32-S3 is Ground; ESP32 is Air. Role no longer depends on sdkconfig.
if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND drone_bridge_sources "usb_transport.c")
endif()

idf_component_register(
    SRCS ${drone_bridge_sources}
    INCLUDE_DIRS "."
        "../../stm32cube/Components/DroneProtocol/Inc"
    REQUIRES
        ${drone_bridge_requires}
)
