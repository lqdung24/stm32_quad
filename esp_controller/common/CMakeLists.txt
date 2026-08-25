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
