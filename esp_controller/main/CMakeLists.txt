idf_component_register(
    SRCS
        "main.c"
        "control_bridge.c"
        "http_server.c"
        "uart_transport.c"
        "websocket_server.c"
        "wifi_ap.c"
        "../../stm32cube/Components/DroneProtocol/Src/dp_protocol.c"
        "../../stm32cube/Components/DroneProtocol/Src/dp_bytes.c"
        "../../stm32cube/Components/DroneProtocol/Src/dp_cobs.c"
    INCLUDE_DIRS "."
        "../../stm32cube/Components/DroneProtocol/Inc"
    REQUIRES
        esp_driver_uart
        esp_event
        esp_http_server
        esp_netif
        esp_timer
        esp_wifi
        nvs_flash
    EMBED_TXTFILES "web/index.html"
)
