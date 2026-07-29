idf_component_register(
    SRCS
        "main.c"
        "control_bridge.c"
        "http_server.c"
        "uart_transport.c"
        "websocket_server.c"
        "wifi_ap.c"
        "../../Components/DroneProtocol/Src/drone_protocol.c"
        "../../Components/DroneProtocol/Src/drone_cobs.c"
    INCLUDE_DIRS "."
        "../../Components/DroneProtocol/Inc"
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
