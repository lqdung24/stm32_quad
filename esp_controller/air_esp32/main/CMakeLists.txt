idf_component_register(
    SRCS
        "main.c"
        "uart_transport.c"
    INCLUDE_DIRS "."
    REQUIRES
        common
        esp_driver_uart
        nvs_flash)
