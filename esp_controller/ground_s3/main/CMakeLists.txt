idf_component_register(
    SRCS
        "main.c"
        "usb_transport.c"
    INCLUDE_DIRS "."
    REQUIRES
        common
        esp_driver_usb_serial_jtag
        nvs_flash)
