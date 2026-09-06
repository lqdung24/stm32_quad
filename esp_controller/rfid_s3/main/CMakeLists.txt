idf_component_register(
    SRCS
        "main.c"
        "usb_console.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_usb_serial_jtag
        rc522)
