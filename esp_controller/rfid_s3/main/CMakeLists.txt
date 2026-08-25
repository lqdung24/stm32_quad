idf_component_register(
    SRCS
        "main.c"
        "mfrc522.c"
        "usb_console.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_driver_usb_serial_jtag
        esp_timer)
