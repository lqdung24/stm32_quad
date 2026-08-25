#include "usb_console.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"

#define USB_TX_TIMEOUT_MS 1000U

esp_err_t usb_console_init(void)
{
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 512,
    };
    return usb_serial_jtag_driver_install(&config);
}

esp_err_t usb_console_write(const char *text)
{
    if (text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t length = strlen(text);
    const int written = usb_serial_jtag_write_bytes(
        text, length, pdMS_TO_TICKS(USB_TX_TIMEOUT_MS));
    return written == (int)length ? ESP_OK : ESP_FAIL;
}

esp_err_t usb_console_printf(const char *format, ...)
{
    char buffer[512];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= sizeof(buffer)) {
        return ESP_ERR_INVALID_SIZE;
    }
    return usb_console_write(buffer);
}

esp_err_t usb_console_readline(char *line, size_t capacity)
{
    if (line == NULL || capacity < 2U) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t used = 0U;
    bool overflow = false;
    while (true) {
        uint8_t byte;
        const int received = usb_serial_jtag_read_bytes(
            &byte, 1U, pdMS_TO_TICKS(100));
        if (received <= 0) {
            continue;
        }
        if (byte == '\r' || byte == '\n') {
            if (used == 0U && !overflow) {
                continue;
            }
            line[used] = '\0';
            (void)usb_console_write("\r\n");
            return overflow ? ESP_ERR_INVALID_SIZE : ESP_OK;
        }
        if (byte == 0x08U || byte == 0x7fU) {
            if (used > 0U) {
                --used;
                (void)usb_console_write("\b \b");
            }
            continue;
        }
        if (byte < 0x20U || byte > 0x7eU) {
            continue;
        }
        (void)usb_serial_jtag_write_bytes(&byte, 1U,
                                           pdMS_TO_TICKS(USB_TX_TIMEOUT_MS));
        if (used + 1U < capacity) {
            line[used++] = (char)byte;
        } else {
            overflow = true;
        }
    }
}
