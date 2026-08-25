#pragma once

#include <stddef.h>

#include "esp_err.h"

esp_err_t usb_console_init(void);
esp_err_t usb_console_readline(char *line, size_t capacity);
esp_err_t usb_console_write(const char *text);
esp_err_t usb_console_printf(const char *format, ...)
    __attribute__((format(printf, 1, 2)));
