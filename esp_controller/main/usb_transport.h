#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef void (*usb_transport_packet_callback_t)(const uint8_t *packet,
                                                 size_t length);

/* Native ESP32-S3 USB Serial/JTAG transport, framed with packet_stream. */
esp_err_t usb_transport_start(usb_transport_packet_callback_t callback);
esp_err_t usb_transport_send_packet(const uint8_t *packet, size_t length);

/* Zero-delimited text for monitor/debug; protocol readers safely discard it. */
esp_err_t usb_transport_send_debug_line(const char *line);
