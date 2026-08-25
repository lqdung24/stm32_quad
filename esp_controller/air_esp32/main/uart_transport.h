#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Air-only transport between ESP32 and the STM32 flight controller. */
typedef void (*uart_transport_packet_callback_t)(const uint8_t *packet,
                                                 size_t length);

esp_err_t uart_transport_start(uart_transport_packet_callback_t callback);
esp_err_t uart_transport_send_packet(const uint8_t *packet, size_t length);
