#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t control_bridge_init(void);
void control_bridge_phone_connected(int socket_fd);
void control_bridge_phone_disconnected(int socket_fd);
esp_err_t control_bridge_on_phone_packet(int socket_fd,
                                         const uint8_t *packet,
                                         size_t length);
void control_bridge_on_uart_packet(const uint8_t *packet, size_t length);
