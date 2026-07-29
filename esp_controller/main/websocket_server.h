#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t websocket_server_start(httpd_handle_t server);
void websocket_server_send_binary(const uint8_t *packet, size_t length);
