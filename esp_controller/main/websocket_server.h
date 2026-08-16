#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t websocket_server_start(httpd_handle_t server);
void websocket_server_send_binary(const uint8_t *packet, size_t length);
void websocket_server_send_text(const char *text);

/* Release a stale controller connection so a new phone can connect. */
void websocket_server_disconnect_client(int socket_fd);
