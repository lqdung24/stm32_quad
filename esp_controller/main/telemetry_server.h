#ifndef TELEMETRY_SERVER_H
#define TELEMETRY_SERVER_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

/* Read-only WebSocket endpoint for the newest STM32 flight telemetry sample. */
esp_err_t telemetry_server_start(httpd_handle_t server);
void telemetry_server_publish(const uint8_t *packet, size_t length);

#endif
