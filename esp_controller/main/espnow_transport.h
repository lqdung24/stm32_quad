#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef void (*espnow_transport_packet_callback_t)(const uint8_t *packet,
                                                    size_t length);

typedef struct {
    uint32_t tx_submitted;
    uint32_t tx_submit_failed;
    uint32_t tx_delivered;
    uint32_t tx_delivery_failed;
    uint32_t rx_enqueued;
    uint32_t rx_queue_dropped;
    uint32_t rx_rejected;
} espnow_transport_stats_t;

typedef struct {
    uint8_t local_mac[6];
    uint8_t peer_mac[6];
    uint8_t channel;
} espnow_transport_link_info_t;

/* ESP-NOW carries a complete, unmodified DroneProtocol packet per message. */
esp_err_t espnow_transport_start(espnow_transport_packet_callback_t callback);
esp_err_t espnow_transport_send_packet(const uint8_t *packet, size_t length);
void espnow_transport_get_stats(espnow_transport_stats_t *stats);
esp_err_t espnow_transport_get_link_info(espnow_transport_link_info_t *info);
