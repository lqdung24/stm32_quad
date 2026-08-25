#include "espnow_transport.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "dp_protocol.h"

#define ESPNOW_RX_QUEUE_LENGTH 16U

typedef struct {
    uint8_t data[DRONE_PROTOCOL_MAX_PACKET_SIZE];
    uint8_t length;
} espnow_packet_t;

static const char *TAG = "espnow_transport";
static QueueHandle_t s_rx_queue;
static espnow_transport_packet_callback_t s_callback;
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN];
static uint8_t s_local_mac[ESP_NOW_ETH_ALEN];
static uint8_t s_channel;
static bool s_started;
static espnow_transport_stats_t s_stats;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;

static void increment_stat(uint32_t *counter)
{
    portENTER_CRITICAL(&s_stats_lock);
    ++(*counter);
    portEXIT_CRITICAL(&s_stats_lock);
}

static void espnow_receive_callback(const esp_now_recv_info_t *info,
                                    const uint8_t *data, int data_length)
{
    if ((data == NULL) || (data_length <= 0) ||
        (data_length > DRONE_PROTOCOL_MAX_PACKET_SIZE) || (s_rx_queue == NULL) ||
        (info == NULL) ||
        (memcmp(info->src_addr, s_peer_mac, ESP_NOW_ETH_ALEN) != 0)) {
        increment_stat(&s_stats.rx_rejected);
        return;
    }
    espnow_packet_t packet = {.length = (uint8_t)data_length};
    memcpy(packet.data, data, (size_t)data_length);
    if (xQueueSend(s_rx_queue, &packet, 0) == pdTRUE) {
        increment_stat(&s_stats.rx_enqueued);
    } else {
        increment_stat(&s_stats.rx_queue_dropped);
    }
}

static void espnow_send_callback(const esp_now_send_info_t *info,
                                 esp_now_send_status_t status)
{
    (void)info;
    if (status == ESP_NOW_SEND_SUCCESS) {
        increment_stat(&s_stats.tx_delivered);
    } else {
        increment_stat(&s_stats.tx_delivery_failed);
    }
}

static void espnow_rx_task(void *arg)
{
    (void)arg;
    espnow_packet_t packet;
    while (xQueueReceive(s_rx_queue, &packet, portMAX_DELAY) == pdTRUE) {
        if (s_callback != NULL) {
            s_callback(packet.data, packet.length);
        }
    }
}

esp_err_t espnow_transport_start(espnow_transport_packet_callback_t callback,
                                 const uint8_t peer_mac[ESP_NOW_ETH_ALEN],
                                 uint8_t channel)
{
    ESP_RETURN_ON_FALSE(callback != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "packet callback is null");
    ESP_RETURN_ON_FALSE(peer_mac != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "peer MAC is null");
    ESP_RETURN_ON_FALSE(channel >= 1U && channel <= 13U, ESP_ERR_INVALID_ARG,
                        TAG, "channel must be from 1 to 13");
    memcpy(s_peer_mac, peer_mac, sizeof(s_peer_mac));
    s_channel = channel;

    const esp_err_t event_loop_result = esp_event_loop_create_default();
    ESP_RETURN_ON_FALSE((event_loop_result == ESP_OK) ||
                            (event_loop_result == ESP_ERR_INVALID_STATE),
                        event_loop_result, TAG,
                        "default event loop setup failed");

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_config), TAG, "Wi-Fi init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG,
                        "Wi-Fi storage setup failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG,
                        "Wi-Fi STA mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE),
                        TAG, "Wi-Fi channel setup failed");
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "ESP-NOW init failed");

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    peer.channel = s_channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_RETURN_ON_ERROR(esp_now_add_peer(&peer), TAG, "ESP-NOW peer setup failed");

    s_callback = callback;
    s_rx_queue = xQueueCreate(ESPNOW_RX_QUEUE_LENGTH, sizeof(espnow_packet_t));
    ESP_RETURN_ON_FALSE(s_rx_queue != NULL, ESP_ERR_NO_MEM, TAG,
                        "cannot create receive queue");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(espnow_receive_callback), TAG,
                        "ESP-NOW receive callback setup failed");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(espnow_send_callback), TAG,
                        "ESP-NOW send callback setup failed");
    ESP_RETURN_ON_FALSE(xTaskCreate(espnow_rx_task, "espnow_rx", 4096, NULL,
                                    12, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "cannot start receive task");

    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, s_local_mac), TAG,
                        "cannot read local STA MAC");
    s_started = true;
    return ESP_OK;
}

esp_err_t espnow_transport_send_packet(const uint8_t *packet, size_t length)
{
    if ((packet == NULL) || (length == 0U) ||
        (length > DRONE_PROTOCOL_MAX_PACKET_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t result = esp_now_send(s_peer_mac, packet, length);
    if (result == ESP_OK) {
        increment_stat(&s_stats.tx_submitted);
    } else {
        increment_stat(&s_stats.tx_submit_failed);
    }
    return result;
}

void espnow_transport_get_stats(espnow_transport_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_stats_lock);
    *stats = s_stats;
    portEXIT_CRITICAL(&s_stats_lock);
}

esp_err_t espnow_transport_get_link_info(espnow_transport_link_info_t *info)
{
    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    memcpy(info->local_mac, s_local_mac, sizeof(info->local_mac));
    memcpy(info->peer_mac, s_peer_mac, sizeof(info->peer_mac));
    info->channel = s_channel;
    return ESP_OK;
}
