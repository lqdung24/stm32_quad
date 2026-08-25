#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "bridge_config.h"
#include "dp_protocol.h"
#include "espnow_transport.h"
#include "status_led.h"
#include "usb_transport.h"

#define LINK_TIMEOUT_MS 300U
#define LINK_LOG_PERIOD_MS 1000U
#define LINK_MAX_ACK_LAG_PACKETS 16U

typedef struct {
    bool has_control;
    bool has_status;
    bool has_ack;
    uint32_t last_control_ms;
    uint32_t last_status_ms;
    uint32_t last_ack_ms;
    uint32_t control_packets;
    uint32_t status_packets;
    uint32_t telemetry_packets;
    uint32_t invalid_packets;
    uint16_t control_session;
    uint16_t control_sequence;
    DroneSystemStatus last_status;
} ground_link_state_t;

static const char *TAG = "ground";
static const uint8_t s_air_peer_mac[6] = DRONE_AIR_STA_MAC_BYTES;
static ground_link_state_t s_ground_link;
static portMUX_TYPE s_ground_link_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t monotonic_ms(void)
{
    return (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
}

static bool decode_control(const uint8_t *packet, size_t length,
                           DroneControlCommand *command)
{
    return DroneProtocol_DecodeControl(packet, length, command) ==
           DRONE_PROTOCOL_OK;
}

static void log_espnow_link_config(void)
{
    espnow_transport_link_info_t info;
    ESP_ERROR_CHECK(espnow_transport_get_link_info(&info));

    char line[128];
    (void)snprintf(
        line, sizeof(line),
        "ESPNOW local=%02x:%02x:%02x:%02x:%02x:%02x "
        "peer=%02x:%02x:%02x:%02x:%02x:%02x channel=%u",
        info.local_mac[0], info.local_mac[1], info.local_mac[2],
        info.local_mac[3], info.local_mac[4], info.local_mac[5],
        info.peer_mac[0], info.peer_mac[1], info.peer_mac[2],
        info.peer_mac[3], info.peer_mac[4], info.peer_mac[5], info.channel);
    ESP_LOGI(TAG, "%s", line);
    (void)usb_transport_send_debug_line(line);
}

/* Laptop Web Serial -> ESP-NOW. The DroneProtocol packet stays unmodified. */
static void on_usb_packet(const uint8_t *packet, size_t length)
{
    DroneControlCommand command;
    if (!decode_control(packet, length, &command)) {
        ESP_LOGW(TAG, "Dropped invalid control packet from USB");
        return;
    }
    portENTER_CRITICAL(&s_ground_link_lock);
    s_ground_link.has_control = true;
    s_ground_link.last_control_ms = monotonic_ms();
    s_ground_link.control_session = command.header.session_id;
    s_ground_link.control_sequence = command.header.sequence;
    ++s_ground_link.control_packets;
    portEXIT_CRITICAL(&s_ground_link_lock);
    if (espnow_transport_send_packet(packet, length) != ESP_OK) {
        ESP_LOGW(TAG, "ESP-NOW transmit failed");
    }
}

/* ESP-NOW -> laptop Web Serial: status and telemetry remain raw packets. */
static void on_espnow_packet(const uint8_t *packet, size_t length)
{
    DroneSystemStatus status;
    DroneFlightTelemetry telemetry;
    const uint32_t now_ms = monotonic_ms();

    if (DroneProtocol_DecodeStatus(packet, length, &status) ==
        DRONE_PROTOCOL_OK) {
        portENTER_CRITICAL(&s_ground_link_lock);
        s_ground_link.has_status = true;
        s_ground_link.last_status_ms = now_ms;
        s_ground_link.last_status = status;
        ++s_ground_link.status_packets;
        const uint16_t ack_lag =
            (uint16_t)(s_ground_link.control_sequence -
                       status.last_control_sequence);
        if (s_ground_link.has_control &&
            status.header.session_id == s_ground_link.control_session &&
            ack_lag <= LINK_MAX_ACK_LAG_PACKETS) {
            s_ground_link.has_ack = true;
            s_ground_link.last_ack_ms = now_ms;
        }
        portEXIT_CRITICAL(&s_ground_link_lock);
    } else if (DroneProtocol_DecodeFlightTelemetry(packet, length, &telemetry) ==
               DRONE_PROTOCOL_OK) {
        portENTER_CRITICAL(&s_ground_link_lock);
        ++s_ground_link.telemetry_packets;
        portEXIT_CRITICAL(&s_ground_link_lock);
    } else {
        portENTER_CRITICAL(&s_ground_link_lock);
        ++s_ground_link.invalid_packets;
        portEXIT_CRITICAL(&s_ground_link_lock);
    }
    (void)usb_transport_send_packet(packet, length);
}

static const char *link_text(bool online)
{
    return online ? "UP" : "DOWN";
}

static void ground_link_log_task(void *arg)
{
    (void)arg;
    ground_link_state_t previous = {0};
    espnow_transport_stats_t previous_transport = {0};

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(LINK_LOG_PERIOD_MS));

        const uint32_t now_ms = monotonic_ms();
        ground_link_state_t link;
        espnow_transport_stats_t transport;

        portENTER_CRITICAL(&s_ground_link_lock);
        link = s_ground_link;
        portEXIT_CRITICAL(&s_ground_link_lock);
        espnow_transport_get_stats(&transport);

        const bool usb_online = link.has_control &&
            (uint32_t)(now_ms - link.last_control_ms) < LINK_TIMEOUT_MS;
        const bool espnow_online = link.has_ack &&
            (uint32_t)(now_ms - link.last_ack_ms) < LINK_TIMEOUT_MS;
        const bool stm32_online = espnow_online && link.has_status &&
            (uint32_t)(now_ms - link.last_status_ms) < LINK_TIMEOUT_MS &&
            (link.last_status.error_flags & DRONE_ERROR_UART_LINK_LOST) == 0U;

        char debug_line[256];
        (void)snprintf(
            debug_line, sizeof(debug_line),
            "LINK ground usb=%s espnow=%s stm32=%s "
            "ctrl=%" PRIu32 "/s status=%" PRIu32 "/s tele=%" PRIu32
            "/s invalid=%" PRIu32 "/s tx_ok=%" PRIu32
            "/s tx_fail=%" PRIu32 "/s rx_drop=%" PRIu32
            " seq=%u ack=%u err=0x%04x",
            link_text(usb_online), link_text(espnow_online),
            link_text(stm32_online),
            link.control_packets - previous.control_packets,
            link.status_packets - previous.status_packets,
            link.telemetry_packets - previous.telemetry_packets,
            link.invalid_packets - previous.invalid_packets,
            transport.tx_delivered - previous_transport.tx_delivered,
            (transport.tx_delivery_failed -
             previous_transport.tx_delivery_failed) +
                (transport.tx_submit_failed -
                 previous_transport.tx_submit_failed),
            transport.rx_queue_dropped - previous_transport.rx_queue_dropped,
            link.control_sequence,
            link.has_status ? link.last_status.last_control_sequence : 0U,
            link.has_status ? link.last_status.error_flags : 0U);
        ESP_LOGI(TAG, "%s", debug_line);
        (void)usb_transport_send_debug_line(debug_line);

        previous = link;
        previous_transport = transport;
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = status_led_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Status LED unavailable: %s", esp_err_to_name(err));
    }

    ESP_ERROR_CHECK(espnow_transport_start(on_espnow_packet, s_air_peer_mac,
                                           DRONE_ESPNOW_CHANNEL));
    ESP_ERROR_CHECK(usb_transport_start(on_usb_packet));
    log_espnow_link_config();
    ESP_ERROR_CHECK(xTaskCreate(ground_link_log_task, "ground_link_log", 4096,
                                NULL, 5, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
    status_led_set(STATUS_LED_WAITING_FOR_CLIENT);
    ESP_LOGI(TAG, "Ground bridge ready: browser Web Serial -> ESP-NOW");
}
