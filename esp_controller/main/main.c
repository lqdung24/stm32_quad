#include "esp_err.h"
#include "esp_log.h"
#include <inttypes.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "dp_protocol.h"

#include "bridge_config.h"
#include "espnow_transport.h"
#include "status_led.h"
#include "uart_transport.h"
#include "usb_transport.h"

static const char *TAG = "app";

#define LINK_TIMEOUT_MS 300U
#define LINK_STATUS_PERIOD_MS 50U
#define LINK_MONITOR_TASK_STACK_BYTES 3072U
#define LINK_LOG_PERIOD_MS 1000U
#define LINK_MAX_ACK_LAG_PACKETS 16U

#if !DRONE_BRIDGE_IS_GROUND
typedef struct {
    bool has_ground_control;
    bool has_stm32_status;
    uint32_t last_ground_control_ms;
    uint32_t last_stm32_status_ms;
    DroneControlCommand last_ground_control;
    uint16_t synthetic_status_sequence;
} air_link_state_t;

static air_link_state_t s_air_link;
static portMUX_TYPE s_air_link_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t monotonic_ms(void)
{
    return (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
}
#endif

static bool decode_control(const uint8_t *packet, size_t length,
                           DroneControlCommand *command)
{
    return DroneProtocol_DecodeControl(packet, length, command) == DRONE_PROTOCOL_OK;
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
#if DRONE_BRIDGE_IS_GROUND
    (void)usb_transport_send_debug_line(line);
#endif
}

#if DRONE_BRIDGE_IS_GROUND
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

static ground_link_state_t s_ground_link;
static portMUX_TYPE s_ground_link_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t monotonic_ms(void)
{
    return (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
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

/* ESP-NOW -> laptop Web Serial: status and telemetry are raw protocol bytes. */
static void on_espnow_packet(const uint8_t *packet, size_t length)
{
    DroneSystemStatus status;
    DroneFlightTelemetry telemetry;
    const uint32_t now_ms = monotonic_ms();

    if (DroneProtocol_DecodeStatus(packet, length, &status) == DRONE_PROTOCOL_OK) {
        portENTER_CRITICAL(&s_ground_link_lock);
        s_ground_link.has_status = true;
        s_ground_link.last_status_ms = now_ms;
        s_ground_link.last_status = status;
        ++s_ground_link.status_packets;
        const uint16_t ack_lag =
            (uint16_t)(s_ground_link.control_sequence -
                       status.last_control_sequence);
        if (s_ground_link.has_control &&
            (status.header.session_id == s_ground_link.control_session) &&
            (ack_lag <= LINK_MAX_ACK_LAG_PACKETS)) {
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
            ((link.last_status.error_flags & DRONE_ERROR_UART_LINK_LOST) == 0U);

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
#else
static void update_air_led(const DroneSystemStatus *status)
{
    switch (status->state) {
    case DRONE_STATE_DISARMED: status_led_set(STATUS_LED_DISARMED); break;
    case DRONE_STATE_ARMED: status_led_set(STATUS_LED_ARMED); break;
    case DRONE_STATE_FAILSAFE:
    case DRONE_STATE_ERROR: status_led_set(STATUS_LED_FAULT); break;
    case DRONE_STATE_BOOT:
    default: status_led_set(STATUS_LED_CLIENT_CONNECTED); break;
    }
}

static void remember_ground_control(const DroneControlCommand *command)
{
    portENTER_CRITICAL(&s_air_link_lock);
    s_air_link.last_ground_control = *command;
    s_air_link.last_ground_control_ms = monotonic_ms();
    s_air_link.has_ground_control = true;
    portEXIT_CRITICAL(&s_air_link_lock);
}

/* ESP-NOW -> STM32 UART. STM32 owns all motor and failsafe decisions. */
static void on_espnow_packet(const uint8_t *packet, size_t length)
{
    DroneControlCommand command;
    if (!decode_control(packet, length, &command)) {
        ESP_LOGW(TAG, "Dropped invalid control packet from ESP-NOW");
        return;
    }
    remember_ground_control(&command);
    if (uart_transport_send_packet(packet, length) != ESP_OK) {
        ESP_LOGW(TAG, "UART transmit failed");
    }
}

/* STM32 UART -> ESP-NOW. Forward protocol packets unchanged to the laptop. */
static void on_uart_packet(const uint8_t *packet, size_t length)
{
    DroneSystemStatus status;
    if (DroneProtocol_DecodeStatus(packet, length, &status) == DRONE_PROTOCOL_OK) {
        portENTER_CRITICAL(&s_air_link_lock);
        s_air_link.last_stm32_status_ms = monotonic_ms();
        s_air_link.has_stm32_status = true;
        portEXIT_CRITICAL(&s_air_link_lock);
        update_air_led(&status);
    }
    if (espnow_transport_send_packet(packet, length) != ESP_OK) {
        ESP_LOGW(TAG, "ESP-NOW telemetry/status transmit failed");
    }
}

static void send_stm32_link_lost_status(const DroneControlCommand *last_control,
                                        uint16_t status_sequence,
                                        uint32_t now_ms)
{
    DroneSystemStatus status = {
        .header = {
            .sequence = status_sequence,
            .session_id = last_control->header.session_id,
            .sender_time_ms = now_ms,
        },
        .last_control_sequence = last_control->header.sequence,
        .requested_throttle = last_control->throttle,
        .applied_throttle = 0U,
        .pwm_pulse_us = {1000U, 1000U, 1000U, 1000U},
        .state = DRONE_STATE_FAILSAFE,
        .error_flags = DRONE_ERROR_UART_LINK_LOST | DRONE_ERROR_FAILSAFE_ACTIVE,
        .uart_rx_rate = 0U,
    };
    uint8_t packet[DRONE_STATUS_PACKET_SIZE];

    if (DroneProtocol_EncodeStatus(&status, packet) != DRONE_PROTOCOL_OK) {
        ESP_LOGE(TAG, "Cannot encode STM32 link-lost status");
        return;
    }
    if (espnow_transport_send_packet(packet, sizeof(packet)) != ESP_OK) {
        ESP_LOGW(TAG, "ESP-NOW STM32 link-lost status transmit failed");
    }
}

static void air_link_monitor_task(void *arg)
{
    (void)arg;

    while (true) {
        const uint32_t now_ms = monotonic_ms();
        DroneControlCommand last_control;
        uint16_t status_sequence = 0U;
        bool ground_online;
        bool stm32_online;

        portENTER_CRITICAL(&s_air_link_lock);
        ground_online = s_air_link.has_ground_control &&
                        (uint32_t)(now_ms - s_air_link.last_ground_control_ms) <
                            LINK_TIMEOUT_MS;
        stm32_online = s_air_link.has_stm32_status &&
                       (uint32_t)(now_ms - s_air_link.last_stm32_status_ms) <
                           LINK_TIMEOUT_MS;
        if (ground_online && !stm32_online) {
            last_control = s_air_link.last_ground_control;
            status_sequence = s_air_link.synthetic_status_sequence++;
        }
        portEXIT_CRITICAL(&s_air_link_lock);

        if (ground_online && !stm32_online) {
            status_led_set(STATUS_LED_FAULT);
            send_stm32_link_lost_status(&last_control, status_sequence, now_ms);
        }
        vTaskDelay(pdMS_TO_TICKS(LINK_STATUS_PERIOD_MS));
    }
}
#endif

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = status_led_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Status LED unavailable: %s", esp_err_to_name(err));
    }

#if DRONE_BRIDGE_IS_GROUND
    ESP_ERROR_CHECK(espnow_transport_start(on_espnow_packet));
    ESP_ERROR_CHECK(usb_transport_start(on_usb_packet));
    log_espnow_link_config();
    ESP_ERROR_CHECK(xTaskCreate(ground_link_log_task, "ground_link_log", 4096,
                                NULL, 5, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
    status_led_set(STATUS_LED_WAITING_FOR_CLIENT);
    ESP_LOGI(TAG, "Ground bridge ready: browser Web Serial -> ESP-NOW");
#else
    ESP_ERROR_CHECK(uart_transport_start(on_uart_packet));
    ESP_ERROR_CHECK(espnow_transport_start(on_espnow_packet));
    log_espnow_link_config();
    ESP_ERROR_CHECK(xTaskCreate(air_link_monitor_task, "air_link_monitor",
                                LINK_MONITOR_TASK_STACK_BYTES, NULL, 9, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
    status_led_set(STATUS_LED_WAITING_FOR_CLIENT);
    ESP_LOGI(TAG, "Air bridge ready: ESP-NOW -> STM32 UART");
#endif
}
