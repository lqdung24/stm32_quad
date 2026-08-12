#include "control_bridge.h"

#include <string.h>

#include "dp_protocol.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uart_transport.h"
#include "websocket_server.h"

static const char *TAG = "control_bridge";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_phone_fd = -1;
static int64_t s_last_phone_packet_us;
static uint16_t s_session_id;
static uint16_t s_last_phone_sequence;
static uint32_t s_phone_packet_count;
static bool s_phone_packet_seen;
static bool s_phone_timed_out;

static void send_timeout_stop(uint16_t session_id, uint16_t last_sequence)
{
    DroneControlCommand stop = {
        .header = {
            .sequence = (uint16_t)(last_sequence + 1U),
            .session_id = session_id,
            .flags = DRONE_CONTROL_FLAG_EMERGENCY_STOP,
            .sender_time_ms = (uint32_t)(esp_timer_get_time() / 1000),
        },
        .throttle = 0,
    };
    uint8_t packet[DRONE_CONTROL_PACKET_SIZE];
    if (DroneProtocol_EncodeControl(&stop, packet) == DRONE_PROTOCOL_OK)
    {
        (void)uart_transport_send_packet(packet, sizeof(packet));
        ESP_LOGE(TAG, "PHONE FAILSAFE: no valid packet for %d ms",
                 CONFIG_DRONE_PHONE_TIMEOUT_MS);
    }
}

static void bridge_watchdog_task(void *arg)
{
    (void)arg;
    while (true)
    {
        bool must_stop = false;
        uint16_t session_id = 0;
        uint16_t last_sequence = 0;
        const int64_t now = esp_timer_get_time();

        portENTER_CRITICAL(&s_lock);
        if (s_phone_packet_seen && !s_phone_timed_out &&
            now - s_last_phone_packet_us >
                (int64_t)CONFIG_DRONE_PHONE_TIMEOUT_MS * 1000)
        {
            s_phone_timed_out = true;
            must_stop = true;
            session_id = s_session_id;
            last_sequence = s_last_phone_sequence;
        }
        portEXIT_CRITICAL(&s_lock);

        if (must_stop)
        {
            send_timeout_stop(session_id, last_sequence);
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

esp_err_t control_bridge_init(void)
{
    const BaseType_t result =
        xTaskCreate(bridge_watchdog_task, "phone_watchdog",
                    3072, NULL, 11, NULL);
    return result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void control_bridge_phone_connected(int socket_fd)
{
    portENTER_CRITICAL(&s_lock);
    s_phone_fd = socket_fd;
    s_phone_packet_count = 0;
    s_phone_packet_seen = false;
    s_phone_timed_out = false;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "WebSocket connected, fd=%d; waiting for new session",
             socket_fd);
}

void control_bridge_phone_disconnected(int socket_fd)
{
    bool must_stop = false;
    uint16_t session_id = 0;
    uint16_t last_sequence = 0;
    portENTER_CRITICAL(&s_lock);
    if (s_phone_fd == socket_fd)
    {
        s_phone_fd = -1;
        if (s_phone_packet_seen && !s_phone_timed_out)
        {
            s_phone_timed_out = true;
            must_stop = true;
            session_id = s_session_id;
            last_sequence = s_last_phone_sequence;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    if (must_stop)
    {
        send_timeout_stop(session_id, last_sequence);
    }
    ESP_LOGW(TAG, "WebSocket disconnected, fd=%d", socket_fd);
}

esp_err_t control_bridge_on_phone_packet(int socket_fd,
                                         const uint8_t *packet,
                                         size_t length)
{
    DroneControlCommand command;
    const DroneProtocolResult result =
        DroneProtocol_DecodeControl(packet, length, &command);
    if (result != DRONE_PROTOCOL_OK)
    {
        ESP_LOGW(TAG, "Invalid control packet, protocol error=%d", result);
        return ESP_ERR_INVALID_CRC;
    }

    portENTER_CRITICAL(&s_lock);
    if (socket_fd != s_phone_fd)
    {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_phone_packet_seen &&
        command.header.session_id == s_session_id &&
        !DroneProtocol_IsSequenceNewer(command.header.sequence,
                                       s_last_phone_sequence))
    {
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGW(TAG, "Duplicate or old control sequence=%u",
                 command.header.sequence);
        return ESP_ERR_INVALID_STATE;
    }
    s_session_id = command.header.session_id;
    s_last_phone_sequence = command.header.sequence;
    s_last_phone_packet_us = esp_timer_get_time();
    const uint32_t packet_count = ++s_phone_packet_count;
    s_phone_packet_seen = true;
    s_phone_timed_out = false;
    portEXIT_CRITICAL(&s_lock);

    if ((packet_count % 40U) == 0U)
    {
        ESP_LOGI(TAG,
                 "PHONE RX: packets=%lu session=%u sequence=%u "
                 "throttle=%u motor=%u flags=0x%04x",
                 (unsigned long)packet_count,
                 command.header.session_id,
                 command.header.sequence,
                 command.throttle,
                 command.aux1,
                 command.header.flags);
    }

    if ((packet_count == 1U) || ((packet_count % 10U) == 0U))
    {
        websocket_server_send_text("ESP_ALIVE");
    }

    return uart_transport_send_packet(packet, length);
}

void control_bridge_on_uart_packet(const uint8_t *packet, size_t length)
{
    DroneSystemStatus status;
    if (DroneProtocol_DecodeStatus(packet, length, &status) !=
        DRONE_PROTOCOL_OK)
    {
        ESP_LOGW(TAG, "Invalid SYSTEM_STATUS from STM32");
        return;
    }

    ESP_LOGI(TAG,
             "STM32 STATUS: seq=%u session=%u throttle=%u/%u pwm=%u,%u,%u,%u state=%u flags=0x%04x rx_rate=%u",
             status.header.sequence,
             status.header.session_id,
             status.requested_throttle,
             status.applied_throttle,
             status.pwm_pulse_us[0],
             status.pwm_pulse_us[1],
             status.pwm_pulse_us[2],
             status.pwm_pulse_us[3],
             status.state,
             status.error_flags,
             status.uart_rx_rate);

    websocket_server_send_binary(packet, length);
}
