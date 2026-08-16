#include "control_bridge.h"

#include <string.h>

#include "dp_protocol.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uart_transport.h"
#include "websocket_server.h"
#include "status_led.h"
#include "telemetry_server.h"

#define DIAGNOSTIC_LOG_PERIOD_US 1000000LL
#define WATCHDOG_PERIOD_MS 25U
#define STM32_STATUS_LOG_PERIOD_US 1000000LL

static const char *TAG = "control_bridge";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_phone_fd = -1;
static int64_t s_last_phone_packet_us;
static int64_t s_last_status_log_us;
static uint16_t s_session_id;
static uint16_t s_last_phone_sequence;
static uint32_t s_phone_packet_count;
static int64_t s_rx_log_window_start_us;
static int64_t s_rx_gap_sum_us;
static int64_t s_rx_gap_min_us;
static int64_t s_rx_gap_max_us;
static uint32_t s_rx_gap_sample_count;
static bool s_phone_packet_seen;
static bool s_phone_timed_out;
/* A phone-link stop is transient; do not display it as a permanent STM fault. */
static bool s_phone_link_stop_latched;

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
        ESP_LOGW(TAG,
                 "PHONE LINK TIMEOUT: emergency stop sent after %d ms; returning to AP ready",
                 CONFIG_DRONE_PHONE_TIMEOUT_MS);
    }
}

static void bridge_watchdog_task(void *arg)
{
    (void)arg;
    int64_t previous_loop_us = 0;
    int64_t report_start_us = esp_timer_get_time();
    int64_t period_sum_us = 0;
    int64_t period_min_us = 0;
    int64_t period_max_us = 0;
    int64_t latest_period_us = 0;
    uint32_t period_sample_count = 0;

    while (true)
    {
        bool must_stop = false;
        int phone_fd = -1;
        uint16_t session_id = 0;
        uint16_t last_sequence = 0;
        const int64_t now = esp_timer_get_time();

        if (previous_loop_us != 0)
        {
            const int64_t period_us = now - previous_loop_us;
            latest_period_us = period_us;
            period_sum_us += period_us;
            if ((period_sample_count == 0U) || (period_us < period_min_us))
            {
                period_min_us = period_us;
            }
            if (period_us > period_max_us)
            {
                period_max_us = period_us;
            }
            ++period_sample_count;
        }
        previous_loop_us = now;

        portENTER_CRITICAL(&s_lock);
        if (s_phone_packet_seen && !s_phone_timed_out &&
            now - s_last_phone_packet_us >
                (int64_t)CONFIG_DRONE_PHONE_TIMEOUT_MS * 1000)
        {
            s_phone_timed_out = true;
            s_phone_link_stop_latched = true;
            must_stop = true;
            phone_fd = s_phone_fd;
            session_id = s_session_id;
            last_sequence = s_last_phone_sequence;
        }
        portEXIT_CRITICAL(&s_lock);

        if (must_stop)
        {
            send_timeout_stop(session_id, last_sequence);
            /*
             * The command stop is a failsafe action, not a permanent ESP
             * state. Close a non-responsive WebSocket and return to AP ready.
             */
            websocket_server_disconnect_client(phone_fd);
        }

        if (((now - report_start_us) >= DIAGNOSTIC_LOG_PERIOD_US) &&
            (period_sample_count > 0U))
        {
            const int64_t average_period_us =
                period_sum_us / period_sample_count;
            ESP_LOGI(TAG,
                     "[WD] actual_period_us last=%lld avg=%lld min=%lld max=%lld expected_ms=%u samples=%lu",
                     (long long)latest_period_us,
                     (long long)average_period_us,
                     (long long)period_min_us,
                     (long long)period_max_us,
                     WATCHDOG_PERIOD_MS,
                     (unsigned long)period_sample_count);
            report_start_us = now;
            period_sum_us = 0;
            period_min_us = 0;
            period_max_us = 0;
            period_sample_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_PERIOD_MS));
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
    s_rx_log_window_start_us = 0;
    s_rx_gap_sum_us = 0;
    s_rx_gap_min_us = 0;
    s_rx_gap_max_us = 0;
    s_rx_gap_sample_count = 0;
    s_phone_packet_seen = false;
    s_phone_timed_out = false;
    portEXIT_CRITICAL(&s_lock);
    status_led_set(STATUS_LED_CLIENT_CONNECTED);
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
            s_phone_link_stop_latched = true;
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
    status_led_set(STATUS_LED_WAITING_FOR_CLIENT);
    ESP_LOGW(TAG, "WebSocket disconnected, fd=%d", socket_fd);
}

esp_err_t control_bridge_on_phone_packet(int socket_fd,
                                         const uint8_t *packet,
                                         size_t length)
{
    DroneControlCommand command;
    const int64_t now_us = esp_timer_get_time();
    int64_t latest_gap_us = 0;
    int64_t gap_sum_us = 0;
    int64_t average_gap_us = 0;
    int64_t minimum_gap_us = 0;
    int64_t maximum_gap_us = 0;
    uint32_t gap_sample_count = 0;
    bool log_rx_diagnostics = false;
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
    if (s_phone_packet_seen)
    {
        latest_gap_us = now_us - s_last_phone_packet_us;
        s_rx_gap_sum_us += latest_gap_us;
        if ((s_rx_gap_sample_count == 0U) ||
            (latest_gap_us < s_rx_gap_min_us))
        {
            s_rx_gap_min_us = latest_gap_us;
        }
        if (latest_gap_us > s_rx_gap_max_us)
        {
            s_rx_gap_max_us = latest_gap_us;
        }
        ++s_rx_gap_sample_count;
    }
    if (s_rx_log_window_start_us == 0)
    {
        s_rx_log_window_start_us = now_us;
    }
    s_session_id = command.header.session_id;
    s_last_phone_sequence = command.header.sequence;
    s_last_phone_packet_us = now_us;
    const uint32_t packet_count = ++s_phone_packet_count;
    s_phone_packet_seen = true;
    s_phone_timed_out = false;
    s_phone_link_stop_latched = false;
    if ((now_us - s_rx_log_window_start_us) >= DIAGNOSTIC_LOG_PERIOD_US)
    {
        gap_sample_count = s_rx_gap_sample_count;
        if (gap_sample_count > 0U)
        {
            gap_sum_us = s_rx_gap_sum_us;
            minimum_gap_us = s_rx_gap_min_us;
            maximum_gap_us = s_rx_gap_max_us;
        }
        log_rx_diagnostics = true;
        s_rx_log_window_start_us = now_us;
        s_rx_gap_sum_us = 0;
        s_rx_gap_min_us = 0;
        s_rx_gap_max_us = 0;
        s_rx_gap_sample_count = 0;
    }
    portEXIT_CRITICAL(&s_lock);

    if (log_rx_diagnostics)
    {
        if (gap_sample_count > 0U)
        {
            average_gap_us = gap_sum_us / gap_sample_count;
        }
        ESP_LOGI(TAG,
                 "[RX] seq=%u session=%u gap_us=%lld avg=%lld min=%lld max=%lld samples=%lu packets=%lu",
                 command.header.sequence,
                 command.header.session_id,
                 (long long)latest_gap_us,
                 (long long)average_gap_us,
                 (long long)minimum_gap_us,
                 (long long)maximum_gap_us,
                 (unsigned long)gap_sample_count,
                 (unsigned long)packet_count);
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
    DroneFlightTelemetry telemetry;
    const int64_t now_us = esp_timer_get_time();

    if ((packet != NULL) && (length >= 4U) &&
        (packet[3] == DRONE_PACKET_FLIGHT_TELEMETRY))
    {
        if (DroneProtocol_DecodeFlightTelemetry(packet, length, &telemetry) !=
            DRONE_PROTOCOL_OK)
        {
            ESP_LOGW(TAG, "Invalid FLIGHT_TELEMETRY from STM32");
            return;
        }
        telemetry_server_publish(packet, length);
        return;
    }

    if (DroneProtocol_DecodeStatus(packet, length, &status) !=
        DRONE_PROTOCOL_OK)
    {
        ESP_LOGW(TAG, "Invalid SYSTEM_STATUS from STM32");
        return;
    }

    if ((s_last_status_log_us == 0) ||
        ((now_us - s_last_status_log_us) >= STM32_STATUS_LOG_PERIOD_US))
    {
        s_last_status_log_us = now_us;
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
    }

    bool phone_connected;
    bool phone_link_stop_latched;
    portENTER_CRITICAL(&s_lock);
    phone_connected = s_phone_fd >= 0;
    phone_link_stop_latched = s_phone_link_stop_latched;
    portEXIT_CRITICAL(&s_lock);

    /*
     * No phone is an AP-ready state. A failsafe caused by that phone link is
     * already handled by the emergency stop, so it must not hold the LED red.
     * A real STM32 ERROR still takes precedence and remains visible.
     */
    if (!phone_connected) {
        if (status.state == DRONE_STATE_ERROR) {
            status_led_set(STATUS_LED_FAULT);
        } else {
            status_led_set(STATUS_LED_WAITING_FOR_CLIENT);
        }
        websocket_server_send_binary(packet, length);
        return;
    }

    switch (status.state)
    {
    case DRONE_STATE_DISARMED:
        status_led_set(STATUS_LED_DISARMED);
        break;
    case DRONE_STATE_ARMED:
        status_led_set(STATUS_LED_ARMED);
        break;
    case DRONE_STATE_BOOT:
        /* Keep the local Wi-Fi/WebSocket indication while STM32 starts. */
        break;
    case DRONE_STATE_FAILSAFE:
        if (phone_link_stop_latched) {
            status_led_set(STATUS_LED_CLIENT_CONNECTED);
            break;
        }
        status_led_set(STATUS_LED_FAULT);
        break;
    case DRONE_STATE_ERROR:
    default:
        status_led_set(STATUS_LED_FAULT);
        break;
    }

    websocket_server_send_binary(packet, length);
}
