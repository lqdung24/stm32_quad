#include "telemetry_server.h"

#include <string.h>

#include "dp_protocol.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TELEMETRY_SEND_PERIOD_MS 20U

static const char *TAG = "telemetry_ws";
static httpd_handle_t s_server;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_client_fd = -1;
static uint8_t s_latest_packet[DRONE_PROTOCOL_MAX_PACKET_SIZE];
static size_t s_latest_length;
static uint32_t s_generation;
static uint32_t s_queued_generation;
static bool s_send_pending;
static uint8_t s_busy_message[] = "TELEMETRY_BUSY";

static bool release_client(int fd)
{
    bool released = false;

    portENTER_CRITICAL(&s_lock);
    if (s_client_fd == fd)
    {
        s_client_fd = -1;
        released = true;
    }
    portEXIT_CRITICAL(&s_lock);

    if (released)
    {
        ESP_LOGI(TAG, "Telemetry client disconnected, fd=%d", fd);
    }
    return released;
}

static esp_err_t reject_additional_client(httpd_req_t *req, int fd,
                                          int active_fd)
{
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = s_busy_message,
        .len = sizeof(s_busy_message) - 1U,
    };

    ESP_LOGW(TAG, "Rejected telemetry fd=%d; active fd=%d", fd, active_fd);
    const esp_err_t result = httpd_ws_send_frame(req, &frame);
    (void)httpd_sess_trigger_close(s_server, fd);
    return result;
}

static void telemetry_send_job(void *context)
{
    uint8_t packet[DRONE_PROTOCOL_MAX_PACKET_SIZE];
    size_t length = 0U;
    int fd = -1;
    esp_err_t result = ESP_OK;

    (void)context;
    portENTER_CRITICAL(&s_lock);
    fd = s_client_fd;
    length = s_latest_length;
    if (length > 0U)
    {
        memcpy(packet, s_latest_packet, length);
    }
    portEXIT_CRITICAL(&s_lock);

    if ((fd >= 0) && (length > 0U) &&
        (httpd_ws_get_fd_info(s_server, fd) == HTTPD_WS_CLIENT_WEBSOCKET))
    {
        httpd_ws_frame_t frame = {
            .type = HTTPD_WS_TYPE_BINARY,
            .payload = packet,
            .len = length,
        };
        result = httpd_ws_send_frame_async(s_server, fd, &frame);
        if (result != ESP_OK)
        {
            ESP_LOGW(TAG, "Telemetry send failed fd=%d: %s", fd,
                     esp_err_to_name(result));
            (void)release_client(fd);
        }
    }

    portENTER_CRITICAL(&s_lock);
    s_send_pending = false;
    portEXIT_CRITICAL(&s_lock);
}

static void telemetry_sender_task(void *arg)
{
    TickType_t previous_wake = xTaskGetTickCount();

    (void)arg;
    while (true)
    {
        bool queue_work = false;

        portENTER_CRITICAL(&s_lock);
        if ((s_client_fd >= 0) && (s_latest_length > 0U) &&
            !s_send_pending && (s_generation != s_queued_generation))
        {
            s_send_pending = true;
            s_queued_generation = s_generation;
            queue_work = true;
        }
        portEXIT_CRITICAL(&s_lock);

        if (queue_work &&
            (httpd_queue_work(s_server, telemetry_send_job, NULL) != ESP_OK))
        {
            ESP_LOGW(TAG, "Telemetry work queue full; keeping newest sample");
            portENTER_CRITICAL(&s_lock);
            s_send_pending = false;
            s_queued_generation = s_generation - 1U;
            portEXIT_CRITICAL(&s_lock);
        }
        vTaskDelayUntil(&previous_wake, pdMS_TO_TICKS(TELEMETRY_SEND_PERIOD_MS));
    }
}

static esp_err_t telemetry_handler(httpd_req_t *req)
{
    const int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET)
    {
        bool accepted = false;
        int active_fd;

        portENTER_CRITICAL(&s_lock);
        active_fd = s_client_fd;
        if ((active_fd < 0) || (active_fd == fd))
        {
            s_client_fd = fd;
            s_queued_generation = s_generation - 1U;
            active_fd = fd;
            accepted = true;
        }
        portEXIT_CRITICAL(&s_lock);

        if (!accepted)
        {
            return reject_additional_client(req, fd, active_fd);
        }
        ESP_LOGI(TAG, "Telemetry client connected, fd=%d", fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0U);
    if (err != ESP_OK)
    {
        (void)release_client(fd);
        return err;
    }
    if (frame.len > 0U)
    {
        uint8_t ignored[DRONE_PROTOCOL_MAX_PACKET_SIZE];
        if (frame.len > sizeof(ignored))
        {
            (void)release_client(fd);
            return ESP_ERR_INVALID_SIZE;
        }
        frame.payload = ignored;
        err = httpd_ws_recv_frame(req, &frame, frame.len);
        if (err != ESP_OK)
        {
            (void)release_client(fd);
            return err;
        }
    }
    if (frame.type == HTTPD_WS_TYPE_CLOSE)
    {
        (void)release_client(fd);
    }
    return ESP_OK;
}

esp_err_t telemetry_server_start(httpd_handle_t server)
{
    const httpd_uri_t telemetry_route = {
        .uri = "/telemetry",
        .method = HTTP_GET,
        .handler = telemetry_handler,
        .is_websocket = true,
        .handle_ws_control_frames = true,
    };
    BaseType_t task_result;

    ESP_RETURN_ON_FALSE(server != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "HTTP server is null");
    s_server = server;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &telemetry_route),
                        TAG, "telemetry route registration failed");
    task_result = xTaskCreate(telemetry_sender_task, "telemetry_send",
                              3072, NULL, 4, NULL);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "telemetry sender task creation failed");
    ESP_LOGI(TAG, "Flight telemetry WebSocket ready at /telemetry (50 Hz)");
    return ESP_OK;
}

void telemetry_server_publish(const uint8_t *packet, size_t length)
{
    if ((packet == NULL) || (length == 0U) ||
        (length > DRONE_PROTOCOL_MAX_PACKET_SIZE))
    {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    memcpy(s_latest_packet, packet, length);
    s_latest_length = length;
    ++s_generation;
    portEXIT_CRITICAL(&s_lock);
}
