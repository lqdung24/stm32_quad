#include "websocket_server.h"

#include <stdlib.h>
#include <string.h>

#include "drone_protocol.h"
#include "esp_check.h"
#include "esp_log.h"

#include "control_bridge.h"

typedef struct {
    int fd;
    size_t length;
    uint8_t payload[DRONE_PROTOCOL_MAX_PACKET_SIZE];
} ws_send_job_t;

static const char *TAG = "websocket";
static httpd_handle_t s_server;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_client_fd = -1;
static uint8_t s_busy_message[] = "CONTROLLER_BUSY";

static bool release_active_client(int fd)
{
    bool released = false;

    portENTER_CRITICAL(&s_lock);
    if (s_client_fd == fd) {
        s_client_fd = -1;
        released = true;
    }
    portEXIT_CRITICAL(&s_lock);

    if (released) {
        control_bridge_phone_disconnected(fd);
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

    ESP_LOGW(TAG,
             "Rejected WebSocket fd=%d; controller fd=%d already active",
             fd, active_fd);
    const esp_err_t send_result = httpd_ws_send_frame(req, &frame);
    (void)httpd_sess_trigger_close(s_server, fd);
    return send_result;
}

static void send_job(void *context)
{
    ws_send_job_t *job = context;
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = job->payload,
        .len = job->length,
    };
    if (httpd_ws_get_fd_info(s_server, job->fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
        (void)httpd_ws_send_frame_async(s_server, job->fd, &frame);
    }
    free(job);
}

static esp_err_t websocket_handler(httpd_req_t *req)
{
    const int fd = httpd_req_to_sockfd(req);
    if (req->method == HTTP_GET) {
        bool accepted = false;
        int active_fd;

        portENTER_CRITICAL(&s_lock);
        active_fd = s_client_fd;
        if (active_fd < 0) {
            s_client_fd = fd;
            active_fd = fd;
            accepted = true;
        } else if (active_fd == fd) {
            accepted = true;
        }
        portEXIT_CRITICAL(&s_lock);

        if (!accepted) {
            return reject_additional_client(req, fd, active_fd);
        }
        control_bridge_phone_connected(fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        (void)release_active_client(fd);
        return err;
    }
    if (frame.len > DRONE_PROTOCOL_MAX_PACKET_SIZE) {
        ESP_LOGW(TAG, "Oversized WebSocket frame: %u", (unsigned)frame.len);
        (void)release_active_client(fd);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t payload[DRONE_PROTOCOL_MAX_PACKET_SIZE];
    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK) {
        (void)release_active_client(fd);
        return err;
    }
    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        (void)release_active_client(fd);
        return ESP_OK;
    }
    if (frame.type != HTTPD_WS_TYPE_BINARY) {
        ESP_LOGW(TAG, "Only binary control frames are accepted");
        return ESP_OK;
    }
    err = control_bridge_on_phone_packet(fd, payload, frame.len);
    if (err != ESP_OK) {
        /*
         * Ignore a bad/stale packet without closing the active WebSocket.
         * The phone watchdog remains responsible for stopping stale control.
         */
        return ESP_OK;
    }
    return ESP_OK;
}

esp_err_t websocket_server_start(httpd_handle_t server)
{
    ESP_RETURN_ON_FALSE(server != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "HTTP server is null");
    s_server = server;
    const httpd_uri_t websocket_route = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .is_websocket = true,
        .handle_ws_control_frames = true,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &websocket_route),
                        TAG, "WebSocket route registration failed");
    ESP_LOGI(TAG, "Binary WebSocket endpoint ready at /ws");
    return ESP_OK;
}

void websocket_server_send_binary(const uint8_t *packet, size_t length)
{
    if (packet == NULL || length == 0 ||
        length > DRONE_PROTOCOL_MAX_PACKET_SIZE || s_server == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    const int fd = s_client_fd;
    portEXIT_CRITICAL(&s_lock);
    if (fd < 0) {
        return;
    }

    ws_send_job_t *job = malloc(sizeof(*job));
    if (job == NULL) {
        return;
    }
    job->fd = fd;
    job->length = length;
    memcpy(job->payload, packet, length);
    if (httpd_queue_work(s_server, send_job, job) != ESP_OK) {
        free(job);
    }
}
