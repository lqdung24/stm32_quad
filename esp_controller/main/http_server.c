#include "http_server.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "http";

extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[] asm("_binary_index_html_end");

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

esp_err_t http_server_start(httpd_handle_t *server)
{
    ESP_RETURN_ON_FALSE(server != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "server output is null");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.lru_purge_enable = true;

    ESP_RETURN_ON_ERROR(httpd_start(server, &config), TAG,
                        "HTTP server start failed");

    const httpd_uri_t index_route = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(*server, &index_route),
                        TAG, "index route registration failed");
    ESP_LOGI(TAG, "Static control page ready");
    return ESP_OK;
}
