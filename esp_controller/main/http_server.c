#include "http_server.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "http";

extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[] asm("_binary_index_html_end");
extern const unsigned char style_css_start[] asm("_binary_style_css_start");
extern const unsigned char style_css_end[] asm("_binary_style_css_end");
extern const unsigned char app_js_start[] asm("_binary_app_js_start");
extern const unsigned char app_js_end[] asm("_binary_app_js_end");

typedef struct {
    const unsigned char *start;
    const unsigned char *end;
    const char *content_type;
} static_asset_t;

static static_asset_t s_index_asset = {
    .start = index_html_start,
    .end = index_html_end,
    .content_type = "text/html; charset=utf-8",
};
static static_asset_t s_style_asset = {
    .start = style_css_start,
    .end = style_css_end,
    .content_type = "text/css; charset=utf-8",
};
static static_asset_t s_script_asset = {
    .start = app_js_start,
    .end = app_js_end,
    .content_type = "application/javascript; charset=utf-8",
};

static esp_err_t static_asset_handler(httpd_req_t *req)
{
    const static_asset_t *asset = req->user_ctx;

    httpd_resp_set_type(req, asset->content_type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)asset->start,
                           asset->end - asset->start);
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
        .handler = static_asset_handler,
        .user_ctx = &s_index_asset,
    };
    const httpd_uri_t style_route = {
        .uri = "/style.css",
        .method = HTTP_GET,
        .handler = static_asset_handler,
        .user_ctx = &s_style_asset,
    };
    const httpd_uri_t script_route = {
        .uri = "/app.js",
        .method = HTTP_GET,
        .handler = static_asset_handler,
        .user_ctx = &s_script_asset,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(*server, &index_route),
                        TAG, "index route registration failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(*server, &style_route),
                        TAG, "stylesheet route registration failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(*server, &script_route),
                        TAG, "script route registration failed");
    ESP_LOGI(TAG, "Static control page ready");
    return ESP_OK;
}
