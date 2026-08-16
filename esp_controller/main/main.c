#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "control_bridge.h"
#include "http_server.h"
#include "status_led.h"
#include "telemetry_server.h"
#include "uart_transport.h"
#include "websocket_server.h"
#include "wifi_ap.h"

static const char *TAG = "app";

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

    ESP_ERROR_CHECK(control_bridge_init());
    ESP_ERROR_CHECK(uart_transport_start(control_bridge_on_uart_packet));
    ESP_ERROR_CHECK(wifi_ap_start());
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(http_server_start(&server));
    ESP_ERROR_CHECK(websocket_server_start(server));
    ESP_ERROR_CHECK(telemetry_server_start(server));

    ESP_LOGI(TAG, "Drone bridge ready: connect to Wi-Fi '%s'", CONFIG_DRONE_WIFI_SSID);
    ESP_LOGI(TAG, "Open http://192.168.4.1 in a browser");
}
