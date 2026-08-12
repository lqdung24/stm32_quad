#include "wifi_ap.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"

static const char *TAG = "wifi_ap";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event = event_data;
        ESP_LOGI(TAG, "Phone connected, aid=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event = event_data;
        ESP_LOGW(TAG, "Client disconnected, aid=%d", event->aid);
    }
}

esp_err_t wifi_ap_start(void)
{
    if (strlen(CONFIG_DRONE_WIFI_PASSWORD) < 8) {
        ESP_LOGE(TAG, "Wi-Fi password must contain at least 8 characters");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG,
                        "event loop creation failed");

    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    if (ap == NULL) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "esp_wifi_init failed");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                   wifi_event_handler, NULL),
        TAG, "event handler registration failed");

    wifi_config_t config = {
        .ap = {
            .channel = CONFIG_DRONE_WIFI_CHANNEL,
            .max_connection = CONFIG_DRONE_WIFI_MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    strlcpy((char *)config.ap.ssid, CONFIG_DRONE_WIFI_SSID,
            sizeof(config.ap.ssid));
    strlcpy((char *)config.ap.password, CONFIG_DRONE_WIFI_PASSWORD,
            sizeof(config.ap.password));
    config.ap.ssid_len = strlen((char *)config.ap.ssid);

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG,
                        "cannot select AP mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config), TAG,
                        "cannot configure AP");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "cannot start AP");

    ESP_LOGI(TAG, "Hotspot started: SSID=%s channel=%d IP=192.168.4.1",
             CONFIG_DRONE_WIFI_SSID, CONFIG_DRONE_WIFI_CHANNEL);
    return ESP_OK;
}
