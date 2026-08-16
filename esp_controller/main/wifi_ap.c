#include "wifi_ap.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "status_led.h"

#define WIFI_RSSI_LOG_PERIOD_MS 1000U
#define WIFI_RSSI_TASK_STACK_BYTES 3072U
#define WIFI_RSSI_TASK_PRIORITY 5U

static const char *TAG = "wifi_ap";

static const char *wifi_reason_name(uint16_t reason)
{
    switch (reason) {
    case WIFI_REASON_UNSPECIFIED:
        return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE:
        return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE:
        return "AUTH_LEAVE";
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY:
        return "INACTIVITY";
    case WIFI_REASON_ASSOC_LEAVE:
        return "ASSOC_LEAVE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_STA_LEAVING:
        return "STA_LEAVING";
    case WIFI_REASON_TIMEOUT:
        return "TIMEOUT";
    case WIFI_REASON_PEER_INITIATED:
        return "PEER_INITIATED";
    case WIFI_REASON_AP_INITIATED:
        return "AP_INITIATED";
    case WIFI_REASON_AUTH_FAIL:
        return "AUTH_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL:
        return "CONNECTION_FAIL";
    default:
        return "OTHER";
    }
}

static void wifi_rssi_log_task(void *arg)
{
    (void)arg;
    while (true) {
        wifi_sta_list_t stations = {0};
        vTaskDelay(pdMS_TO_TICKS(WIFI_RSSI_LOG_PERIOD_MS));

        const esp_err_t result = esp_wifi_ap_get_sta_list(&stations);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "[WIFI] RSSI unavailable: %s",
                     esp_err_to_name(result));
            continue;
        }
        for (int station = 0; station < stations.num; ++station) {
            ESP_LOGI(TAG, "[WIFI] RSSI mac=" MACSTR " rssi=%d dBm",
                     MAC2STR(stations.sta[station].mac),
                     stations.sta[station].rssi);
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event = event_data;
        ESP_LOGI(TAG, "[WIFI] AP_STACONNECTED mac=" MACSTR " aid=%u",
                 MAC2STR(event->mac), (unsigned int)event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event = event_data;
        ESP_LOGW(TAG,
                 "[WIFI] AP_STADISCONNECTED mac=" MACSTR
                 " aid=%u reason=%u (%s)",
                 MAC2STR(event->mac), (unsigned int)event->aid,
                 (unsigned int)event->reason,
                 wifi_reason_name(event->reason));
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

    if (xTaskCreate(wifi_rssi_log_task, "wifi_rssi_log",
                    WIFI_RSSI_TASK_STACK_BYTES, NULL,
                    WIFI_RSSI_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Cannot create Wi-Fi RSSI log task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Hotspot started: SSID=%s channel=%d IP=192.168.4.1",
             CONFIG_DRONE_WIFI_SSID, CONFIG_DRONE_WIFI_CHANNEL);
    status_led_set(STATUS_LED_WAITING_FOR_CLIENT);
    return ESP_OK;
}
