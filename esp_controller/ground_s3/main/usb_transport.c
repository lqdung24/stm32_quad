#include "usb_transport.h"

#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "packet_stream.h"

static const char *TAG = "usb_transport"; /* Ground only */
static packet_stream_t s_stream;
static SemaphoreHandle_t s_tx_lock;

#define USB_DEBUG_LINE_MAX_LENGTH 255U

static void usb_packet_received(const uint8_t *packet, size_t length, void *context)
{
    usb_transport_packet_callback_t callback = context;
    if (callback != NULL) {
        callback(packet, length);
    }
}

static void usb_rx_task(void *arg)
{
    (void)arg;
    uint8_t buffer[128];
    while (true) {
        const int received = usb_serial_jtag_read_bytes(buffer, sizeof(buffer),
                                                         pdMS_TO_TICKS(20));
        if (received > 0) {
            packet_stream_feed(&s_stream, buffer, (size_t)received);
        }
    }
}

esp_err_t usb_transport_start(usb_transport_packet_callback_t callback)
{
    ESP_RETURN_ON_FALSE(callback != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "packet callback is null");
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&config), TAG,
                        "USB Serial/JTAG driver install failed");
    s_tx_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_tx_lock != NULL, ESP_ERR_NO_MEM, TAG,
                        "cannot create transmit lock");
    packet_stream_init(&s_stream, usb_packet_received, (void *)callback);
    ESP_RETURN_ON_FALSE(xTaskCreate(usb_rx_task, "usb_rx", 4096, NULL, 12,
                                    NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "cannot start USB receive task");
    return ESP_OK;
}

esp_err_t usb_transport_send_packet(const uint8_t *packet, size_t length)
{
    uint8_t frame[DRONE_PROTOCOL_MAX_PACKET_SIZE + 3U];
    const size_t frame_length = packet_stream_encode(packet, length, frame,
                                                     sizeof(frame));
    if (frame_length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const int written = usb_serial_jtag_write_bytes(frame, frame_length,
                                                     pdMS_TO_TICKS(20));
    xSemaphoreGive(s_tx_lock);
    return written == (int)frame_length ? ESP_OK : ESP_FAIL;
}

esp_err_t usb_transport_send_debug_line(const char *line)
{
    uint8_t frame[USB_DEBUG_LINE_MAX_LENGTH + 4U];
    if ((line == NULL) || (s_tx_lock == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    const size_t length = strnlen(line, USB_DEBUG_LINE_MAX_LENGTH + 1U);
    if ((length == 0U) || (length > USB_DEBUG_LINE_MAX_LENGTH)) {
        return ESP_ERR_INVALID_ARG;
    }

    frame[0] = 0U;
    memcpy(&frame[1], line, length);
    frame[length + 1U] = '\r';
    frame[length + 2U] = '\n';
    frame[length + 3U] = 0U;

    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const size_t frame_length = length + 4U;
    const int written = usb_serial_jtag_write_bytes(frame, frame_length,
                                                     pdMS_TO_TICKS(20));
    xSemaphoreGive(s_tx_lock);
    return written == (int)frame_length ? ESP_OK : ESP_FAIL;
}
