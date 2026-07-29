#include "uart_transport.h"

#include <string.h>

#include "driver/uart.h"
#include "drone_cobs.h"
#include "drone_protocol.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define UART_READ_CHUNK_SIZE 128
#define UART_ENCODED_FRAME_SIZE 64

static const char *TAG = "uart_transport";
static uart_port_t s_uart_port;
static SemaphoreHandle_t s_tx_lock;
static uart_transport_packet_callback_t s_packet_callback;

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t read_buffer[UART_READ_CHUNK_SIZE];
    uint8_t encoded[UART_ENCODED_FRAME_SIZE];
    size_t encoded_length = 0;

    while (true) {
        const int received = uart_read_bytes(s_uart_port, read_buffer,
                                             sizeof(read_buffer),
                                             pdMS_TO_TICKS(20));
        for (int i = 0; i < received; ++i) {
            const uint8_t byte = read_buffer[i];
            if (byte == 0) {
                uint8_t raw[DRONE_PROTOCOL_MAX_PACKET_SIZE];
                const size_t raw_length =
                    DroneCobs_Decode(encoded, encoded_length,
                                    raw, sizeof(raw));
                if (raw_length != 0 && s_packet_callback != NULL) {
                    s_packet_callback(raw, raw_length);
                } else if (encoded_length != 0) {
                    ESP_LOGW(TAG, "Dropped malformed COBS frame");
                }
                encoded_length = 0;
            } else if (encoded_length < sizeof(encoded)) {
                encoded[encoded_length++] = byte;
            } else {
                encoded_length = 0;
                ESP_LOGW(TAG, "UART frame exceeded maximum size");
            }
        }
    }
}

esp_err_t uart_transport_start(uart_transport_packet_callback_t callback)
{
    ESP_RETURN_ON_FALSE(callback != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "packet callback is null");

    s_uart_port = (uart_port_t)CONFIG_DRONE_UART_PORT;
    s_packet_callback = callback;
    s_tx_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_tx_lock != NULL, ESP_ERR_NO_MEM, TAG,
                        "cannot create TX mutex");

    const uart_config_t uart_config = {
        .baud_rate = CONFIG_DRONE_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(s_uart_port, 1024, 0, 0,
                                            NULL, 0),
                        TAG, "UART driver install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(s_uart_port, &uart_config),
                        TAG, "UART parameter setup failed");
    ESP_RETURN_ON_ERROR(
        uart_set_pin(s_uart_port,
                     CONFIG_DRONE_UART_TX_GPIO,
                     CONFIG_DRONE_UART_RX_GPIO,
                     UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE),
        TAG, "UART pin setup failed");

    BaseType_t task_result =
        xTaskCreate(uart_rx_task, "drone_uart_rx", 4096, NULL, 10, NULL);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "cannot start UART RX task");

    ESP_LOGI(TAG, "UART%d %d baud: TX GPIO%d, RX GPIO%d",
             CONFIG_DRONE_UART_PORT, CONFIG_DRONE_UART_BAUD_RATE,
             CONFIG_DRONE_UART_TX_GPIO, CONFIG_DRONE_UART_RX_GPIO);
    return ESP_OK;
}

esp_err_t uart_transport_send_packet(const uint8_t *packet, size_t length)
{
    uint8_t frame[DRONE_PROTOCOL_MAX_PACKET_SIZE + 3];

    if (packet == NULL || length == 0 ||
        length > DRONE_PROTOCOL_MAX_PACKET_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t encoded_length =
        DroneCobs_Encode(packet, length, frame, sizeof(frame) - 1);
    if (encoded_length == 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    frame[encoded_length++] = 0;

    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const int written = uart_write_bytes(s_uart_port, frame, encoded_length);
    xSemaphoreGive(s_tx_lock);
    return written == (int)encoded_length ? ESP_OK : ESP_FAIL;
}
