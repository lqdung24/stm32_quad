#include "status_led.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef CONFIG_DRONE_STATUS_LED_SINGLE_COLOR
#include "driver/gpio.h"
#else
#include "led_strip.h"
#endif

#define STATUS_LED_INDEX 0U
#define STATUS_LED_TASK_STACK_BYTES 2048U
#define STATUS_LED_TASK_PRIORITY 4U
#define STATUS_LED_STEP_MS 100U

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} status_led_color_t;

#ifdef CONFIG_DRONE_STATUS_LED_SINGLE_COLOR
static const char *TAG = "status_led"; /* Air: ordinary GPIO LED. */
#else
static const char *TAG = "status_led"; /* Ground: WS2812 RGB LED. */
static led_strip_handle_t s_strip;
#endif
static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static status_led_state_t s_state = STATUS_LED_BOOT;
static bool s_initialized;

static status_led_color_t status_led_color_for(status_led_state_t state,
                                                uint32_t phase)
{
    const status_led_color_t off = {0, 0, 0};

    switch (state) {
    case STATUS_LED_BOOT:
        /* Amber blink: firmware is starting. */
        return (phase % 5U) < 2U ? (status_led_color_t){20, 8, 0} : off;
    case STATUS_LED_WAITING_FOR_CLIENT:
        /* Blue blink: the ground bridge is ready for a USB controller. */
        return (phase % 10U) < 5U ? (status_led_color_t){0, 0, 18} : off;
    case STATUS_LED_CLIENT_CONNECTED:
        /* Cyan blink: the ESP-NOW bridge is linked; STM32 status is pending. */
        return (phase % 10U) < 8U ? (status_led_color_t){0, 16, 16} : off;
    case STATUS_LED_DISARMED:
        return (status_led_color_t){0, 20, 0};
    case STATUS_LED_ARMED:
        /* Red is deliberately steady while motors are armed. */
        return (status_led_color_t){24, 0, 0};
    case STATUS_LED_FAULT:
        /* Two short red flashes repeat for failsafe or STM32 error. */
        return ((phase % 10U) < 2U ||
                ((phase % 10U) >= 4U && (phase % 10U) < 6U))
                   ? (status_led_color_t){24, 0, 0}
                   : off;
    default:
        return off;
    }
}

static bool status_led_colors_equal(status_led_color_t left,
                                    status_led_color_t right)
{
    return left.red == right.red && left.green == right.green &&
           left.blue == right.blue;
}

#ifdef CONFIG_DRONE_STATUS_LED_SINGLE_COLOR
static bool status_led_is_on(status_led_color_t color)
{
    return color.red != 0U || color.green != 0U || color.blue != 0U;
}
#endif

static void status_led_task(void *arg)
{
    (void)arg;
    uint32_t phase = 0;
    status_led_state_t previous_state = STATUS_LED_FAULT;
    status_led_color_t previous_color = {UINT8_MAX, UINT8_MAX, UINT8_MAX};

    while (true) {
        status_led_state_t state;
        portENTER_CRITICAL(&s_lock);
        state = s_state;
        portEXIT_CRITICAL(&s_lock);

        if (state != previous_state) {
            phase = 0;
            previous_state = state;
        }

        const status_led_color_t color = status_led_color_for(state, phase);
        if (!status_led_colors_equal(color, previous_color)) {
#ifdef CONFIG_DRONE_STATUS_LED_SINGLE_COLOR
            (void)gpio_set_level(CONFIG_DRONE_STATUS_LED_GPIO,
                                 status_led_is_on(color) ? 1 : 0);
#else
            if (color.red == 0U && color.green == 0U && color.blue == 0U) {
                (void)led_strip_clear(s_strip);
            } else {
                (void)led_strip_set_pixel(s_strip, STATUS_LED_INDEX,
                                          color.red, color.green, color.blue);
                (void)led_strip_refresh(s_strip);
            }
#endif
            previous_color = color;
        }

        ++phase;
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(STATUS_LED_STEP_MS));
    }
}

esp_err_t status_led_init(void)
{
#ifdef CONFIG_DRONE_STATUS_LED_SINGLE_COLOR
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << CONFIG_DRONE_STATUS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_set_level(CONFIG_DRONE_STATUS_LED_GPIO, 0);
    if (err != ESP_OK) {
        return err;
    }
#else
    const led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_DRONE_STATUS_LED_GPIO,
        .max_leds = 1,
    };
    const led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config,
                                              &s_strip);
    if (err != ESP_OK) {
        return err;
    }

    err = led_strip_clear(s_strip);
    if (err != ESP_OK) {
        (void)led_strip_del(s_strip);
        s_strip = NULL;
        return err;
    }
#endif

    if (xTaskCreate(status_led_task, "status_led", STATUS_LED_TASK_STACK_BYTES,
                    NULL, STATUS_LED_TASK_PRIORITY, &s_task) != pdPASS) {
#ifdef CONFIG_DRONE_STATUS_LED_SINGLE_COLOR
        (void)gpio_set_level(CONFIG_DRONE_STATUS_LED_GPIO, 0);
#else
        (void)led_strip_clear(s_strip);
        (void)led_strip_del(s_strip);
        s_strip = NULL;
#endif
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
#ifdef CONFIG_DRONE_STATUS_LED_SINGLE_COLOR
    ESP_LOGI(TAG, "Single-color status LED on GPIO%d",
             CONFIG_DRONE_STATUS_LED_GPIO);
#else
    ESP_LOGI(TAG, "WS2812 status LED on GPIO%d via RMT",
             CONFIG_DRONE_STATUS_LED_GPIO);
#endif
    return ESP_OK;
}

void status_led_set(status_led_state_t state)
{
    bool changed = false;

    if (!s_initialized || state > STATUS_LED_FAULT) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    if (s_state != state) {
        s_state = state;
        changed = true;
    }
    portEXIT_CRITICAL(&s_lock);

    if (changed && s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}
