#pragma once

#include "esp_err.h"

typedef enum {
    STATUS_LED_BOOT,
    STATUS_LED_WAITING_FOR_CLIENT,
    STATUS_LED_CLIENT_CONNECTED,
    STATUS_LED_DISARMED,
    STATUS_LED_ARMED,
    STATUS_LED_FAULT,
} status_led_state_t;

/* Initialize the WS2812 LED strip using the RMT backend. */
esp_err_t status_led_init(void);

/* This function is safe to call from normal FreeRTOS task/event contexts. */
void status_led_set(status_led_state_t state);
