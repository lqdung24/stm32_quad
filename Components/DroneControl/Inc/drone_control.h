#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#include <stddef.h>
#include <stdint.h>

#define DRONE_CONTROL_LINK_TIMEOUT_MS       300U
#define DRONE_CONTROL_STATUS_PERIOD_MS      50U
#define DRONE_CONTROL_MAX_TEST_THROTTLE     500U

void DroneControl_Init(UART_HandleTypeDef *uart, TIM_HandleTypeDef *motor_timer);
void DroneControl_Process(uint32_t now_ms);

/* Call from the corresponding STM32 HAL callbacks. */
void DroneControl_OnUartRxEvent(UART_HandleTypeDef *uart, uint16_t size);
void DroneControl_OnUartError(UART_HandleTypeDef *uart);

/*
 * Read a best-effort copy of bytes received from the control UART.
 * This trace buffer is separate from the safety-critical protocol RX buffer.
 */
size_t DroneControl_ReadUartRxLog(uint8_t *output, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
