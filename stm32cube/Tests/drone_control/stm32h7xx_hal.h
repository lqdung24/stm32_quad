#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>

typedef struct
{
  uint32_t gState;
} UART_HandleTypeDef;

typedef enum { HAL_OK = 0, HAL_ERROR } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0U
#define __DMB() ((void)0)

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *uart,
                                              uint8_t *buffer,
                                              uint16_t length);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                      uint8_t *buffer,
                                      uint16_t length);

#endif
