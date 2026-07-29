#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*
 * Set to 1U to print ICM20948/AK09916 init, calibration, attitude and debug
 * logs. Motor command responses are independent and always remain enabled.
 */
#ifndef APP_ICM20948_LOG_ENABLE
#define APP_ICM20948_LOG_ENABLE 0U
#endif

/* Print received USART1 bytes as hexadecimal lines over USB CDC. */
#ifndef APP_UART1_RX_LOG_ENABLE
#define APP_UART1_RX_LOG_ENABLE 1U
#endif

typedef uint8_t (*App_UsbTransmitFn)(uint8_t *data, uint16_t length);

void App_SetSpi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void App_SetUsbTransmit(App_UsbTransmitFn transmit);
void App_SetActivityLed(GPIO_TypeDef *port, uint16_t pin);
void App_SetMotorTimer(TIM_HandleTypeDef *htim);
void App_SetControlUart(UART_HandleTypeDef *huart);

/* Called by the USB CDC receive callback; command execution stays in App_Process. */
void App_OnUsbReceive(const uint8_t *data, uint32_t length);
void App_OnUsbTransmitComplete(void);

void App_Init(void);
void App_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
