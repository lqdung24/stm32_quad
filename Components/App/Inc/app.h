#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef uint8_t (*App_UsbTransmitFn)(uint8_t *data, uint16_t length);

void App_SetSpi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void App_SetUsbTransmit(App_UsbTransmitFn transmit);
void App_SetActivityLed(GPIO_TypeDef *port, uint16_t pin);

void App_Init(void);
void App_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
