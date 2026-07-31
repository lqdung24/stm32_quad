#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
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

/* Print one IMU/rate-loop timing summary per second over USB CDC. */
#ifndef APP_TIMING_LOG_ENABLE
#define APP_TIMING_LOG_ENABLE 1U
#endif

typedef uint8_t (*App_UsbTransmitFn)(uint8_t *data, uint16_t length);

typedef struct
{
  uint32_t sample_count;
  uint32_t data_not_ready_count;
  uint32_t read_error_count;
  uint32_t actual_rate_millihz;
  uint32_t period_min_us;
  uint32_t period_mean_us;
  uint32_t period_max_us;
  uint32_t pipeline_min_us;
  uint32_t pipeline_mean_us;
  uint32_t pipeline_max_us;
  bool cycle_counter_available;
} AppTimingStats;

void App_SetSpi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void App_SetUsbTransmit(App_UsbTransmitFn transmit);
void App_SetActivityLed(GPIO_TypeDef *port, uint16_t pin);
void App_SetMotorTimer(TIM_HandleTypeDef *htim);
void App_SetControlUart(UART_HandleTypeDef *huart);

/* Called by USB callbacks; control execution remains in flightControl task. */
void App_OnUsbReceive(const uint8_t *data, uint32_t length);
void App_OnUsbTransmitComplete(void);

void App_Init(void);

/*
 * Compatibility entry point called by the CubeMX default task. It bootstraps
 * the hand-owned RTOS tasks and does not run the old cooperative super-loop.
 */
void App_Process(void);

/* Task-owned periodic steps. now_ms uses the STM32 HAL millisecond timebase. */
void App_FlightControlStep(uint32_t now_ms);
void App_TelemetryStep(uint32_t now_ms);
void App_HousekeepingStep(uint32_t now_ms);
bool App_GetTimingStats(AppTimingStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
