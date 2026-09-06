This file is a merged representation of the entire codebase, combined into a single document by Repomix.
The content has been processed where content has been compressed (code blocks are separated by ⋮---- delimiter).

# File Summary

## Purpose
This file contains a packed representation of the entire repository's contents.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.

## File Format
The content is organized as follows:
1. This summary section
2. Repository information
3. Directory structure
4. Repository files (if enabled)
5. Multiple file entries, each consisting of:
  a. A header with the file path (## File: path/to/file)
  b. The full contents of the file in a code block

## Usage Guidelines
- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.

## Notes
- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Content has been compressed - code blocks are separated by ⋮---- delimiter
- Files are sorted by Git change count (files with more changes are at the bottom)

# Directory Structure
```
Inc/
  app_rtos.h
  app.h
Src/
  app_rtos.c
  app.c
```

# Files

## File: Inc/app_rtos.h
```c
/*
 * Set to 0 only when isolating scheduler/USB/LED behavior from the IMU and
 * rate-control path.
 */
⋮----
/*
 * Called by the CubeMX default task. It creates the hand-owned application
 * tasks once, then keeps that task alive as the housekeeping/supervisor task.
 */
void AppRtos_Bootstrap(void);
⋮----
#endif /* APP_RTOS_H */
```

## File: Inc/app.h
```c
/*
 * Set to 1U to print ICM20948/AK09916 init, calibration, attitude and debug
 * logs. Motor command responses are independent and always remain enabled.
 */
⋮----
/* Print received USART1 bytes as hexadecimal lines over USB CDC. */
⋮----
/* Print one IMU/rate-loop timing summary per second over USB CDC. */
⋮----
/* Print a RAM snapshot of PID/mixer/motor outputs at 10 Hz over USB CDC. */
⋮----
} AppTimingStats;
⋮----
void App_SetSpi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void App_SetUsbTransmit(App_UsbTransmitFn transmit);
void App_SetActivityLed(GPIO_TypeDef *port, uint16_t pin);
void App_SetMotorTimer(TIM_HandleTypeDef *htim);
void App_SetControlUart(UART_HandleTypeDef *huart);
⋮----
/* Called by USB callbacks; control execution remains in flightControl task. */
void App_OnUsbReceive(const uint8_t *data, uint32_t length);
void App_OnUsbTransmitComplete(void);
⋮----
void App_Init(void);
⋮----
/*
 * Compatibility entry point called by the CubeMX default task. It bootstraps
 * the hand-owned RTOS tasks and does not run the old cooperative super-loop.
 */
void App_Process(void);
⋮----
/* Task-owned periodic steps. now_ms uses the STM32 HAL millisecond timebase. */
void App_FlightControlStep(uint32_t now_ms);
void App_TelemetryStep(uint32_t now_ms);
void App_HousekeepingStep(uint32_t now_ms);
bool App_GetTimingStats(AppTimingStats *stats);
⋮----
#endif /* APP_H */
```

## File: Src/app_rtos.c
```c
static void flight_task(void *argument);
⋮----
static void telemetry_task(void *argument);
static uint32_t milliseconds_to_ticks(uint32_t period_ms);
static void delay_until_next_period(uint32_t *next_tick,
⋮----
static void terminate_thread(osThreadId_t *thread);
⋮----
void AppRtos_Bootstrap(void)
⋮----
/*
   * Keep the CubeMX default task alive as the housekeeping/supervisor task.
   * This leaves a visible LED heartbeat even if a dynamically-created task
   * cannot be allocated or later fails.
   */
⋮----
/*
     * Retry from a clean state. Motor output remains at the disarmed value
     * set during App_Init(), while the default task continues blinking the
     * activity LED to prove that the scheduler is alive.
     */
⋮----
static void flight_task(void *argument)
⋮----
static void telemetry_task(void *argument)
⋮----
static uint32_t milliseconds_to_ticks(uint32_t period_ms)
⋮----
/*
     * The task missed its absolute deadline. Resynchronize instead of
     * spinning at high priority and starving lower-priority tasks. Yielding
     * is insufficient because FreeRTOS only yields to ready tasks at the same
     * priority; a one-tick delay guarantees telemetry and housekeeping can
     * run after an overrun.
     */
⋮----
static void terminate_thread(osThreadId_t *thread)
```

## File: Src/app.c
```c
static void App_TryInitICM20948(void);
static bool App_StartMotorPwm(MotorPwm_Handle_t *motors);
static void App_CalibrateGyro(void);
static void App_UpdateAttitude(void);
static void App_UpdateMagnetometer(void);
static void App_ReportICM20948(void);
static void App_ReportMagDebug(void);
static void App_IcmLog(const char *text, int length);
static void App_FlushControlUartLog(void);
static void App_ReportTiming(void);
static void App_ReportMixer(void);
static int32_t App_FloatToTenths(float value);
⋮----
static void App_UsbSend(const char *text, int length);
⋮----
static int32_t App_GyroRawToMdps(int32_t raw);
static int32_t App_TempRawToCentiC(int16_t raw);
static int32_t App_MagRawToCentiUt(int32_t raw);
static void App_EnableCycleCounter(void);
static void App_ResetTimingStats(void);
static uint32_t App_CyclesToUs(uint64_t cycles);
static void App_RecordSampleTiming(uint32_t sample_cycle,
⋮----
void App_SetSpi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
⋮----
void App_SetUsbTransmit(App_UsbTransmitFn transmit)
⋮----
void App_SetActivityLed(GPIO_TypeDef *port, uint16_t pin)
⋮----
void App_SetMotorTimer(TIM_HandleTypeDef *htim)
⋮----
void App_SetControlUart(UART_HandleTypeDef *huart)
⋮----
void App_OnUsbReceive(const uint8_t *data, uint32_t length)
⋮----
/*
   * USB CDC is diagnostics-only. Safety-critical control is accepted only
   * through the validated Drone Control Packet path on USART1.
   */
⋮----
void App_OnUsbTransmitComplete(void)
⋮----
void App_Init(void)
⋮----
static bool App_StartMotorPwm(MotorPwm_Handle_t *motors)
⋮----
void App_Process(void)
⋮----
void App_FlightControlStep(uint32_t now_ms)
⋮----
/*
   * Service the lower-bandwidth magnetometer only after the gyro/rate loop.
   * This prevents an auxiliary-I2C retry from delaying an already-ready gyro
   * sample.
   */
⋮----
void App_TelemetryStep(uint32_t now_ms)
⋮----
void App_HousekeepingStep(uint32_t now_ms)
⋮----
bool App_GetTimingStats(AppTimingStats *stats)
⋮----
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
⋮----
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
⋮----
static void App_FlushControlUartLog(void)
⋮----
static void App_ReportTiming(void)
⋮----
static void App_ReportMixer(void)
⋮----
static int32_t App_FloatToTenths(float value)
⋮----
static void App_TryInitICM20948(void)
⋮----
static void App_CalibrateGyro(void)
⋮----
/* Let the gyro settle thermally, then discard startup transients. */
⋮----
static void App_UpdateAttitude(void)
⋮----
/*
   * The inner rate loop uses only the calibrated gyroscope, so it must keep
   * working even when the magnetometer is unavailable and Mahony has not
   * initialized an absolute attitude yet.
   */
⋮----
static void App_UpdateMagnetometer(void)
⋮----
/* A zero shadow is not a valid magnetic sample; do not let the
       calibration matrix turn it into a plausible-looking constant. */
⋮----
static void App_ReportICM20948(void)
⋮----
static void App_ReportMagDebug(void)
⋮----
static void App_IcmLog(const char *text, int length)
⋮----
static void App_UsbSend(const char *text, int length)
⋮----
static int32_t App_GyroRawToMdps(int32_t raw)
⋮----
static int32_t App_TempRawToCentiC(int16_t raw)
⋮----
static int32_t App_MagRawToCentiUt(int32_t raw)
⋮----
static void App_EnableCycleCounter(void)
⋮----
static uint32_t App_CyclesToUs(uint64_t cycles)
⋮----
static void App_ResetTimingStats(void)
⋮----
/*
     * Do not include a disarmed/failsafe gap in the next PID period. The first
     * successful update after re-arm starts a fresh timing sequence.
     */
```
