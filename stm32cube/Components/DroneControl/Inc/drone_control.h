#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "motor_pwm.h"
#include "../../RateControl/Inc/rate_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DRONE_CONTROL_LINK_TIMEOUT_MS       300U
#define DRONE_CONTROL_STATUS_PERIOD_MS      50U
#define DRONE_CONTROL_FLIGHT_TELEMETRY_PERIOD_MS 20U
/* Pilot input 1..500 maps to 1225..1800 us before PID/mixer corrections. */
#define DRONE_CONTROL_MAX_TEST_THROTTLE     500U

typedef struct
{
  float pid_output[RATE_CONTROL_AXIS_COUNT];
  float motor_command[MOTOR_PWM_MOTOR_COUNT];
  uint16_t pulse_us[MOTOR_PWM_MOTOR_COUNT];
  uint16_t applied_throttle;
  float applied_collective;
  float correction_scale;
  bool active;
  bool collective_shifted;
  bool correction_scaled;
} DroneMixerTelemetry;

void DroneControl_Init(UART_HandleTypeDef *uart, MotorPwm_Handle_t *motors);
void DroneControl_Process(uint32_t now_ms);

/*
 * Supply each new calibrated gyroscope sample in BODY FRD, rad/s. A valid
 * armed update applies the rate-PID correction through the Quad-X mixer.
 */
bool DroneControl_UpdateBodyRates(float roll_rad_s,
                                  float pitch_rad_s,
                                  float yaw_rad_s,
                                  float dt_s);
bool DroneControl_GetRateControlDebug(RateControlDebug *debug);
bool DroneControl_GetMixerTelemetry(DroneMixerTelemetry *telemetry);

/*
 * Publish a fresh IMU/control-loop sample for the best-effort telemetry
 * stream. Attitude is in degrees; gyro is BODY FRD rad/s. This function never
 * changes flight state or actuator output.
 */
void DroneControl_PublishFlightTelemetrySample(uint32_t timestamp_ms,
                                               bool attitude_valid,
                                               float roll_deg,
                                               float pitch_deg,
                                               float yaw_deg,
                                               float gyro_roll_rad_s,
                                               float gyro_pitch_rad_s,
                                               float gyro_yaw_rad_s);

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
