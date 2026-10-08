#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "../../Motor/Inc/motor.h"
#include "../../RateControl/Inc/rate_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DRONE_CONTROL_LINK_TIMEOUT_MS       300U
#define DRONE_CONTROL_ATTITUDE_TIMEOUT_MS    20U
/* DShot pre-arm: sustained stop frames before the first positive throttle. */
#define DRONE_CONTROL_DSHOT_ZERO_ARM_MS    1000U
#define DRONE_CONTROL_DSHOT_ARM_TIMEOUT_MS 2000U
#define DRONE_CONTROL_DSHOT_ZERO_MAX_GAP_MS 20U
#define DRONE_CONTROL_DSHOT_ZERO_MIN_FRAMES 100U
#define DRONE_CONTROL_STATUS_PERIOD_MS      50U
#define DRONE_CONTROL_FLIGHT_TELEMETRY_PERIOD_MS 20U
/* Pilot input 1..500 maps to 1225..1800 us before PID/mixer corrections. */
#define DRONE_CONTROL_MAX_TEST_THROTTLE     500U

typedef struct
{
  float pid_output[RATE_CONTROL_AXIS_COUNT];
  float motor_command[MOTOR_OUTPUT_MAX_MOTORS];
  uint16_t pulse_us[MOTOR_OUTPUT_MAX_MOTORS];
  uint16_t applied_throttle;
  float applied_collective;
  float correction_scale;
  bool active;
  bool collective_shifted;
  bool correction_scaled;
} DroneMixerTelemetry;

void DroneControl_Init(UART_HandleTypeDef *uart, MotorOutput *motors);
void DroneControl_Process(uint32_t now_ms);
/* Flight task: call after the fresh IMU/PID sample, before slower diagnostics.
 * DShot commits the latest bank every 2 ms; PWM keeps immediate updates. */
void DroneControl_ServiceMotorOutput(uint32_t now_ms);
/* Call in the flight task before blocking IMU reinitialization/calibration. */
void DroneControl_InvalidateImuSample(void);

/*
 * Each fresh IMU sample: Mahony6 BODY-to-NED roll/pitch in degrees and
 * calibrated BODY FRD gyro in rad/s, using the same sample interval.
 * Angle mode requires a valid attitude younger than 20 ms and dt <=20 ms.
 * Invalid/stale attitude while armed in angle mode enters latched failsafe.
 */
bool DroneControl_UpdateFlightSample(uint32_t timestamp_ms,
                                     bool attitude_valid,
                                     float roll_deg,
                                     float pitch_deg,
                                     float gyro_roll_rad_s,
                                     float gyro_pitch_rad_s,
                                     float gyro_yaw_rad_s,
                                     float dt_s);
/* Compatibility hook for acro only; cannot supply angle-mode validity. */
bool DroneControl_UpdateBodyRates(float roll_rad_s,
                                  float pitch_rad_s,
                                  float yaw_rad_s,
                                  float dt_s);
bool DroneControl_GetRateControlDebug(RateControlDebug *debug);
bool DroneControl_GetMixerTelemetry(DroneMixerTelemetry *telemetry);

/*
 * Publish a fresh IMU/control-loop sample for the best-effort telemetry
 * stream, immediately after UpdateFlightSample for the SAME sample/timestamp.
 * Attitude is in degrees; gyro is BODY FRD rad/s. Active DShot flight caches
 * the IMU/setpoint/PID tuple until a successful motor commit completes it.
 * PWM completes after its immediate commit. Test/disarmed samples explicitly
 * report output_sample_matched=false. This never changes actuator output.
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
