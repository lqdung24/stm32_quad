#include "app.h"

#include "app_rtos.h"
#include "attitude.h"
#include "drone_control.h"
#include "icm20948.h"
#include "mahony9.h"
#include <limits.h>
#include <stdio.h>

#define APP_ICM20948_POLL_PERIOD_MS          1U
#define APP_ICM20948_REPORT_PERIOD_MS       20U
#define APP_ICM20948_MAG_PERIOD_MS          10U
#define APP_LED_BLINK_PERIOD_MS				1000U
#define APP_ICM20948_REINIT_PERIOD_MS       1000U
#define APP_MAG_DEBUG_PERIOD_MS             1000U
#define APP_MIXER_REPORT_PERIOD_MS           100U
#define APP_GYRO_CALIBRATION_DISCARD_SAMPLES 200U
#define APP_GYRO_CALIBRATION_SAMPLES        1000U
#define APP_UART_LOG_BYTES_PER_LINE         16U
#define APP_USB_TX_OK                       0U
#define APP_ICM20948_SAMPLE_RATE_DIVIDER     0U
#define APP_ICM20948_DLPF_CONFIG             3U
#define APP_ICM20948_NOMINAL_RATE_HZ      1125U
#define APP_MOTOR_DISARMED_PULSE_US       1000U
#define APP_MOTOR_MINIMUM_PULSE_US        1000U
#define APP_MOTOR_MAXIMUM_PULSE_US        2000U
#define APP_ICM20948_ACCEL_LSB_PER_G \
  ICM20948_ACCEL_4G_LSB_PER_G
#define APP_ICM20948_GYRO_LSB_PER_DPS \
  ICM20948_GYRO_1000DPS_LSB_PER_DPS
static SPI_HandleTypeDef *app_spi;
static GPIO_TypeDef *app_icm_cs_port;
static uint16_t app_icm_cs_pin;
static App_UsbTransmitFn app_usb_transmit;
static GPIO_TypeDef *app_activity_led_port;
static uint16_t app_activity_led_pin;
static TIM_HandleTypeDef *app_motor_timer;
static UART_HandleTypeDef *app_control_uart;
static volatile uint8_t app_uart_log_usb_busy;
static MotorPwm_Handle_t app_motors;

static ICM20948_Handle_t icm20948;
static ICM20948_RawData_t icm20948_raw;
static ICM20948_VectorRaw_t icm20948_mag_raw;
static ICM20948_VectorRaw_t icm20948_gyro_bias;
static Attitude_VectorRaw_t icm20948_mag_body_cal_cuT;
static uint8_t icm20948_who_am_i;
static ICM20948_Status_t icm20948_status = ICM20948_ERROR;
static ICM20948_Status_t icm20948_mag_status = ICM20948_ERROR;
static ICM20948_Status_t icm20948_mag_init_status = ICM20948_ERROR;
static uint8_t icm20948_mag_valid;
static uint32_t icm20948_last_sample_ms;
static uint32_t icm20948_last_report_ms;
static uint32_t icm20948_last_mag_ms;
static uint32_t icm20948_last_reinit_ms;
static uint32_t app_last_led_ms;
static uint32_t app_last_mag_debug_ms;
static uint32_t app_last_timing_report_ms;
static uint32_t app_last_mixer_report_ms;
static uint32_t app_last_sample_cycle;
static uint8_t app_has_sample_cycle;
static uint32_t app_previous_profile_sample_cycle;
static uint8_t app_has_previous_profile_sample;
static volatile uint32_t app_timing_sequence;
static uint32_t app_timing_sample_count;
static uint32_t app_timing_not_ready_count;
static uint32_t app_timing_read_error_count;
static uint32_t app_timing_period_count;
static uint32_t app_timing_period_min_cycles = UINT32_MAX;
static uint32_t app_timing_period_max_cycles;
static uint64_t app_timing_period_total_cycles;
static uint32_t app_timing_imu_read_min_cycles = UINT32_MAX;
static uint32_t app_timing_imu_read_max_cycles;
static uint64_t app_timing_imu_read_total_cycles;
static uint32_t app_previous_pid_cycle;
static uint8_t app_has_previous_pid_cycle;
static uint32_t app_timing_pid_update_count;
static uint32_t app_timing_pid_period_count;
static uint32_t app_timing_pid_period_min_cycles = UINT32_MAX;
static uint32_t app_timing_pid_period_max_cycles;
static uint64_t app_timing_pid_period_total_cycles;
static uint32_t app_timing_pid_exec_min_cycles = UINT32_MAX;
static uint32_t app_timing_pid_exec_max_cycles;
static uint64_t app_timing_pid_exec_total_cycles;
static uint32_t app_timing_pipeline_min_cycles = UINT32_MAX;
static uint32_t app_timing_pipeline_max_cycles;
static uint64_t app_timing_pipeline_total_cycles;
static uint8_t app_cycle_counter_available;
static Mahony9_Handle_t mahony9;
static Mahony9_Euler_t attitude;
static const Mahony9_Config_t mahony9_config = {
  .kp = 2.0f,
  .ki = 0.05f,
  .integral_limit_rad_s = 0.1f,
  .accel_min_norm = 0.80f,
  .accel_max_norm = 1.20f,
  .mag_min_norm = 2000.0f,
  .mag_max_norm = 7000.0f
};
static char usb_tx_buffer[384];
#if APP_UART1_RX_LOG_ENABLE
static char uart_log_tx_buffer[80];
#endif
static char timing_tx_buffer[512];
static char mixer_tx_buffer[384];

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
#if APP_ICM20948_LOG_ENABLE
static void App_UsbSend(const char *text, int length);
#endif
static int32_t App_GyroRawToMdps(int32_t raw);
static int32_t App_TempRawToCentiC(int16_t raw);
static int32_t App_MagRawToCentiUt(int32_t raw);
static void App_EnableCycleCounter(void);
static void App_ResetTimingStats(void);
static uint32_t App_CyclesToUs(uint64_t cycles);
static void App_RecordSampleTiming(uint32_t sample_cycle,
                                   uint32_t imu_read_cycles,
                                   uint32_t pipeline_cycles,
                                   bool pid_updated,
                                   uint32_t pid_exec_cycles);

void App_SetSpi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  app_spi = hspi;
  app_icm_cs_port = cs_port;
  app_icm_cs_pin = cs_pin;
}

void App_SetUsbTransmit(App_UsbTransmitFn transmit)
{
  app_usb_transmit = transmit;
}

void App_SetActivityLed(GPIO_TypeDef *port, uint16_t pin)
{
  app_activity_led_port = port;
  app_activity_led_pin = pin;
}

void App_SetMotorTimer(TIM_HandleTypeDef *htim)
{
  app_motor_timer = htim;
}

void App_SetControlUart(UART_HandleTypeDef *huart)
{
  app_control_uart = huart;
}

void App_OnUsbReceive(const uint8_t *data, uint32_t length)
{
  /*
   * USB CDC is diagnostics-only. Safety-critical control is accepted only
   * through the validated Drone Control Packet path on USART1.
   */
  (void)data;
  (void)length;
}

void App_OnUsbTransmitComplete(void)
{
  app_uart_log_usb_busy = 0U;
}

void App_Init(void)
{
  MotorPwm_Handle_t *motors = NULL;
  uint32_t now_ms;

  App_EnableCycleCounter();
  if (App_StartMotorPwm(&app_motors))
  {
    motors = &app_motors;
  }
  DroneControl_Init(app_control_uart, motors);
  App_TryInitICM20948();
  App_CalibrateGyro();
  App_UpdateMagnetometer();
  App_UpdateAttitude();

  now_ms = HAL_GetTick();
  icm20948_last_sample_ms = now_ms;
  icm20948_last_report_ms = now_ms;
  icm20948_last_mag_ms = now_ms;
  icm20948_last_reinit_ms = now_ms;
  app_last_led_ms = now_ms;
  app_last_mag_debug_ms = now_ms;
  app_last_timing_report_ms = now_ms;
  app_last_mixer_report_ms = now_ms;
  app_last_sample_cycle = 0U;
  app_has_sample_cycle = 0U;
  App_ResetTimingStats();

  if (app_activity_led_port != NULL)
  {
    HAL_GPIO_TogglePin(app_activity_led_port, app_activity_led_pin);
  }
}

static bool App_StartMotorPwm(MotorPwm_Handle_t *motors)
{
  const MotorPwm_Config_t motor_config = {
    .timer = app_motor_timer,
    .channel = {
      TIM_CHANNEL_1,
      TIM_CHANNEL_2,
      TIM_CHANNEL_3,
      TIM_CHANNEL_4,
    },
    .disarmed_pulse_us = APP_MOTOR_DISARMED_PULSE_US,
    .minimum_pulse_us = APP_MOTOR_MINIMUM_PULSE_US,
    .maximum_pulse_us = APP_MOTOR_MAXIMUM_PULSE_US,
  };
  TIM_OC_InitTypeDef pwm_config = {0};
  uint8_t i;

  if ((motors == NULL) || (app_motor_timer == NULL))
  {
    return false;
  }

  if (HAL_TIM_PWM_Init(app_motor_timer) != HAL_OK)
  {
    return false;
  }

  pwm_config.OCMode = TIM_OCMODE_PWM1;
  pwm_config.Pulse = APP_MOTOR_DISARMED_PULSE_US;
  pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  pwm_config.OCFastMode = TIM_OCFAST_DISABLE;
  pwm_config.OCIdleState = TIM_OCIDLESTATE_RESET;
  pwm_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    if (HAL_TIM_PWM_ConfigChannel(app_motor_timer,
                                  &pwm_config,
                                  motor_config.channel[i]) != HAL_OK)
    {
      return false;
    }
  }

  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    if (HAL_TIM_PWM_Start(app_motor_timer,
                          motor_config.channel[i]) != HAL_OK)
    {
      while (i > 0U)
      {
        --i;
        (void)HAL_TIM_PWM_Stop(app_motor_timer,
                               motor_config.channel[i]);
      }
      return false;
    }
  }

  if (!MotorPwm_Attach(motors, &motor_config))
  {
    for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
    {
      (void)HAL_TIM_PWM_Stop(app_motor_timer, motor_config.channel[i]);
    }
    return false;
  }

  return true;
}

void App_Process(void)
{
  AppRtos_Bootstrap();
}

void App_FlightControlStep(uint32_t now_ms)
{
  DroneControl_Process(now_ms);

  if ((icm20948_status != ICM20948_OK) &&
      ((now_ms - icm20948_last_reinit_ms) >= APP_ICM20948_REINIT_PERIOD_MS))
  {
    icm20948_last_reinit_ms = now_ms;
    App_TryInitICM20948();
    App_CalibrateGyro();
  }

  if ((now_ms - icm20948_last_sample_ms) >= APP_ICM20948_POLL_PERIOD_MS)
  {
    App_UpdateAttitude();
  }

  /*
   * Service the lower-bandwidth magnetometer only after the gyro/rate loop.
   * This prevents an auxiliary-I2C retry from delaying an already-ready gyro
   * sample.
   */
  if ((now_ms - icm20948_last_mag_ms) >= APP_ICM20948_MAG_PERIOD_MS)
  {
    icm20948_last_mag_ms = now_ms;
    App_UpdateMagnetometer();
  }

  if ((APP_ICM20948_LOG_ENABLE != 0U) &&
      ((now_ms - icm20948_last_report_ms) >= APP_ICM20948_REPORT_PERIOD_MS))
  {
    icm20948_last_report_ms = now_ms;
    App_ReportICM20948();
  }
}

void App_TelemetryStep(uint32_t now_ms)
{
#if APP_TIMING_LOG_ENABLE
  if ((now_ms - app_last_timing_report_ms) >= 1000U)
  {
    app_last_timing_report_ms = now_ms;
    App_ReportTiming();
  }
#endif
#if APP_MIXER_LOG_ENABLE
  if ((now_ms - app_last_mixer_report_ms) >=
      APP_MIXER_REPORT_PERIOD_MS)
  {
    app_last_mixer_report_ms = now_ms;
    App_ReportMixer();
  }
#endif
#if !APP_TIMING_LOG_ENABLE && !APP_MIXER_LOG_ENABLE
  (void)now_ms;
#endif
  App_FlushControlUartLog();
}

void App_HousekeepingStep(uint32_t now_ms)
{
  if ((now_ms - app_last_led_ms) >= APP_LED_BLINK_PERIOD_MS)
  {
    app_last_led_ms = now_ms;
    if (app_activity_led_port != NULL)
    {
      HAL_GPIO_TogglePin(app_activity_led_port, app_activity_led_pin);
    }
  }
}

bool App_GetTimingStats(AppTimingStats *stats)
{
  uint32_t sequence_before;
  uint32_t sequence_after;
  uint32_t sample_count;
  uint32_t not_ready_count;
  uint32_t read_error_count;
  uint32_t period_count;
  uint32_t period_min_cycles;
  uint32_t period_max_cycles;
  uint64_t period_total_cycles;
  uint32_t imu_read_min_cycles;
  uint32_t imu_read_max_cycles;
  uint64_t imu_read_total_cycles;
  uint32_t pid_update_count;
  uint32_t pid_period_count;
  uint32_t pid_period_min_cycles;
  uint32_t pid_period_max_cycles;
  uint64_t pid_period_total_cycles;
  uint32_t pid_exec_min_cycles;
  uint32_t pid_exec_max_cycles;
  uint64_t pid_exec_total_cycles;
  uint32_t pipeline_min_cycles;
  uint32_t pipeline_max_cycles;
  uint64_t pipeline_total_cycles;

  if (stats == NULL)
  {
    return false;
  }

  do
  {
    sequence_before = app_timing_sequence;
    if ((sequence_before & 1U) != 0U)
    {
      continue;
    }
    __DMB();
    sample_count = app_timing_sample_count;
    not_ready_count = app_timing_not_ready_count;
    read_error_count = app_timing_read_error_count;
    period_count = app_timing_period_count;
    period_min_cycles = app_timing_period_min_cycles;
    period_max_cycles = app_timing_period_max_cycles;
    period_total_cycles = app_timing_period_total_cycles;
    imu_read_min_cycles = app_timing_imu_read_min_cycles;
    imu_read_max_cycles = app_timing_imu_read_max_cycles;
    imu_read_total_cycles = app_timing_imu_read_total_cycles;
    pid_update_count = app_timing_pid_update_count;
    pid_period_count = app_timing_pid_period_count;
    pid_period_min_cycles = app_timing_pid_period_min_cycles;
    pid_period_max_cycles = app_timing_pid_period_max_cycles;
    pid_period_total_cycles = app_timing_pid_period_total_cycles;
    pid_exec_min_cycles = app_timing_pid_exec_min_cycles;
    pid_exec_max_cycles = app_timing_pid_exec_max_cycles;
    pid_exec_total_cycles = app_timing_pid_exec_total_cycles;
    pipeline_min_cycles = app_timing_pipeline_min_cycles;
    pipeline_max_cycles = app_timing_pipeline_max_cycles;
    pipeline_total_cycles = app_timing_pipeline_total_cycles;
    __DMB();
    sequence_after = app_timing_sequence;
  } while ((sequence_before != sequence_after) ||
           ((sequence_after & 1U) != 0U));

  stats->imu_sample_count = sample_count;
  stats->data_not_ready_count = not_ready_count;
  stats->read_error_count = read_error_count;
  stats->cycle_counter_available = (app_cycle_counter_available != 0U);
  stats->imu_period_min_us =
      (period_count > 0U) ? App_CyclesToUs(period_min_cycles) : 0U;
  stats->imu_period_mean_us =
      (period_count > 0U)
          ? App_CyclesToUs(period_total_cycles / period_count)
          : 0U;
  stats->imu_period_max_us =
      (period_count > 0U) ? App_CyclesToUs(period_max_cycles) : 0U;
  stats->imu_read_min_us =
      (sample_count > 0U) ? App_CyclesToUs(imu_read_min_cycles) : 0U;
  stats->imu_read_mean_us =
      (sample_count > 0U)
          ? App_CyclesToUs(imu_read_total_cycles / sample_count)
          : 0U;
  stats->imu_read_max_us =
      (sample_count > 0U) ? App_CyclesToUs(imu_read_max_cycles) : 0U;
  stats->pid_update_count = pid_update_count;
  stats->pid_period_min_us =
      (pid_period_count > 0U) ? App_CyclesToUs(pid_period_min_cycles) : 0U;
  stats->pid_period_mean_us =
      (pid_period_count > 0U)
          ? App_CyclesToUs(pid_period_total_cycles / pid_period_count)
          : 0U;
  stats->pid_period_max_us =
      (pid_period_count > 0U) ? App_CyclesToUs(pid_period_max_cycles) : 0U;
  stats->pid_exec_min_us =
      (pid_update_count > 0U) ? App_CyclesToUs(pid_exec_min_cycles) : 0U;
  stats->pid_exec_mean_us =
      (pid_update_count > 0U)
          ? App_CyclesToUs(pid_exec_total_cycles / pid_update_count)
          : 0U;
  stats->pid_exec_max_us =
      (pid_update_count > 0U) ? App_CyclesToUs(pid_exec_max_cycles) : 0U;
  stats->pipeline_min_us =
      (sample_count > 0U) ? App_CyclesToUs(pipeline_min_cycles) : 0U;
  stats->pipeline_mean_us =
      (sample_count > 0U)
          ? App_CyclesToUs(pipeline_total_cycles / sample_count)
          : 0U;
  stats->pipeline_max_us =
      (sample_count > 0U) ? App_CyclesToUs(pipeline_max_cycles) : 0U;
  stats->imu_rate_millihz =
      (period_total_cycles > 0U)
          ? (uint32_t)(((uint64_t)period_count *
                        (uint64_t)SystemCoreClock * 1000ULL) /
                       period_total_cycles)
          : 0U;
  stats->pid_rate_millihz =
      (pid_period_total_cycles > 0U)
          ? (uint32_t)(((uint64_t)pid_period_count *
                        (uint64_t)SystemCoreClock * 1000ULL) /
                       pid_period_total_cycles)
          : 0U;
  return true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  DroneControl_OnUartRxEvent(huart, size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  DroneControl_OnUartError(huart);
}

static void App_FlushControlUartLog(void)
{
#if APP_UART1_RX_LOG_ENABLE
  static const char hex[] = "0123456789ABCDEF";
  uint8_t data[APP_UART_LOG_BYTES_PER_LINE];
  size_t count;
  size_t i;
  size_t length;

  if ((app_usb_transmit == NULL) || (app_uart_log_usb_busy != 0U))
  {
    return;
  }

  count = DroneControl_ReadUartRxLog(data, sizeof(data));
  if (count == 0U)
  {
    return;
  }

  length = (size_t)snprintf(uart_log_tx_buffer,
                            sizeof(uart_log_tx_buffer),
                            "UART1 RX (%u):", (unsigned int)count);
  for (i = 0U;
       (i < count) && ((length + 3U) < sizeof(uart_log_tx_buffer));
       ++i)
  {
    uart_log_tx_buffer[length++] = ' ';
    uart_log_tx_buffer[length++] = hex[data[i] >> 4U];
    uart_log_tx_buffer[length++] = hex[data[i] & 0x0FU];
  }
  uart_log_tx_buffer[length++] = '\r';
  uart_log_tx_buffer[length++] = '\n';

  if (app_usb_transmit((uint8_t *)uart_log_tx_buffer,
                       (uint16_t)length) == APP_USB_TX_OK)
  {
    app_uart_log_usb_busy = 1U;
  }
#endif
}

static void App_ReportTiming(void)
{
#if APP_TIMING_LOG_ENABLE
  AppTimingStats stats;
  uint32_t imu_jitter_peak_to_peak_us;
  uint32_t pid_jitter_peak_to_peak_us;
  int length;

  if ((app_usb_transmit == NULL) || (app_uart_log_usb_busy != 0U) ||
      !App_GetTimingStats(&stats))
  {
    return;
  }

  imu_jitter_peak_to_peak_us =
      stats.imu_period_max_us - stats.imu_period_min_us;
  pid_jitter_peak_to_peak_us =
      stats.pid_period_max_us - stats.pid_period_min_us;
  length = snprintf(
      timing_tx_buffer,
      sizeof(timing_tx_buffer),
      "TIMING imu=%lu.%03luHz n=%lu period_us=%lu/%lu/%lu "
      "jitter_pp=%lu read_us=%lu/%lu/%lu | "
      "pid=%lu.%03luHz n=%lu period_us=%lu/%lu/%lu "
      "jitter_pp=%lu exec_us=%lu/%lu/%lu | "
      "pipe_us=%lu/%lu/%lu nrdy=%lu err=%lu dwt=%u\r\n",
      (unsigned long)(stats.imu_rate_millihz / 1000U),
      (unsigned long)(stats.imu_rate_millihz % 1000U),
      (unsigned long)stats.imu_sample_count,
      (unsigned long)stats.imu_period_min_us,
      (unsigned long)stats.imu_period_mean_us,
      (unsigned long)stats.imu_period_max_us,
      (unsigned long)imu_jitter_peak_to_peak_us,
      (unsigned long)stats.imu_read_min_us,
      (unsigned long)stats.imu_read_mean_us,
      (unsigned long)stats.imu_read_max_us,
      (unsigned long)(stats.pid_rate_millihz / 1000U),
      (unsigned long)(stats.pid_rate_millihz % 1000U),
      (unsigned long)stats.pid_update_count,
      (unsigned long)stats.pid_period_min_us,
      (unsigned long)stats.pid_period_mean_us,
      (unsigned long)stats.pid_period_max_us,
      (unsigned long)pid_jitter_peak_to_peak_us,
      (unsigned long)stats.pid_exec_min_us,
      (unsigned long)stats.pid_exec_mean_us,
      (unsigned long)stats.pid_exec_max_us,
      (unsigned long)stats.pipeline_min_us,
      (unsigned long)stats.pipeline_mean_us,
      (unsigned long)stats.pipeline_max_us,
      (unsigned long)stats.data_not_ready_count,
      (unsigned long)stats.read_error_count,
      stats.cycle_counter_available ? 1U : 0U);

  if (length >= (int)sizeof(timing_tx_buffer))
  {
    length = (int)sizeof(timing_tx_buffer) - 1;
  }
  if ((length > 0) &&
      (app_usb_transmit((uint8_t *)timing_tx_buffer,
                        (uint16_t)length) == APP_USB_TX_OK))
  {
    app_uart_log_usb_busy = 1U;
  }
#endif
}

static void App_ReportMixer(void)
{
#if APP_MIXER_LOG_ENABLE
  DroneMixerTelemetry telemetry;
  int length;

  if ((app_usb_transmit == NULL) || (app_uart_log_usb_busy != 0U) ||
      !DroneControl_GetMixerTelemetry(&telemetry))
  {
    return;
  }

  length = snprintf(
      mixer_tx_buffer,
      sizeof(mixer_tx_buffer),
      "MIX active=%u thr=%u pid_x10=%ld,%ld,%ld "
      "cmd_x10=%ld,%ld,%ld,%ld pwm_us=%u,%u,%u,%u "
      "collective_x10=%ld shift=%u scale_permille=%ld scaled=%u\r\n",
      telemetry.active ? 1U : 0U,
      (unsigned int)telemetry.applied_throttle,
      (long)App_FloatToTenths(
          telemetry.pid_output[RATE_CONTROL_ROLL]),
      (long)App_FloatToTenths(
          telemetry.pid_output[RATE_CONTROL_PITCH]),
      (long)App_FloatToTenths(
          telemetry.pid_output[RATE_CONTROL_YAW]),
      (long)App_FloatToTenths(telemetry.motor_command[0]),
      (long)App_FloatToTenths(telemetry.motor_command[1]),
      (long)App_FloatToTenths(telemetry.motor_command[2]),
      (long)App_FloatToTenths(telemetry.motor_command[3]),
      (unsigned int)telemetry.pulse_us[0],
      (unsigned int)telemetry.pulse_us[1],
      (unsigned int)telemetry.pulse_us[2],
      (unsigned int)telemetry.pulse_us[3],
      (long)App_FloatToTenths(telemetry.applied_collective),
      telemetry.collective_shifted ? 1U : 0U,
      (long)App_FloatToTenths(telemetry.correction_scale * 100.0f),
      telemetry.correction_scaled ? 1U : 0U);

  if (length >= (int)sizeof(mixer_tx_buffer))
  {
    length = (int)sizeof(mixer_tx_buffer) - 1;
  }
  if ((length > 0) &&
      (app_usb_transmit((uint8_t *)mixer_tx_buffer,
                        (uint16_t)length) == APP_USB_TX_OK))
  {
    app_uart_log_usb_busy = 1U;
  }
#endif
}

static int32_t App_FloatToTenths(float value)
{
  return (int32_t)((value >= 0.0f)
                       ? ((value * 10.0f) + 0.5f)
                       : ((value * 10.0f) - 0.5f));
}

static void App_TryInitICM20948(void)
{
  static const ICM20948_MotionConfig_t motion_config = {
    .accel_range = ICM20948_ACCEL_RANGE_4G,
    .gyro_range = ICM20948_GYRO_RANGE_1000DPS,
    .accel_sample_rate_divider = APP_ICM20948_SAMPLE_RATE_DIVIDER,
    .gyro_sample_rate_divider = APP_ICM20948_SAMPLE_RATE_DIVIDER,
    .accel_dlpf_config = APP_ICM20948_DLPF_CONFIG,
    .gyro_dlpf_config = APP_ICM20948_DLPF_CONFIG
  };
  uint8_t mag_wia2 = 0U;
  ICM20948_Status_t who_status;
  int length;

  if ((app_spi == NULL) || (app_icm_cs_port == NULL))
  {
    icm20948_status = ICM20948_ERROR;
    return;
  }

  icm20948_status = ICM20948_Init(&icm20948, app_spi, app_icm_cs_port, app_icm_cs_pin);
  if (icm20948_status == ICM20948_OK)
  {
    icm20948_status =
        ICM20948_ConfigureMotion(&icm20948, &motion_config);
  }
  icm20948_who_am_i = 0U;
  who_status = ICM20948_ReadWhoAmI(&icm20948, &icm20948_who_am_i);
  if (who_status == ICM20948_OK)
  {
    length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                      "ICM20948 WHO_AM_I=0x%02X expected=0x%02X (%s, ignored)\r\n",
                      icm20948_who_am_i,
                      ICM20948_WHO_AM_I_VALUE,
                      (icm20948_who_am_i == ICM20948_WHO_AM_I_VALUE) ? "match" : "mismatch");
  }
  else
  {
    length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                      "ICM20948 WHO_AM_I read error=%d\r\n",
                      (int)who_status);
  }
  App_IcmLog(usb_tx_buffer, length);

  if (icm20948_status == ICM20948_OK)
  {
    icm20948_mag_valid = 0U;
    Mahony9_Init(&mahony9, &mahony9_config);
    length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                      "ICM20948 init OK\r\n");
    App_IcmLog(usb_tx_buffer, length);

    icm20948_mag_init_status = ICM20948_InitMagnetometer(&icm20948, &mag_wia2);
    icm20948_mag_status = icm20948_mag_init_status;
    if (icm20948_mag_init_status == ICM20948_OK)
    {
      length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                        "AK09916 mag init OK, WIA2=0x%02X expected=0x%02X\r\n",
                        mag_wia2,
                        ICM20948_MAG_WIA2_VALUE);
    }
    else
    {
      length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                        "AK09916 mag init error=%d, WIA2=0x%02X expected=0x%02X\r\n",
                        (int)icm20948_mag_init_status,
                        mag_wia2,
                        ICM20948_MAG_WIA2_VALUE);
    }
  }
  else
  {
    length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                      "ICM20948 init error=%d\r\n",
                      (int)icm20948_status);
  }

  App_IcmLog(usb_tx_buffer, length);
}

static void App_CalibrateGyro(void)
{
  ICM20948_RawData_t sample;
  int64_t sum_x = 0;
  int64_t sum_y = 0;
  int64_t sum_z = 0;
  uint32_t count = 0;
  uint32_t i;
  int length;

  icm20948_gyro_bias.x = 0;
  icm20948_gyro_bias.y = 0;
  icm20948_gyro_bias.z = 0;

  if (icm20948_status != ICM20948_OK)
  {
    return;
  }

  /* Let the gyro settle thermally, then discard startup transients. */
  HAL_Delay(500U);
  for (i = 0U; i < APP_GYRO_CALIBRATION_DISCARD_SAMPLES; i++)
  {
    (void)ICM20948_ReadRaw(&icm20948, &sample);
    HAL_Delay(2U);
  }

  for (i = 0U; i < APP_GYRO_CALIBRATION_SAMPLES; i++)
  {
    if (ICM20948_ReadRaw(&icm20948, &sample) == ICM20948_OK)
    {
      sum_x += sample.gyro.x;
      sum_y += sample.gyro.y;
      sum_z += sample.gyro.z;
      count++;
    }
    HAL_Delay(2U);
  }

  if (count == 0U)
  {
    return;
  }

  icm20948_gyro_bias.x = (int16_t)(sum_x / (int64_t)count);
  icm20948_gyro_bias.y = (int16_t)(sum_y / (int64_t)count);
  icm20948_gyro_bias.z = (int16_t)(sum_z / (int64_t)count);

  length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                    "Gyro bias raw=%d,%d,%d samples=%lu\r\n",
                    icm20948_gyro_bias.x,
                    icm20948_gyro_bias.y,
                    icm20948_gyro_bias.z,
                    (unsigned long)count);
  App_IcmLog(usb_tx_buffer, length);
}

static void App_UpdateAttitude(void)
{
  uint32_t pipeline_start_cycle;
  uint32_t sample_cycle;
  uint32_t imu_read_start_cycle;
  uint32_t imu_read_cycles;
  uint32_t pid_start_cycle;
  uint32_t pid_exec_cycles;
  Attitude_VectorRaw_t accel_sensor_raw;
  Attitude_VectorRaw_t gyro_sensor_raw;
  Attitude_VectorRaw_t gyro_bias_sensor_raw;
  Attitude_Vector3f_t accel_body_g;
  Attitude_Vector3f_t gravity_body;
  Attitude_Vector3f_t gyro_body_rad_s;
  bool updated;
  bool pid_updated;
  uint8_t data_ready;
  float dt_s;

  if (icm20948_status != ICM20948_OK)
  {
    return;
  }

  icm20948_status = ICM20948_IsRawDataReady(&icm20948, &data_ready);
  if (icm20948_status != ICM20948_OK)
  {
    ++app_timing_sequence;
    __DMB();
    ++app_timing_read_error_count;
    __DMB();
    ++app_timing_sequence;
    return;
  }
  if (data_ready == 0U)
  {
    ++app_timing_sequence;
    __DMB();
    ++app_timing_not_ready_count;
    __DMB();
    ++app_timing_sequence;
    return;
  }

  sample_cycle = DWT->CYCCNT;
  pipeline_start_cycle = sample_cycle;
  imu_read_start_cycle = DWT->CYCCNT;
  icm20948_status = ICM20948_ReadRaw(&icm20948, &icm20948_raw);
  imu_read_cycles = DWT->CYCCNT - imu_read_start_cycle;
  icm20948_last_sample_ms = HAL_GetTick();
  if (icm20948_status != ICM20948_OK)
  {
    ++app_timing_sequence;
    __DMB();
    ++app_timing_read_error_count;
    __DMB();
    ++app_timing_sequence;
    return;
  }

  accel_sensor_raw.x = icm20948_raw.accel.x;
  accel_sensor_raw.y = icm20948_raw.accel.y;
  accel_sensor_raw.z = icm20948_raw.accel.z;
  accel_body_g = Attitude_AccelRawToBodyG(accel_sensor_raw,
                                           APP_ICM20948_ACCEL_LSB_PER_G);
  gravity_body = Attitude_SpecificForceToGravity(accel_body_g);

  if (app_has_sample_cycle != 0U)
  {
    dt_s = (float)(sample_cycle - app_last_sample_cycle) /
           (float)SystemCoreClock;
  }
  else
  {
    dt_s = 1.0f / (float)APP_ICM20948_NOMINAL_RATE_HZ;
    app_has_sample_cycle = 1U;
  }
  app_last_sample_cycle = sample_cycle;

  /*
   * The inner rate loop uses only the calibrated gyroscope, so it must keep
   * working even when the magnetometer is unavailable and Mahony has not
   * initialized an absolute attitude yet.
   */
  gyro_sensor_raw.x = icm20948_raw.gyro.x;
  gyro_sensor_raw.y = icm20948_raw.gyro.y;
  gyro_sensor_raw.z = icm20948_raw.gyro.z;
  gyro_bias_sensor_raw.x = icm20948_gyro_bias.x;
  gyro_bias_sensor_raw.y = icm20948_gyro_bias.y;
  gyro_bias_sensor_raw.z = icm20948_gyro_bias.z;
  gyro_body_rad_s = Attitude_GyroRawToBodyRadS(gyro_sensor_raw,
                                                gyro_bias_sensor_raw,
                                                APP_ICM20948_GYRO_LSB_PER_DPS);
  pid_start_cycle = DWT->CYCCNT;
  pid_updated = DroneControl_UpdateBodyRates(gyro_body_rad_s.x,
                                             gyro_body_rad_s.y,
                                             gyro_body_rad_s.z,
                                             dt_s);
  pid_exec_cycles = DWT->CYCCNT - pid_start_cycle;

  if (!mahony9.initialized)
  {
    if ((icm20948_mag_valid != 0U) &&
        Mahony9_InitFromAccelMag(&mahony9,
                                 gravity_body.x,
                                 gravity_body.y,
                                 gravity_body.z,
                                 (float)icm20948_mag_body_cal_cuT.x,
                                 (float)icm20948_mag_body_cal_cuT.y,
                                 (float)icm20948_mag_body_cal_cuT.z))
    {
      (void)Mahony9_GetEulerDegrees(&mahony9, &attitude);
    }
    DroneControl_PublishFlightTelemetrySample(
        icm20948_last_sample_ms,
        mahony9.initialized,
        attitude.roll,
        attitude.pitch,
        attitude.yaw,
        gyro_body_rad_s.x,
        gyro_body_rad_s.y,
        gyro_body_rad_s.z);
    App_RecordSampleTiming(sample_cycle,
                           imu_read_cycles,
                           DWT->CYCCNT - pipeline_start_cycle,
                           pid_updated,
                           pid_exec_cycles);
    return;
  }

  if (icm20948_mag_valid != 0U)
  {
    updated = Mahony9_Update(&mahony9,
                             gyro_body_rad_s.x,
                             gyro_body_rad_s.y,
                             gyro_body_rad_s.z,
                             gravity_body.x,
                             gravity_body.y,
                             gravity_body.z,
                             (float)icm20948_mag_body_cal_cuT.x,
                             (float)icm20948_mag_body_cal_cuT.y,
                             (float)icm20948_mag_body_cal_cuT.z,
                             dt_s);
  }
  else
  {
    updated = Mahony9_UpdateImu(&mahony9,
                                gyro_body_rad_s.x,
                                gyro_body_rad_s.y,
                                gyro_body_rad_s.z,
                                gravity_body.x,
                                gravity_body.y,
                                gravity_body.z,
                                dt_s);
  }

  if (updated)
  {
    (void)Mahony9_GetEulerDegrees(&mahony9, &attitude);
  }
  DroneControl_PublishFlightTelemetrySample(icm20948_last_sample_ms,
                                             updated,
                                             attitude.roll,
                                             attitude.pitch,
                                             attitude.yaw,
                                             gyro_body_rad_s.x,
                                             gyro_body_rad_s.y,
                                             gyro_body_rad_s.z);
  App_RecordSampleTiming(sample_cycle,
                         imu_read_cycles,
                         DWT->CYCCNT - pipeline_start_cycle,
                         pid_updated,
                         pid_exec_cycles);
}

static void App_UpdateMagnetometer(void)
{
  Attitude_VectorRaw_t mag_sensor_raw;
  Attitude_VectorRaw_t mag_body_cuT;

  if (icm20948_status != ICM20948_OK)
  {
    return;
  }

  if (icm20948_mag_init_status != ICM20948_OK)
  {
    icm20948_mag_status = icm20948_mag_init_status;
    return;
  }

  icm20948_mag_status = ICM20948_ReadMagRaw(&icm20948, &icm20948_mag_raw);
  if (icm20948_mag_status == ICM20948_OK)
  {
    /* A zero shadow is not a valid magnetic sample; do not let the
       calibration matrix turn it into a plausible-looking constant. */
    if ((icm20948_mag_raw.x != 0) ||
        (icm20948_mag_raw.y != 0) ||
        (icm20948_mag_raw.z != 0))
    {
      mag_sensor_raw.x = icm20948_mag_raw.x;
      mag_sensor_raw.y = icm20948_mag_raw.y;
      mag_sensor_raw.z = icm20948_mag_raw.z;
      mag_body_cuT = Attitude_MapMagSensorRawToBody(mag_sensor_raw);
      mag_body_cuT.x = App_MagRawToCentiUt(mag_body_cuT.x);
      mag_body_cuT.y = App_MagRawToCentiUt(mag_body_cuT.y);
      mag_body_cuT.z = App_MagRawToCentiUt(mag_body_cuT.z);
      icm20948_mag_body_cal_cuT =
          Attitude_CalibrateMagBodyCentiUt(mag_body_cuT);
      icm20948_mag_valid = 1U;
    }
    else
    {
      icm20948_mag_valid = 0U;
      icm20948_mag_status = ICM20948_ERROR_MAG_NOT_READY;
    }
  }
  else
  {
    icm20948_mag_valid = 0U;
  }
}

static void App_ReportICM20948(void)
{
  Attitude_VectorRaw_t accel_sensor_raw;
  Attitude_Vector3f_t accel_body_g;
  Attitude_VectorRaw_t gyro_sensor_raw;
  Attitude_VectorRaw_t gyro_body_raw;
  char mag_report[48];
  int length;

  if (icm20948_status != ICM20948_OK)
  {
    return;
  }

  accel_sensor_raw.x = icm20948_raw.accel.x;
  accel_sensor_raw.y = icm20948_raw.accel.y;
  accel_sensor_raw.z = icm20948_raw.accel.z;
  accel_body_g = Attitude_AccelRawToBodyG(accel_sensor_raw,
                                           APP_ICM20948_ACCEL_LSB_PER_G);

  gyro_sensor_raw.x = (int32_t)icm20948_raw.gyro.x - icm20948_gyro_bias.x;
  gyro_sensor_raw.y = (int32_t)icm20948_raw.gyro.y - icm20948_gyro_bias.y;
  gyro_sensor_raw.z = (int32_t)icm20948_raw.gyro.z - icm20948_gyro_bias.z;
  gyro_body_raw = Attitude_MapSensorRawToBody(gyro_sensor_raw);

  if (icm20948_mag_valid != 0U)
  {
    (void)snprintf(mag_report, sizeof(mag_report),
                   "mag_b_cal[c_uT]=%8ld,%8ld,%8ld",
                   (long)icm20948_mag_body_cal_cuT.x,
                   (long)icm20948_mag_body_cal_cuT.y,
                   (long)icm20948_mag_body_cal_cuT.z);
  }
  else
  {
    (void)snprintf(mag_report, sizeof(mag_report),
                   "mag_status=%2d",
                   (int)icm20948_mag_status);
  }

  length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                    "t=%10lums  who=0x%02X  accel_b_cal[mg]=%7ld,%7ld,%7ld"
                    "  gyro_b[mdps]=%8ld,%8ld,%8ld  temp[cC]=%8ld"
                    "  %-42s  rpy[cdeg]=%7ld,%7ld,%7ld\r\n",
                    (unsigned long)HAL_GetTick(),
                    icm20948_who_am_i,
                    (long)(accel_body_g.x * 1000.0f),
                    (long)(accel_body_g.y * 1000.0f),
                    (long)(accel_body_g.z * 1000.0f),
                    (long)App_GyroRawToMdps(gyro_body_raw.x),
                    (long)App_GyroRawToMdps(gyro_body_raw.y),
                    (long)App_GyroRawToMdps(gyro_body_raw.z),
                    (long)App_TempRawToCentiC(icm20948_raw.temperature),
                    mag_report,
                    (long)(attitude.roll * 100.0f),
                    (long)(attitude.pitch * 100.0f),
                    (long)(attitude.yaw * 100.0f));
  App_IcmLog(usb_tx_buffer, length);

  if ((icm20948_mag_raw.x == 0) &&
      (icm20948_mag_raw.y == 0) &&
      (icm20948_mag_raw.z == 0) &&
      ((HAL_GetTick() - app_last_mag_debug_ms) >= APP_MAG_DEBUG_PERIOD_MS))
  {
    app_last_mag_debug_ms = HAL_GetTick();
    App_ReportMagDebug();
  }
}

static void App_ReportMagDebug(void)
{
  ICM20948_MagDebug_t debug;
  ICM20948_Status_t status;
  int length;

  status = ICM20948_ReadMagDebug(&icm20948, &debug);
  if (status != ICM20948_OK)
  {
    length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                      "MAGDBG1 read_error=%d\r\n",
                      (int)status);
  }
  else
  {
    length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
                      "MAGDBG1 user=%02X lp=%02X pwr1=%02X pwr2=%02X intpin=%02X mst_status=%02X odr=%02X mst_ctrl=%02X delay=%02X slv0_addr=%02X slv0_reg=%02X slv0_ctrl=%02X shadow=%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X slv4_wia2=%02X slv4_status=%02X slv4_ctrl=%02X\r\n",
                      debug.user_ctrl,
                      debug.lp_config,
                      debug.pwr_mgmt_1,
                      debug.pwr_mgmt_2,
                      debug.int_pin_cfg,
                      debug.i2c_mst_status,
                      debug.i2c_mst_odr_config,
                      debug.i2c_mst_ctrl,
                      debug.i2c_mst_delay_ctrl,
                      debug.slv0_addr,
                      debug.slv0_reg,
                      debug.slv0_ctrl,
                      debug.shadow[0],
                      debug.shadow[1],
                      debug.shadow[2],
                      debug.shadow[3],
                      debug.shadow[4],
                      debug.shadow[5],
                      debug.shadow[6],
                      debug.shadow[7],
                      debug.shadow[8],
                      debug.slv4_wia2,
                      debug.slv4_status,
                      debug.slv4_ctrl_after);
  }

  App_IcmLog(usb_tx_buffer, length);
}

static void App_IcmLog(const char *text, int length)
{
#if APP_ICM20948_LOG_ENABLE
  App_UsbSend(text, length);
#else
  (void)text;
  (void)length;
#endif
}

#if APP_ICM20948_LOG_ENABLE
static void App_UsbSend(const char *text, int length)
{
  if ((app_usb_transmit == NULL) || (text == NULL) || (length <= 0))
  {
    return;
  }

  if (length >= (int)sizeof(usb_tx_buffer))
  {
    length = (int)sizeof(usb_tx_buffer) - 1;
  }

  (void)app_usb_transmit((uint8_t *)text, (uint16_t)length);
}
#endif

static int32_t App_GyroRawToMdps(int32_t raw)
{
  return (int32_t)(((float)raw * 1000.0f) /
                   APP_ICM20948_GYRO_LSB_PER_DPS);
}

static int32_t App_TempRawToCentiC(int16_t raw)
{
  return 2100L + (((int32_t)raw * 10000L) / 33387L);
}

static int32_t App_MagRawToCentiUt(int32_t raw)
{
  return raw * 15L;
}

static void App_EnableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR)
  DWT->LAR = 0xC5ACCE55UL;
#endif
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  app_cycle_counter_available =
      ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
}

static uint32_t App_CyclesToUs(uint64_t cycles)
{
  if (SystemCoreClock == 0U)
  {
    return 0U;
  }
  return (uint32_t)((cycles * 1000000ULL) / SystemCoreClock);
}

static void App_ResetTimingStats(void)
{
  ++app_timing_sequence;
  __DMB();
  app_timing_sample_count = 0U;
  app_timing_not_ready_count = 0U;
  app_timing_read_error_count = 0U;
  app_timing_period_count = 0U;
  app_timing_period_min_cycles = UINT32_MAX;
  app_timing_period_max_cycles = 0U;
  app_timing_period_total_cycles = 0U;
  app_timing_imu_read_min_cycles = UINT32_MAX;
  app_timing_imu_read_max_cycles = 0U;
  app_timing_imu_read_total_cycles = 0U;
  app_previous_pid_cycle = 0U;
  app_has_previous_pid_cycle = 0U;
  app_timing_pid_update_count = 0U;
  app_timing_pid_period_count = 0U;
  app_timing_pid_period_min_cycles = UINT32_MAX;
  app_timing_pid_period_max_cycles = 0U;
  app_timing_pid_period_total_cycles = 0U;
  app_timing_pid_exec_min_cycles = UINT32_MAX;
  app_timing_pid_exec_max_cycles = 0U;
  app_timing_pid_exec_total_cycles = 0U;
  app_timing_pipeline_min_cycles = UINT32_MAX;
  app_timing_pipeline_max_cycles = 0U;
  app_timing_pipeline_total_cycles = 0U;
  app_previous_profile_sample_cycle = 0U;
  app_has_previous_profile_sample = 0U;
  __DMB();
  ++app_timing_sequence;
}

static void App_RecordSampleTiming(uint32_t sample_cycle,
                                   uint32_t imu_read_cycles,
                                   uint32_t pipeline_cycles,
                                   bool pid_updated,
                                   uint32_t pid_exec_cycles)
{
  uint32_t period_cycles = 0U;

  ++app_timing_sequence;
  __DMB();
  if (app_has_previous_profile_sample != 0U)
  {
    period_cycles = sample_cycle - app_previous_profile_sample_cycle;
    if (period_cycles < app_timing_period_min_cycles)
    {
      app_timing_period_min_cycles = period_cycles;
    }
    if (period_cycles > app_timing_period_max_cycles)
    {
      app_timing_period_max_cycles = period_cycles;
    }
    app_timing_period_total_cycles += period_cycles;
    ++app_timing_period_count;
  }
  else
  {
    app_has_previous_profile_sample = 1U;
  }
  app_previous_profile_sample_cycle = sample_cycle;

  if (imu_read_cycles < app_timing_imu_read_min_cycles)
  {
    app_timing_imu_read_min_cycles = imu_read_cycles;
  }
  if (imu_read_cycles > app_timing_imu_read_max_cycles)
  {
    app_timing_imu_read_max_cycles = imu_read_cycles;
  }
  app_timing_imu_read_total_cycles += imu_read_cycles;

  if (pid_updated)
  {
    if (app_has_previous_pid_cycle != 0U)
    {
      const uint32_t pid_period_cycles =
          sample_cycle - app_previous_pid_cycle;
      if (pid_period_cycles < app_timing_pid_period_min_cycles)
      {
        app_timing_pid_period_min_cycles = pid_period_cycles;
      }
      if (pid_period_cycles > app_timing_pid_period_max_cycles)
      {
        app_timing_pid_period_max_cycles = pid_period_cycles;
      }
      app_timing_pid_period_total_cycles += pid_period_cycles;
      ++app_timing_pid_period_count;
    }
    else
    {
      app_has_previous_pid_cycle = 1U;
    }
    app_previous_pid_cycle = sample_cycle;

    if (pid_exec_cycles < app_timing_pid_exec_min_cycles)
    {
      app_timing_pid_exec_min_cycles = pid_exec_cycles;
    }
    if (pid_exec_cycles > app_timing_pid_exec_max_cycles)
    {
      app_timing_pid_exec_max_cycles = pid_exec_cycles;
    }
    app_timing_pid_exec_total_cycles += pid_exec_cycles;
    ++app_timing_pid_update_count;
  }
  else
  {
    /*
     * Do not include a disarmed/failsafe gap in the next PID period. The first
     * successful update after re-arm starts a fresh timing sequence.
     */
    app_has_previous_pid_cycle = 0U;
  }

  if (pipeline_cycles < app_timing_pipeline_min_cycles)
  {
    app_timing_pipeline_min_cycles = pipeline_cycles;
  }
  if (pipeline_cycles > app_timing_pipeline_max_cycles)
  {
    app_timing_pipeline_max_cycles = pipeline_cycles;
  }
  app_timing_pipeline_total_cycles += pipeline_cycles;
  ++app_timing_sample_count;
  __DMB();
  ++app_timing_sequence;
}
