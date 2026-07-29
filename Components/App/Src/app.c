#include "app.h"

#include "attitude.h"
#include "drone_control.h"
#include "icm20948.h"
#include "mahony9.h"
#include <stdio.h>

#define APP_ICM20948_SAMPLE_PERIOD_MS       10U
#define APP_ICM20948_REPORT_PERIOD_MS       20U
#define APP_ICM20948_MAG_PERIOD_MS          10U
#define APP_LED_BLINK_PERIOD_MS				1000U
#define APP_ICM20948_REINIT_PERIOD_MS       1000U
#define APP_MAG_DEBUG_PERIOD_MS             1000U
#define APP_GYRO_CALIBRATION_DISCARD_SAMPLES 200U
#define APP_GYRO_CALIBRATION_SAMPLES        1000U
#define APP_UART_LOG_BYTES_PER_LINE         16U
#define APP_USB_TX_OK                       0U
static SPI_HandleTypeDef *app_spi;
static GPIO_TypeDef *app_icm_cs_port;
static uint16_t app_icm_cs_pin;
static App_UsbTransmitFn app_usb_transmit;
static GPIO_TypeDef *app_activity_led_port;
static uint16_t app_activity_led_pin;
static TIM_HandleTypeDef *app_motor_timer;
static UART_HandleTypeDef *app_control_uart;
static volatile uint8_t app_uart_log_usb_busy;

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
static char uart_log_tx_buffer[80];

static void App_TryInitICM20948(void);
static void App_CalibrateGyro(void);
static void App_UpdateAttitude(void);
static void App_UpdateMagnetometer(void);
static void App_ReportICM20948(void);
static void App_ReportMagDebug(void);
static void App_IcmLog(const char *text, int length);
static void App_FlushControlUartLog(void);
#if APP_ICM20948_LOG_ENABLE
static void App_UsbSend(const char *text, int length);
#endif
static int32_t App_GyroRawToMdps(int32_t raw);
static int32_t App_TempRawToCentiC(int16_t raw);
static int32_t App_MagRawToCentiUt(int32_t raw);

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
  uint32_t now_ms;

  DroneControl_Init(app_control_uart, app_motor_timer);
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

  if (app_activity_led_port != NULL)
  {
    HAL_GPIO_TogglePin(app_activity_led_port, app_activity_led_pin);
  }
}

void App_Process(void)
{
  uint32_t now_ms = HAL_GetTick();

  DroneControl_Process(now_ms);
  App_FlushControlUartLog();

  if ((icm20948_status != ICM20948_OK) &&
      ((now_ms - icm20948_last_reinit_ms) >= APP_ICM20948_REINIT_PERIOD_MS))
  {
    icm20948_last_reinit_ms = now_ms;
    App_TryInitICM20948();
    App_CalibrateGyro();
  }

  if ((now_ms - icm20948_last_mag_ms) >= APP_ICM20948_MAG_PERIOD_MS)
  {
    icm20948_last_mag_ms = now_ms;
    App_UpdateMagnetometer();
  }

  if ((now_ms - icm20948_last_sample_ms) >= APP_ICM20948_SAMPLE_PERIOD_MS)
  {
    App_UpdateAttitude();
  }

  if ((now_ms - app_last_led_ms) >= APP_LED_BLINK_PERIOD_MS)
  {
    app_last_led_ms = now_ms;
    if (app_activity_led_port != NULL)
    {
      HAL_GPIO_TogglePin(app_activity_led_port, app_activity_led_pin);
    }
  }

  if ((APP_ICM20948_LOG_ENABLE != 0U) &&
      ((now_ms - icm20948_last_report_ms) >= APP_ICM20948_REPORT_PERIOD_MS))
  {
    icm20948_last_report_ms = now_ms;
    App_ReportICM20948();
  }
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

static void App_TryInitICM20948(void)
{
  uint8_t mag_wia2 = 0U;
  ICM20948_Status_t who_status;
  int length;

  if ((app_spi == NULL) || (app_icm_cs_port == NULL))
  {
    icm20948_status = ICM20948_ERROR;
    return;
  }

  icm20948_status = ICM20948_Init(&icm20948, app_spi, app_icm_cs_port, app_icm_cs_pin);
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
  uint32_t now_ms;
  uint32_t elapsed_ms;
  Attitude_VectorRaw_t accel_sensor_raw;
  Attitude_VectorRaw_t gyro_sensor_raw;
  Attitude_VectorRaw_t gyro_bias_sensor_raw;
  Attitude_Vector3f_t accel_body_g;
  Attitude_Vector3f_t gravity_body;
  Attitude_Vector3f_t gyro_body_rad_s;
  bool updated;
  float dt_s;

  if (icm20948_status != ICM20948_OK)
  {
    return;
  }

  now_ms = HAL_GetTick();
  elapsed_ms = now_ms - icm20948_last_sample_ms;
  if (mahony9.initialized && (elapsed_ms < APP_ICM20948_SAMPLE_PERIOD_MS))
  {
    return;
  }

  icm20948_status = ICM20948_ReadRaw(&icm20948, &icm20948_raw);
  icm20948_last_sample_ms = now_ms;
  if (icm20948_status != ICM20948_OK)
  {
    return;
  }

  accel_sensor_raw.x = icm20948_raw.accel.x;
  accel_sensor_raw.y = icm20948_raw.accel.y;
  accel_sensor_raw.z = icm20948_raw.accel.z;
  accel_body_g = Attitude_AccelRawToBodyG(accel_sensor_raw,
                                           ICM20948_ACCEL_2G_LSB_PER_G);
  gravity_body = Attitude_SpecificForceToGravity(accel_body_g);

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
    return;
  }

  dt_s = (float)elapsed_ms * 0.001f;
  if (dt_s > 0.05f)
  {
    dt_s = 0.05f;
  }

  gyro_sensor_raw.x = icm20948_raw.gyro.x;
  gyro_sensor_raw.y = icm20948_raw.gyro.y;
  gyro_sensor_raw.z = icm20948_raw.gyro.z;
  gyro_bias_sensor_raw.x = icm20948_gyro_bias.x;
  gyro_bias_sensor_raw.y = icm20948_gyro_bias.y;
  gyro_bias_sensor_raw.z = icm20948_gyro_bias.z;
  gyro_body_rad_s = Attitude_GyroRawToBodyRadS(gyro_sensor_raw,
                                                gyro_bias_sensor_raw,
                                                131.0f);

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
                                           ICM20948_ACCEL_2G_LSB_PER_G);

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
  return (raw * 1000L) / 131L;
}

static int32_t App_TempRawToCentiC(int16_t raw)
{
  return 2100L + (((int32_t)raw * 10000L) / 33387L);
}

static int32_t App_MagRawToCentiUt(int32_t raw)
{
  return raw * 15L;
}
