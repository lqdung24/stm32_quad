#include "icm20948.h"

#define ICM20948_SPI_READ_BIT          0x80U
#define ICM20948_SPI_WRITE_MASK        0x7FU
#define ICM20948_SPI_MAX_READ_LEN      24U
#define ICM20948_SPI_DUMMY_BYTE        0xFFU

#define ICM20948_REG_BANK_SEL          0x7FU

#define ICM20948_REG_WHO_AM_I          0x00U
#define ICM20948_REG_USER_CTRL         0x03U
#define ICM20948_REG_PWR_MGMT_1        0x06U
#define ICM20948_REG_PWR_MGMT_2        0x07U
#define ICM20948_REG_LP_CONFIG         0x05U
#define ICM20948_REG_INT_PIN_CFG       0x0FU
#define ICM20948_REG_I2C_MST_STATUS    0x17U
#define ICM20948_REG_INT_STATUS_1      0x1AU
#define ICM20948_REG_ACCEL_XOUT_H      0x2DU
#define ICM20948_REG_EXT_SLV_SENS_DATA_00 0x3BU

#define ICM20948_REG_GYRO_SMPLRT_DIV   0x00U
#define ICM20948_REG_GYRO_CONFIG_1     0x01U
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x10U
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x11U
#define ICM20948_REG_ACCEL_CONFIG      0x14U

#define ICM20948_REG_I2C_MST_CTRL      0x01U
#define ICM20948_REG_I2C_MST_ODR_CONFIG 0x00U
#define ICM20948_REG_I2C_MST_DELAY_CTRL 0x02U
#define ICM20948_REG_I2C_SLV0_ADDR     0x03U
#define ICM20948_REG_I2C_SLV0_REG      0x04U
#define ICM20948_REG_I2C_SLV0_CTRL     0x05U
#define ICM20948_REG_I2C_SLV0_DO       0x06U
#define ICM20948_REG_I2C_SLV1_CTRL     0x09U
#define ICM20948_REG_I2C_SLV2_CTRL     0x0DU
#define ICM20948_REG_I2C_SLV3_CTRL     0x11U
#define ICM20948_REG_I2C_SLV4_ADDR     0x13U
#define ICM20948_REG_I2C_SLV4_REG      0x14U
#define ICM20948_REG_I2C_SLV4_CTRL     0x15U
#define ICM20948_REG_I2C_SLV4_DI       0x17U

#define ICM20948_USER_CTRL_I2C_IF_DIS  0x10U
#define ICM20948_USER_CTRL_I2C_MST_EN  0x20U
#define ICM20948_PWR1_DEVICE_RESET     0x80U
#define ICM20948_PWR1_CLKSEL_AUTO      0x01U
#define ICM20948_DLPF_ENABLE           0x01U
#define ICM20948_RAW_DATA_READY        0x01U

#define ICM20948_I2C_SLV_READ_BIT      0x80U
#define ICM20948_I2C_SLV_EN            0x80U
#define ICM20948_I2C_SLV4_DONE         0x40U
#define ICM20948_I2C_SLV4_NACK         0x10U
#define ICM20948_I2C_MST_P_NSR         0x10U
#define ICM20948_I2C_MST_CLK_345KHZ    0x07U

#define AK09916_I2C_ADDR               0x0CU
#define AK09916_REG_WIA2               0x01U
#define AK09916_REG_ST1                0x10U
#define AK09916_REG_CNTL2              0x31U
#define AK09916_REG_CNTL3              0x32U
#define AK09916_ST2_HOFL               0x08U
#define AK09916_MODE_CONTINUOUS_100HZ  0x08U
#define AK09916_RESET                  0x01U

static ICM20948_Status_t ICM20948_SelectBank(ICM20948_Handle_t *dev, ICM20948_Bank_t bank);
static ICM20948_Status_t ICM20948_ReadRawRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
static ICM20948_Status_t ICM20948_WriteRawRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t value);
static ICM20948_Status_t ICM20948_MagReadRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *value);
static ICM20948_Status_t ICM20948_MagReadRegisters(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *data, uint8_t len);
static ICM20948_Status_t ICM20948_MagWriteRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t value);
static ICM20948_Status_t ICM20948_MagDebugReadSlave4(ICM20948_Handle_t *dev,
                                                       uint8_t reg,
                                                       uint8_t *value,
                                                       uint8_t *master_status,
                                                       uint8_t *ctrl_after);
static ICM20948_Status_t ICM20948_MagConfigureContinuousRead(ICM20948_Handle_t *dev);
static ICM20948_Status_t ICM20948_FromHalStatus(HAL_StatusTypeDef status);
static uint8_t ICM20948_BankValue(ICM20948_Bank_t bank);
static int16_t ICM20948_ToInt16(uint8_t msb, uint8_t lsb);
static float ICM20948_AccelSensitivity(ICM20948_AccelRange_t range);
static float ICM20948_GyroSensitivity(ICM20948_GyroRange_t range);

ICM20948_Status_t ICM20948_Init(ICM20948_Handle_t *dev,
                                SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *ncs_port,
                                uint16_t ncs_pin)
{
  uint8_t who_am_i = 0U;
  ICM20948_Status_t status;

  if ((dev == NULL) || (hspi == NULL) || (ncs_port == NULL) || (ncs_pin == 0U))
  {
    return ICM20948_ERROR_PARAM;
  }

  dev->hspi = hspi;
  dev->ncs_port = ncs_port;
  dev->ncs_pin = ncs_pin;
  dev->timeout_ms = ICM20948_DEFAULT_TIMEOUT_MS;
  dev->current_bank = ICM20948_BANK_UNKNOWN;
  dev->accel_range = ICM20948_ACCEL_RANGE_2G;
  dev->gyro_range = ICM20948_GYRO_RANGE_250DPS;

  HAL_GPIO_WritePin(dev->ncs_port, dev->ncs_pin, GPIO_PIN_SET);
  HAL_Delay(10U);

  status = ICM20948_ReadWhoAmI(dev, &who_am_i);
  if (status != ICM20948_OK)
  {
    return status;
  }

  /* WHO_AM_I is informational only. A different value must not stop init. */
  (void)who_am_i;

  /*
   * The MCU can reset while the still-powered ICM is streaming from AK09916.
   * Stop scheduling new AUX-I2C transfers and allow the current one to finish
   * before resetting the ICM. Abrupt I2C_MST_RST during an active transfer can
   * leave the auxiliary slave holding the bus until it is power-cycled.
   */
  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_CTRL, 0U);
  if (status != ICM20948_OK) return status;
  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV1_CTRL, 0U);
  if (status != ICM20948_OK) return status;
  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV2_CTRL, 0U);
  if (status != ICM20948_OK) return status;
  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV3_CTRL, 0U);
  if (status != ICM20948_OK) return status;
  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV4_CTRL, 0U);
  if (status != ICM20948_OK) return status;

  HAL_Delay(10U);

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_0, ICM20948_REG_PWR_MGMT_1, ICM20948_PWR1_DEVICE_RESET);
  if (status != ICM20948_OK)
  {
    return status;
  }

  HAL_Delay(100U);
  dev->current_bank = ICM20948_BANK_UNKNOWN;

  status = ICM20948_SelectBank(dev, ICM20948_BANK_0);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_0, ICM20948_REG_USER_CTRL, ICM20948_USER_CTRL_I2C_IF_DIS);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_0, ICM20948_REG_PWR_MGMT_1, ICM20948_PWR1_CLKSEL_AUTO);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_0, ICM20948_REG_PWR_MGMT_2, 0x00U);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_SetAccelRange(dev, ICM20948_ACCEL_RANGE_2G);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_SetGyroRange(dev, ICM20948_GYRO_RANGE_250DPS);
  if (status != ICM20948_OK)
  {
    return status;
  }

  return ICM20948_SelectBank(dev, ICM20948_BANK_0);
}

void ICM20948_SetTimeout(ICM20948_Handle_t *dev, uint32_t timeout_ms)
{
  if (dev != NULL)
  {
    dev->timeout_ms = timeout_ms;
  }
}

ICM20948_Status_t ICM20948_ReadWhoAmI(ICM20948_Handle_t *dev, uint8_t *who_am_i)
{
  return ICM20948_ReadRegister(dev, ICM20948_BANK_0, ICM20948_REG_WHO_AM_I, who_am_i);
}

ICM20948_Status_t ICM20948_SetAccelRange(ICM20948_Handle_t *dev, ICM20948_AccelRange_t range)
{
  uint8_t config;
  ICM20948_Status_t status;

  if ((dev == NULL) || (range > ICM20948_ACCEL_RANGE_16G))
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_ReadRegister(dev, ICM20948_BANK_2, ICM20948_REG_ACCEL_CONFIG, &config);
  if (status != ICM20948_OK)
  {
    return status;
  }

  config &= (uint8_t)~0x06U;
  config |= (uint8_t)(((uint8_t)range << 1U) | ICM20948_DLPF_ENABLE);

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_2, ICM20948_REG_ACCEL_CONFIG, config);
  if (status == ICM20948_OK)
  {
    dev->accel_range = range;
  }

  return status;
}

ICM20948_Status_t ICM20948_SetGyroRange(ICM20948_Handle_t *dev, ICM20948_GyroRange_t range)
{
  uint8_t config;
  ICM20948_Status_t status;

  if ((dev == NULL) || (range > ICM20948_GYRO_RANGE_2000DPS))
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_ReadRegister(dev, ICM20948_BANK_2, ICM20948_REG_GYRO_CONFIG_1, &config);
  if (status != ICM20948_OK)
  {
    return status;
  }

  config &= (uint8_t)~0x06U;
  config |= (uint8_t)(((uint8_t)range << 1U) | ICM20948_DLPF_ENABLE);

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_2, ICM20948_REG_GYRO_CONFIG_1, config);
  if (status == ICM20948_OK)
  {
    dev->gyro_range = range;
  }

  return status;
}

ICM20948_Status_t ICM20948_ConfigureMotion(
    ICM20948_Handle_t *dev,
    const ICM20948_MotionConfig_t *config)
{
  ICM20948_Status_t status;
  uint8_t accel_config;
  uint8_t gyro_config;

  if ((dev == NULL) || (config == NULL) ||
      (config->accel_range > ICM20948_ACCEL_RANGE_16G) ||
      (config->gyro_range > ICM20948_GYRO_RANGE_2000DPS) ||
      (config->accel_sample_rate_divider > 4095U) ||
      (config->accel_dlpf_config > 7U) ||
      (config->gyro_dlpf_config > 7U))
  {
    return ICM20948_ERROR_PARAM;
  }

  accel_config =
      (uint8_t)((config->accel_dlpf_config << 3U) |
                ((uint8_t)config->accel_range << 1U) |
                ICM20948_DLPF_ENABLE);
  gyro_config =
      (uint8_t)((config->gyro_dlpf_config << 3U) |
                ((uint8_t)config->gyro_range << 1U) |
                ICM20948_DLPF_ENABLE);

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_2,
                                  ICM20948_REG_GYRO_SMPLRT_DIV,
                                  config->gyro_sample_rate_divider);
  if (status != ICM20948_OK) return status;

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_2,
                                  ICM20948_REG_GYRO_CONFIG_1,
                                  gyro_config);
  if (status != ICM20948_OK) return status;

  status = ICM20948_WriteRegister(
      dev,
      ICM20948_BANK_2,
      ICM20948_REG_ACCEL_SMPLRT_DIV_1,
      (uint8_t)((config->accel_sample_rate_divider >> 8U) & 0x0FU));
  if (status != ICM20948_OK) return status;

  status = ICM20948_WriteRegister(
      dev,
      ICM20948_BANK_2,
      ICM20948_REG_ACCEL_SMPLRT_DIV_2,
      (uint8_t)config->accel_sample_rate_divider);
  if (status != ICM20948_OK) return status;

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_2,
                                  ICM20948_REG_ACCEL_CONFIG,
                                  accel_config);
  if (status != ICM20948_OK) return status;

  dev->accel_range = config->accel_range;
  dev->gyro_range = config->gyro_range;
  return ICM20948_SelectBank(dev, ICM20948_BANK_0);
}

ICM20948_Status_t ICM20948_IsRawDataReady(ICM20948_Handle_t *dev,
                                          uint8_t *ready)
{
  ICM20948_Status_t status;
  uint8_t value;

  if ((dev == NULL) || (ready == NULL))
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_ReadRegister(dev,
                                 ICM20948_BANK_0,
                                 ICM20948_REG_INT_STATUS_1,
                                 &value);
  if (status == ICM20948_OK)
  {
    *ready = ((value & ICM20948_RAW_DATA_READY) != 0U) ? 1U : 0U;
  }
  return status;
}

ICM20948_Status_t ICM20948_ReadRaw(ICM20948_Handle_t *dev, ICM20948_RawData_t *data)
{
  uint8_t buffer[14];
  ICM20948_Status_t status;

  if ((dev == NULL) || (data == NULL))
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_ReadRegisters(dev, ICM20948_BANK_0, ICM20948_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));
  if (status != ICM20948_OK)
  {
    return status;
  }

  data->accel.x = ICM20948_ToInt16(buffer[0], buffer[1]);
  data->accel.y = ICM20948_ToInt16(buffer[2], buffer[3]);
  data->accel.z = ICM20948_ToInt16(buffer[4], buffer[5]);
  data->gyro.x = ICM20948_ToInt16(buffer[6], buffer[7]);
  data->gyro.y = ICM20948_ToInt16(buffer[8], buffer[9]);
  data->gyro.z = ICM20948_ToInt16(buffer[10], buffer[11]);
  data->temperature = ICM20948_ToInt16(buffer[12], buffer[13]);

  return ICM20948_OK;
}

ICM20948_Status_t ICM20948_InitMagnetometer(ICM20948_Handle_t *dev, uint8_t *wia2)
{
  ICM20948_Status_t status;
  uint8_t mag_id = 0U;
  uint8_t mag_mode = 0U;

  if (dev == NULL)
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_0,
                                  ICM20948_REG_USER_CTRL,
                                  ICM20948_USER_CTRL_I2C_MST_EN | ICM20948_USER_CTRL_I2C_IF_DIS);
  if (status != ICM20948_OK)
  {
    return status;
  }

  HAL_Delay(20U);

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_0, ICM20948_REG_LP_CONFIG, 0x40U);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_MST_ODR_CONFIG, 0x00U);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_MST_CTRL,
                                  ICM20948_I2C_MST_P_NSR | ICM20948_I2C_MST_CLK_345KHZ);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_MagReadRegister(dev, AK09916_REG_WIA2, &mag_id);
  if (status != ICM20948_OK)
  {
    return status;
  }

  if (wia2 != NULL)
  {
    *wia2 = mag_id;
  }

  if (mag_id != ICM20948_MAG_WIA2_VALUE)
  {
    return ICM20948_ERROR;
  }

  status = ICM20948_MagWriteRegister(dev, AK09916_REG_CNTL3, AK09916_RESET);
  if (status != ICM20948_OK)
  {
    return status;
  }

  HAL_Delay(10U);

  status = ICM20948_MagWriteRegister(dev, AK09916_REG_CNTL2, AK09916_MODE_CONTINUOUS_100HZ);
  if (status != ICM20948_OK)
  {
    return status;
  }

  HAL_Delay(10U);

  status = ICM20948_MagReadRegister(dev, AK09916_REG_CNTL2, &mag_mode);
  if ((status != ICM20948_OK) || (mag_mode != AK09916_MODE_CONTINUOUS_100HZ))
  {
    return ICM20948_ERROR;
  }

  return ICM20948_MagConfigureContinuousRead(dev);
}

ICM20948_Status_t ICM20948_ReadMagRaw(ICM20948_Handle_t *dev, ICM20948_VectorRaw_t *mag)
{
  uint8_t buffer[9];
  uint8_t attempt;
  ICM20948_Status_t status;

  if ((dev == NULL) || (mag == NULL))
  {
    return ICM20948_ERROR_PARAM;
  }

  for (attempt = 0U; attempt < 3U; attempt++)
  {
    status = ICM20948_ReadRegisters(dev,
                                    ICM20948_BANK_0,
                                    ICM20948_REG_EXT_SLV_SENS_DATA_00,
                                    buffer,
                                    sizeof(buffer));
    if (status != ICM20948_OK)
    {
      return status;
    }

    mag->x = ICM20948_ToInt16(buffer[2], buffer[1]);
    mag->y = ICM20948_ToInt16(buffer[4], buffer[3]);
    mag->z = ICM20948_ToInt16(buffer[6], buffer[5]);

    if ((buffer[8] & AK09916_ST2_HOFL) != 0U)
    {
      return ICM20948_ERROR_MAG_NOT_READY;
    }

    if ((mag->x != 0) || (mag->y != 0) || (mag->z != 0))
    {
      return ICM20948_OK;
    }

    HAL_Delay(1U);
  }

  return ICM20948_ERROR_MAG_NOT_READY;
}

ICM20948_Status_t ICM20948_ReadMagDebug(ICM20948_Handle_t *dev, ICM20948_MagDebug_t *debug)
{
  ICM20948_Status_t status;

  if ((dev == NULL) || (debug == NULL))
  {
    return ICM20948_ERROR_PARAM;
  }

#define ICM20948_MAG_DEBUG_READ(bank_, reg_, dst_)              \
  do                                                            \
  {                                                             \
    status = ICM20948_ReadRegister(dev, (bank_), (reg_), (dst_)); \
    if (status != ICM20948_OK)                                  \
    {                                                           \
      return status;                                            \
    }                                                           \
  } while (0)

  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_0, ICM20948_REG_USER_CTRL, &debug->user_ctrl);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_0, ICM20948_REG_LP_CONFIG, &debug->lp_config);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_0, ICM20948_REG_PWR_MGMT_1, &debug->pwr_mgmt_1);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_0, ICM20948_REG_PWR_MGMT_2, &debug->pwr_mgmt_2);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_0, ICM20948_REG_INT_PIN_CFG, &debug->int_pin_cfg);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_0, ICM20948_REG_I2C_MST_STATUS, &debug->i2c_mst_status);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_3, ICM20948_REG_I2C_MST_ODR_CONFIG, &debug->i2c_mst_odr_config);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_3, ICM20948_REG_I2C_MST_CTRL, &debug->i2c_mst_ctrl);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_3, ICM20948_REG_I2C_MST_DELAY_CTRL, &debug->i2c_mst_delay_ctrl);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_ADDR, &debug->slv0_addr);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_REG, &debug->slv0_reg);
  ICM20948_MAG_DEBUG_READ(ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_CTRL, &debug->slv0_ctrl);

#undef ICM20948_MAG_DEBUG_READ

  status = ICM20948_ReadRegisters(dev,
                                  ICM20948_BANK_0,
                                  ICM20948_REG_EXT_SLV_SENS_DATA_00,
                                  debug->shadow,
                                  sizeof(debug->shadow));
  if (status != ICM20948_OK)
  {
    return status;
  }

  /*
   * A one-shot SLV4 read is independent of SLV0's shadow stream.  Keep this
   * diagnostic here so a zero SLV0 shadow can be distinguished from an AUX
   * I2C/AK09916 communication failure.
   */
  return ICM20948_MagDebugReadSlave4(dev,
                                     AK09916_REG_WIA2,
                                     &debug->slv4_wia2,
                                     &debug->slv4_status,
                                     &debug->slv4_ctrl_after);
}

ICM20948_Status_t ICM20948_ReadScaled(ICM20948_Handle_t *dev, ICM20948_Data_t *data)
{
  ICM20948_RawData_t raw;
  ICM20948_Status_t status;
  float accel_sensitivity;
  float gyro_sensitivity;

  if ((dev == NULL) || (data == NULL))
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_ReadRaw(dev, &raw);
  if (status != ICM20948_OK)
  {
    return status;
  }

  accel_sensitivity = ICM20948_AccelSensitivity(dev->accel_range);
  gyro_sensitivity = ICM20948_GyroSensitivity(dev->gyro_range);

  data->accel_g.x = (float)raw.accel.x / accel_sensitivity;
  data->accel_g.y = (float)raw.accel.y / accel_sensitivity;
  data->accel_g.z = (float)raw.accel.z / accel_sensitivity;
  data->gyro_dps.x = (float)raw.gyro.x / gyro_sensitivity;
  data->gyro_dps.y = (float)raw.gyro.y / gyro_sensitivity;
  data->gyro_dps.z = (float)raw.gyro.z / gyro_sensitivity;
  data->temperature_c = ((float)raw.temperature / 333.87f) + 21.0f;

  return ICM20948_OK;
}

ICM20948_Status_t ICM20948_ReadRegister(ICM20948_Handle_t *dev,
                                        ICM20948_Bank_t bank,
                                        uint8_t reg,
                                        uint8_t *value)
{
  if (value == NULL)
  {
    return ICM20948_ERROR_PARAM;
  }

  return ICM20948_ReadRegisters(dev, bank, reg, value, 1U);
}

ICM20948_Status_t ICM20948_ReadRegisters(ICM20948_Handle_t *dev,
                                         ICM20948_Bank_t bank,
                                         uint8_t reg,
                                         uint8_t *data,
                                         uint16_t len)
{
  ICM20948_Status_t status;

  if ((dev == NULL) || (data == NULL) || (len == 0U) || (bank > ICM20948_BANK_3))
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_SelectBank(dev, bank);
  if (status != ICM20948_OK)
  {
    return status;
  }

  return ICM20948_ReadRawRegister(dev, reg, data, len);
}

ICM20948_Status_t ICM20948_WriteRegister(ICM20948_Handle_t *dev,
                                         ICM20948_Bank_t bank,
                                         uint8_t reg,
                                         uint8_t value)
{
  ICM20948_Status_t status;

  if ((dev == NULL) || (bank > ICM20948_BANK_3))
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_SelectBank(dev, bank);
  if (status != ICM20948_OK)
  {
    return status;
  }

  return ICM20948_WriteRawRegister(dev, reg, value);
}

static ICM20948_Status_t ICM20948_SelectBank(ICM20948_Handle_t *dev, ICM20948_Bank_t bank)
{
  ICM20948_Status_t status;

  if ((dev == NULL) || (bank > ICM20948_BANK_3))
  {
    return ICM20948_ERROR_PARAM;
  }

  /* Always select the bank so a missed bank write cannot leave clone parts
     permanently reading the wrong register map. */
  status = ICM20948_WriteRawRegister(dev, ICM20948_REG_BANK_SEL, ICM20948_BankValue(bank));
  if (status == ICM20948_OK)
  {
    dev->current_bank = bank;
  }

  return status;
}

static ICM20948_Status_t ICM20948_ReadRawRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t tx_buffer[ICM20948_SPI_MAX_READ_LEN + 1U];
  uint8_t rx_buffer[ICM20948_SPI_MAX_READ_LEN + 1U];
  ICM20948_Status_t status;
  uint16_t i;

  if ((dev == NULL) || (dev->hspi == NULL) || (dev->ncs_port == NULL) ||
      (data == NULL) || (len == 0U) || (len > ICM20948_SPI_MAX_READ_LEN))
  {
    return ICM20948_ERROR_PARAM;
  }

  tx_buffer[0] = reg | ICM20948_SPI_READ_BIT;
  for (i = 1U; i <= len; i++)
  {
    tx_buffer[i] = ICM20948_SPI_DUMMY_BYTE;
  }

  HAL_GPIO_WritePin(dev->ncs_port, dev->ncs_pin, GPIO_PIN_RESET);
  status = ICM20948_FromHalStatus(HAL_SPI_TransmitReceive(dev->hspi,
                                                          tx_buffer,
                                                          rx_buffer,
                                                          len + 1U,
                                                          dev->timeout_ms));
  HAL_GPIO_WritePin(dev->ncs_port, dev->ncs_pin, GPIO_PIN_SET);

  if (status == ICM20948_OK)
  {
    for (i = 0U; i < len; i++)
    {
      data[i] = rx_buffer[i + 1U];
    }
  }

  return status;
}

static ICM20948_Status_t ICM20948_WriteRawRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t value)
{
  uint8_t buffer[2];
  ICM20948_Status_t status;

  if ((dev == NULL) || (dev->hspi == NULL) || (dev->ncs_port == NULL))
  {
    return ICM20948_ERROR_PARAM;
  }

  buffer[0] = reg & ICM20948_SPI_WRITE_MASK;
  buffer[1] = value;

  HAL_GPIO_WritePin(dev->ncs_port, dev->ncs_pin, GPIO_PIN_RESET);
  status = ICM20948_FromHalStatus(HAL_SPI_Transmit(dev->hspi, buffer, sizeof(buffer), dev->timeout_ms));
  HAL_GPIO_WritePin(dev->ncs_port, dev->ncs_pin, GPIO_PIN_SET);

  return status;
}

static ICM20948_Status_t ICM20948_MagReadRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *value)
{
  return ICM20948_MagReadRegisters(dev, reg, value, 1U);
}

static ICM20948_Status_t ICM20948_MagReadRegisters(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
  ICM20948_Status_t status;

  if ((dev == NULL) || (data == NULL) || (len == 0U) || (len > 24U))
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_ADDR,
                                  AK09916_I2C_ADDR | ICM20948_I2C_SLV_READ_BIT);
  if (status != ICM20948_OK) return status;
  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_REG, reg);
  if (status != ICM20948_OK) return status;
  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_CTRL,
                                  ICM20948_I2C_SLV_EN | len);
  if (status != ICM20948_OK) return status;

  HAL_Delay(10U);
  return ICM20948_ReadRegisters(dev, ICM20948_BANK_0,
                                ICM20948_REG_EXT_SLV_SENS_DATA_00, data, len);
}

static ICM20948_Status_t ICM20948_MagWriteRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t value)
{
  ICM20948_Status_t status;

  if (dev == NULL)
  {
    return ICM20948_ERROR_PARAM;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_REG, reg);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_DO, value);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_CTRL,
                                  ICM20948_I2C_SLV_EN | 1U);
  if (status != ICM20948_OK)
  {
    return status;
  }

  HAL_Delay(10U);
  return ICM20948_WriteRegister(dev, ICM20948_BANK_3, ICM20948_REG_I2C_SLV0_CTRL, 0U);
}

static ICM20948_Status_t ICM20948_MagDebugReadSlave4(ICM20948_Handle_t *dev,
                                                       uint8_t reg,
                                                       uint8_t *value,
                                                       uint8_t *master_status,
                                                       uint8_t *ctrl_after)
{
  ICM20948_Status_t status;
  uint8_t slv0_ctrl;
  uint8_t discarded_status;
  uint8_t attempt;

  if ((dev == NULL) || (value == NULL) || (master_status == NULL) || (ctrl_after == NULL))
  {
    return ICM20948_ERROR_PARAM;
  }

  /* SLV4 is serviced after SLV0..3. Temporarily stop the failed SLV0 stream
     so this test measures SLV4 and the shared auxiliary bus independently. */
  status = ICM20948_ReadRegister(dev,
                                 ICM20948_BANK_3,
                                 ICM20948_REG_I2C_SLV0_CTRL,
                                 &slv0_ctrl);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_SLV0_CTRL,
                                  0U);
  if (status != ICM20948_OK)
  {
    return status;
  }

  /* Clear only status left by transactions which happened before SLV4. */
  status = ICM20948_ReadRegister(dev,
                                 ICM20948_BANK_0,
                                 ICM20948_REG_I2C_MST_STATUS,
                                 &discarded_status);
  if (status != ICM20948_OK)
  {
    return status;
  }

  /* Configure and trigger one direct auxiliary-I2C read. */
  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_SLV4_ADDR,
                                  AK09916_I2C_ADDR | ICM20948_I2C_SLV_READ_BIT);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_SLV4_REG,
                                  reg);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_SLV4_CTRL,
                                  ICM20948_I2C_SLV_EN);
  if (status != ICM20948_OK)
  {
    return status;
  }

  /* EN is cleared by hardware after the one-byte transaction. Polling it does
     not clear I2C_MST_STATUS and tells us whether the scheduler ran at all. */
  *ctrl_after = ICM20948_I2C_SLV_EN;
  for (attempt = 0U; attempt < 50U; attempt++)
  {
    HAL_Delay(1U);
    status = ICM20948_ReadRegister(dev,
                                   ICM20948_BANK_3,
                                   ICM20948_REG_I2C_SLV4_CTRL,
                                   ctrl_after);
    if (status != ICM20948_OK)
    {
      return status;
    }

    if ((*ctrl_after & ICM20948_I2C_SLV_EN) == 0U)
    {
      break;
    }
  }

  /* I2C_MST_STATUS is read-to-clear, so capture it before leaving debug. */
  status = ICM20948_ReadRegister(dev,
                                 ICM20948_BANK_0,
                                 ICM20948_REG_I2C_MST_STATUS,
                                 master_status);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_ReadRegister(dev,
                                 ICM20948_BANK_3,
                                 ICM20948_REG_I2C_SLV4_DI,
                                 value);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_SLV4_CTRL,
                                  0U);
  if (status != ICM20948_OK)
  {
    return status;
  }

  return ICM20948_WriteRegister(dev,
                                ICM20948_BANK_3,
                                ICM20948_REG_I2C_SLV0_CTRL,
                                slv0_ctrl);
}

static ICM20948_Status_t ICM20948_MagConfigureContinuousRead(ICM20948_Handle_t *dev)
{
  ICM20948_Status_t status;

  /*
   * Set this explicitly instead of relying on its reset value. Earlier
   * firmware enabled DELAY_ES_SHADOW, which can leave EXT_SLV_SENS_DATA
   * unpublished on this setup until the ICM is fully power-cycled.
   */
  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_MST_DELAY_CTRL,
                                  0x00U);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_SLV0_ADDR,
                                  AK09916_I2C_ADDR | ICM20948_I2C_SLV_READ_BIT);
  if (status != ICM20948_OK)
  {
    return status;
  }

  status = ICM20948_WriteRegister(dev,
                                  ICM20948_BANK_3,
                                  ICM20948_REG_I2C_SLV0_REG,
                                  AK09916_REG_ST1);
  if (status != ICM20948_OK)
  {
    return status;
  }

  return ICM20948_WriteRegister(dev,
                                ICM20948_BANK_3,
                                ICM20948_REG_I2C_SLV0_CTRL,
                                ICM20948_I2C_SLV_EN | 9U);
}

static ICM20948_Status_t ICM20948_FromHalStatus(HAL_StatusTypeDef status)
{
  if (status == HAL_OK)
  {
    return ICM20948_OK;
  }

  return ICM20948_ERROR_SPI;
}

static uint8_t ICM20948_BankValue(ICM20948_Bank_t bank)
{
  return (uint8_t)((uint8_t)bank << 4U);
}

static int16_t ICM20948_ToInt16(uint8_t msb, uint8_t lsb)
{
  return (int16_t)(((uint16_t)msb << 8U) | (uint16_t)lsb);
}

static float ICM20948_AccelSensitivity(ICM20948_AccelRange_t range)
{
  switch (range)
  {
    case ICM20948_ACCEL_RANGE_4G:
      return ICM20948_ACCEL_2G_LSB_PER_G / 2.0f;
    case ICM20948_ACCEL_RANGE_8G:
      return ICM20948_ACCEL_2G_LSB_PER_G / 4.0f;
    case ICM20948_ACCEL_RANGE_16G:
      return ICM20948_ACCEL_2G_LSB_PER_G / 8.0f;
    case ICM20948_ACCEL_RANGE_2G:
    default:
      return ICM20948_ACCEL_2G_LSB_PER_G;
  }
}

static float ICM20948_GyroSensitivity(ICM20948_GyroRange_t range)
{
  switch (range)
  {
    case ICM20948_GYRO_RANGE_500DPS:
      return 65.5f;
    case ICM20948_GYRO_RANGE_1000DPS:
      return 32.8f;
    case ICM20948_GYRO_RANGE_2000DPS:
      return 16.4f;
    case ICM20948_GYRO_RANGE_250DPS:
    default:
      return 131.0f;
  }
}
