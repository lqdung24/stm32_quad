#ifndef ICM20948_H
#define ICM20948_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define ICM20948_WHO_AM_I_VALUE        0xEAU
#define ICM20948_MAG_WIA2_VALUE        0x09U
#define ICM20948_DEFAULT_TIMEOUT_MS    100U

#ifndef ICM20948_ACCEL_2G_LSB_PER_G
#define ICM20948_ACCEL_2G_LSB_PER_G    16384.0f
#endif

typedef enum
{
  ICM20948_OK = 0,
  ICM20948_ERROR,
  ICM20948_ERROR_PARAM,
  ICM20948_ERROR_SPI,
  ICM20948_ERROR_WHO_AM_I,
  ICM20948_ERROR_MAG_NOT_READY
} ICM20948_Status_t;

typedef enum
{
  ICM20948_BANK_0 = 0,
  ICM20948_BANK_1 = 1,
  ICM20948_BANK_2 = 2,
  ICM20948_BANK_3 = 3,
  ICM20948_BANK_UNKNOWN = 0xFF
} ICM20948_Bank_t;

typedef enum
{
  ICM20948_ACCEL_RANGE_2G = 0,
  ICM20948_ACCEL_RANGE_4G = 1,
  ICM20948_ACCEL_RANGE_8G = 2,
  ICM20948_ACCEL_RANGE_16G = 3
} ICM20948_AccelRange_t;

typedef enum
{
  ICM20948_GYRO_RANGE_250DPS = 0,
  ICM20948_GYRO_RANGE_500DPS = 1,
  ICM20948_GYRO_RANGE_1000DPS = 2,
  ICM20948_GYRO_RANGE_2000DPS = 3
} ICM20948_GyroRange_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} ICM20948_VectorRaw_t;

typedef struct
{
  float x;
  float y;
  float z;
} ICM20948_Vector_t;

typedef struct
{
  ICM20948_VectorRaw_t accel;
  ICM20948_VectorRaw_t gyro;
  int16_t temperature;
} ICM20948_RawData_t;

typedef struct
{
  ICM20948_Vector_t accel_g;
  ICM20948_Vector_t gyro_dps;
  float temperature_c;
} ICM20948_Data_t;

typedef struct
{
  uint8_t user_ctrl;
  uint8_t lp_config;
  uint8_t pwr_mgmt_1;
  uint8_t pwr_mgmt_2;
  uint8_t int_pin_cfg;
  uint8_t i2c_mst_status;
  uint8_t i2c_mst_odr_config;
  uint8_t i2c_mst_ctrl;
  uint8_t i2c_mst_delay_ctrl;
  uint8_t slv0_addr;
  uint8_t slv0_reg;
  uint8_t slv0_ctrl;
  uint8_t slv4_wia2;
  uint8_t slv4_status;
  uint8_t slv4_ctrl_after;
  uint8_t shadow[9];
} ICM20948_MagDebug_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *ncs_port;
  uint16_t ncs_pin;
  uint32_t timeout_ms;
  ICM20948_Bank_t current_bank;
  ICM20948_AccelRange_t accel_range;
  ICM20948_GyroRange_t gyro_range;
} ICM20948_Handle_t;

ICM20948_Status_t ICM20948_Init(ICM20948_Handle_t *dev,
                                SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *ncs_port,
                                uint16_t ncs_pin);

void ICM20948_SetTimeout(ICM20948_Handle_t *dev, uint32_t timeout_ms);

ICM20948_Status_t ICM20948_ReadWhoAmI(ICM20948_Handle_t *dev, uint8_t *who_am_i);
ICM20948_Status_t ICM20948_SetAccelRange(ICM20948_Handle_t *dev, ICM20948_AccelRange_t range);
ICM20948_Status_t ICM20948_SetGyroRange(ICM20948_Handle_t *dev, ICM20948_GyroRange_t range);
ICM20948_Status_t ICM20948_ReadRaw(ICM20948_Handle_t *dev, ICM20948_RawData_t *data);
ICM20948_Status_t ICM20948_ReadScaled(ICM20948_Handle_t *dev, ICM20948_Data_t *data);
ICM20948_Status_t ICM20948_InitMagnetometer(ICM20948_Handle_t *dev, uint8_t *wia2);
ICM20948_Status_t ICM20948_ReadMagRaw(ICM20948_Handle_t *dev, ICM20948_VectorRaw_t *mag);
ICM20948_Status_t ICM20948_ReadMagDebug(ICM20948_Handle_t *dev, ICM20948_MagDebug_t *debug);

ICM20948_Status_t ICM20948_ReadRegister(ICM20948_Handle_t *dev,
                                        ICM20948_Bank_t bank,
                                        uint8_t reg,
                                        uint8_t *value);
ICM20948_Status_t ICM20948_ReadRegisters(ICM20948_Handle_t *dev,
                                         ICM20948_Bank_t bank,
                                         uint8_t reg,
                                         uint8_t *data,
                                         uint16_t len);
ICM20948_Status_t ICM20948_WriteRegister(ICM20948_Handle_t *dev,
                                         ICM20948_Bank_t bank,
                                         uint8_t reg,
                                         uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_H */
