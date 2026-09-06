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
  icm20948.h
Src/
  icm20948.c
todo.md
```

# Files

## File: Inc/icm20948.h
```c
} ICM20948_Status_t;
⋮----
} ICM20948_Bank_t;
⋮----
} ICM20948_AccelRange_t;
⋮----
} ICM20948_GyroRange_t;
⋮----
} ICM20948_VectorRaw_t;
⋮----
} ICM20948_Vector_t;
⋮----
} ICM20948_RawData_t;
⋮----
} ICM20948_Data_t;
⋮----
} ICM20948_MagDebug_t;
⋮----
} ICM20948_Handle_t;
⋮----
} ICM20948_MotionConfig_t;
⋮----
ICM20948_Status_t ICM20948_Init(ICM20948_Handle_t *dev,
⋮----
void ICM20948_SetTimeout(ICM20948_Handle_t *dev, uint32_t timeout_ms);
⋮----
ICM20948_Status_t ICM20948_ReadWhoAmI(ICM20948_Handle_t *dev, uint8_t *who_am_i);
ICM20948_Status_t ICM20948_SetAccelRange(ICM20948_Handle_t *dev, ICM20948_AccelRange_t range);
ICM20948_Status_t ICM20948_SetGyroRange(ICM20948_Handle_t *dev, ICM20948_GyroRange_t range);
ICM20948_Status_t ICM20948_ConfigureMotion(
⋮----
ICM20948_Status_t ICM20948_IsRawDataReady(ICM20948_Handle_t *dev,
⋮----
ICM20948_Status_t ICM20948_ReadRaw(ICM20948_Handle_t *dev, ICM20948_RawData_t *data);
ICM20948_Status_t ICM20948_ReadScaled(ICM20948_Handle_t *dev, ICM20948_Data_t *data);
ICM20948_Status_t ICM20948_InitMagnetometer(ICM20948_Handle_t *dev, uint8_t *wia2);
ICM20948_Status_t ICM20948_ReadMagRaw(ICM20948_Handle_t *dev, ICM20948_VectorRaw_t *mag);
ICM20948_Status_t ICM20948_ReadMagDebug(ICM20948_Handle_t *dev, ICM20948_MagDebug_t *debug);
⋮----
ICM20948_Status_t ICM20948_ReadRegister(ICM20948_Handle_t *dev,
⋮----
ICM20948_Status_t ICM20948_ReadRegisters(ICM20948_Handle_t *dev,
⋮----
ICM20948_Status_t ICM20948_WriteRegister(ICM20948_Handle_t *dev,
⋮----
#endif /* ICM20948_H */
```

## File: Src/icm20948.c
```c
static ICM20948_Status_t ICM20948_SelectBank(ICM20948_Handle_t *dev, ICM20948_Bank_t bank);
static ICM20948_Status_t ICM20948_ReadRawRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
static ICM20948_Status_t ICM20948_WriteRawRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t value);
static ICM20948_Status_t ICM20948_MagReadRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *value);
static ICM20948_Status_t ICM20948_MagReadRegisters(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *data, uint8_t len);
static ICM20948_Status_t ICM20948_MagWriteRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t value);
static ICM20948_Status_t ICM20948_MagDebugReadSlave4(ICM20948_Handle_t *dev,
⋮----
static ICM20948_Status_t ICM20948_MagConfigureContinuousRead(ICM20948_Handle_t *dev);
static ICM20948_Status_t ICM20948_FromHalStatus(HAL_StatusTypeDef status);
static uint8_t ICM20948_BankValue(ICM20948_Bank_t bank);
static int16_t ICM20948_ToInt16(uint8_t msb, uint8_t lsb);
static float ICM20948_AccelSensitivity(ICM20948_AccelRange_t range);
static float ICM20948_GyroSensitivity(ICM20948_GyroRange_t range);
⋮----
ICM20948_Status_t ICM20948_Init(ICM20948_Handle_t *dev,
⋮----
/* WHO_AM_I is informational only. A different value must not stop init. */
⋮----
/*
   * The MCU can reset while the still-powered ICM is streaming from AK09916.
   * Stop scheduling new AUX-I2C transfers and allow the current one to finish
   * before resetting the ICM. Abrupt I2C_MST_RST during an active transfer can
   * leave the auxiliary slave holding the bus until it is power-cycled.
   */
⋮----
void ICM20948_SetTimeout(ICM20948_Handle_t *dev, uint32_t timeout_ms)
⋮----
ICM20948_Status_t ICM20948_ReadWhoAmI(ICM20948_Handle_t *dev, uint8_t *who_am_i)
⋮----
ICM20948_Status_t ICM20948_SetAccelRange(ICM20948_Handle_t *dev, ICM20948_AccelRange_t range)
⋮----
ICM20948_Status_t ICM20948_SetGyroRange(ICM20948_Handle_t *dev, ICM20948_GyroRange_t range)
⋮----
ICM20948_Status_t ICM20948_ConfigureMotion(
⋮----
ICM20948_Status_t ICM20948_IsRawDataReady(ICM20948_Handle_t *dev,
⋮----
ICM20948_Status_t ICM20948_ReadRaw(ICM20948_Handle_t *dev, ICM20948_RawData_t *data)
⋮----
ICM20948_Status_t ICM20948_InitMagnetometer(ICM20948_Handle_t *dev, uint8_t *wia2)
⋮----
ICM20948_Status_t ICM20948_ReadMagRaw(ICM20948_Handle_t *dev, ICM20948_VectorRaw_t *mag)
⋮----
ICM20948_Status_t ICM20948_ReadMagDebug(ICM20948_Handle_t *dev, ICM20948_MagDebug_t *debug)
⋮----
/*
   * A one-shot SLV4 read is independent of SLV0's shadow stream.  Keep this
   * diagnostic here so a zero SLV0 shadow can be distinguished from an AUX
   * I2C/AK09916 communication failure.
   */
⋮----
ICM20948_Status_t ICM20948_ReadScaled(ICM20948_Handle_t *dev, ICM20948_Data_t *data)
⋮----
ICM20948_Status_t ICM20948_ReadRegister(ICM20948_Handle_t *dev,
⋮----
ICM20948_Status_t ICM20948_ReadRegisters(ICM20948_Handle_t *dev,
⋮----
ICM20948_Status_t ICM20948_WriteRegister(ICM20948_Handle_t *dev,
⋮----
static ICM20948_Status_t ICM20948_SelectBank(ICM20948_Handle_t *dev, ICM20948_Bank_t bank)
⋮----
/* Always select the bank so a missed bank write cannot leave clone parts
     permanently reading the wrong register map. */
⋮----
static ICM20948_Status_t ICM20948_ReadRawRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
⋮----
static ICM20948_Status_t ICM20948_WriteRawRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t value)
⋮----
static ICM20948_Status_t ICM20948_MagReadRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *value)
⋮----
static ICM20948_Status_t ICM20948_MagReadRegisters(ICM20948_Handle_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
⋮----
static ICM20948_Status_t ICM20948_MagWriteRegister(ICM20948_Handle_t *dev, uint8_t reg, uint8_t value)
⋮----
/* SLV4 is serviced after SLV0..3. Temporarily stop the failed SLV0 stream
     so this test measures SLV4 and the shared auxiliary bus independently. */
⋮----
/* Clear only status left by transactions which happened before SLV4. */
⋮----
/* Configure and trigger one direct auxiliary-I2C read. */
⋮----
/* EN is cleared by hardware after the one-byte transaction. Polling it does
     not clear I2C_MST_STATUS and tells us whether the scheduler ran at all. */
⋮----
/* I2C_MST_STATUS is read-to-clear, so capture it before leaving debug. */
⋮----
static ICM20948_Status_t ICM20948_MagConfigureContinuousRead(ICM20948_Handle_t *dev)
⋮----
/*
   * Set this explicitly instead of relying on its reset value. Earlier
   * firmware enabled DELAY_ES_SHADOW, which can leave EXT_SLV_SENS_DATA
   * unpublished on this setup until the ICM is fully power-cycled.
   */
⋮----
static ICM20948_Status_t ICM20948_FromHalStatus(HAL_StatusTypeDef status)
⋮----
static uint8_t ICM20948_BankValue(ICM20948_Bank_t bank)
⋮----
static int16_t ICM20948_ToInt16(uint8_t msb, uint8_t lsb)
⋮----
static float ICM20948_AccelSensitivity(ICM20948_AccelRange_t range)
⋮----
static float ICM20948_GyroSensitivity(ICM20948_GyroRange_t range)
```

## File: todo.md
```markdown
# ICM20948 TODO

- [ ] Thêm API chọn full-scale cho accelerometer và gyroscope.
- [ ] Áp dụng range đã chọn đồng nhất cho trục X/Y/Z (ICM20948 không đặt range riêng từng trục).
- [ ] Cập nhật hệ số đổi raw -> `g` và `deg/s` theo range, rồi log lại range đang dùng, viết thành hàm trong component chứ không chỉ trả mỗi raw
```
