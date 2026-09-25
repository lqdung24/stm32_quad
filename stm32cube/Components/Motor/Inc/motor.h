#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_OUTPUT_MAX_MOTORS 4U

typedef enum
{
  MOTOR_OK = 0,
  MOTOR_BUSY,
  MOTOR_ERROR,
  MOTOR_INVALID_ARGUMENT
} MotorStatus;

typedef enum
{
  MOTOR_PROTOCOL_PWM = 0,
  MOTOR_PROTOCOL_DSHOT
} MotorProtocol;

typedef struct MotorOutput MotorOutput;

typedef struct
{
  MotorStatus (*start)(void *driver);
  MotorStatus (*stop)(void *driver);
  MotorStatus (*set_throttle)(void *driver,
                              uint8_t motor_index,
                              float throttle);
  MotorStatus (*set_all_throttle)(void *driver,
                                  const float *throttle,
                                  uint8_t motor_count);
  MotorStatus (*update)(void *driver);
  bool (*is_started)(const void *driver);
  float (*get_throttle)(const void *driver, uint8_t motor_index);
  uint16_t (*get_raw_output)(const void *driver, uint8_t motor_index);
} MotorOps;

struct MotorOutput
{
  const MotorOps *ops;
  void *driver;
  uint8_t motor_count;
  MotorProtocol protocol;
  bool initialized;
};

MotorStatus MotorOutput_Init(MotorOutput *output,
                             const MotorOps *ops,
                             void *driver,
                             uint8_t motor_count,
                             MotorProtocol protocol);
MotorStatus MotorOutput_Start(MotorOutput *output);
MotorStatus MotorOutput_Stop(MotorOutput *output);
MotorStatus MotorOutput_SetThrottle(MotorOutput *output,
                                    uint8_t motor_index,
                                    float throttle);
MotorStatus MotorOutput_SetAllThrottle(MotorOutput *output,
                                       const float *throttle,
                                       uint8_t motor_count);
MotorStatus MotorOutput_Update(MotorOutput *output);
bool MotorOutput_IsReady(const MotorOutput *output);
bool MotorOutput_IsStarted(const MotorOutput *output);
float MotorOutput_GetThrottle(const MotorOutput *output,
                              uint8_t motor_index);
uint16_t MotorOutput_GetRawOutput(const MotorOutput *output,
                                  uint8_t motor_index);
MotorProtocol MotorOutput_GetProtocol(const MotorOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
