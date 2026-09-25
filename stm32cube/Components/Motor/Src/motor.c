#include "../Inc/motor.h"

#include <math.h>
#include <stddef.h>

static bool MotorOutput_OpsValid(const MotorOps *ops);
static bool MotorOutput_ThrottleValid(float throttle);

MotorStatus MotorOutput_Init(MotorOutput *output,
                             const MotorOps *ops,
                             void *driver,
                             uint8_t motor_count,
                             MotorProtocol protocol)
{
  if ((output == NULL) || !MotorOutput_OpsValid(ops) || (driver == NULL) ||
      (motor_count == 0U) || (motor_count > MOTOR_OUTPUT_MAX_MOTORS) ||
      ((protocol != MOTOR_PROTOCOL_PWM) &&
       (protocol != MOTOR_PROTOCOL_DSHOT)))
  {
    return MOTOR_INVALID_ARGUMENT;
  }

  output->ops = ops;
  output->driver = driver;
  output->motor_count = motor_count;
  output->protocol = protocol;
  output->initialized = true;
  return MOTOR_OK;
}

MotorStatus MotorOutput_Start(MotorOutput *output)
{
  if (!MotorOutput_IsReady(output))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  return output->ops->start(output->driver);
}

MotorStatus MotorOutput_Stop(MotorOutput *output)
{
  if (!MotorOutput_IsReady(output))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  return output->ops->stop(output->driver);
}

MotorStatus MotorOutput_SetThrottle(MotorOutput *output,
                                    uint8_t motor_index,
                                    float throttle)
{
  if (!MotorOutput_IsReady(output) ||
      (motor_index >= output->motor_count) ||
      !MotorOutput_ThrottleValid(throttle))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  return output->ops->set_throttle(output->driver, motor_index, throttle);
}

MotorStatus MotorOutput_SetAllThrottle(MotorOutput *output,
                                       const float *throttle,
                                       uint8_t motor_count)
{
  uint8_t motor;

  if (!MotorOutput_IsReady(output) || (throttle == NULL) ||
      (motor_count != output->motor_count))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  for (motor = 0U; motor < motor_count; ++motor)
  {
    if (!MotorOutput_ThrottleValid(throttle[motor]))
    {
      return MOTOR_INVALID_ARGUMENT;
    }
  }
  return output->ops->set_all_throttle(output->driver,
                                       throttle,
                                       motor_count);
}

MotorStatus MotorOutput_Update(MotorOutput *output)
{
  if (!MotorOutput_IsReady(output))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  return output->ops->update(output->driver);
}

bool MotorOutput_IsReady(const MotorOutput *output)
{
  return (output != NULL) && output->initialized &&
         MotorOutput_OpsValid(output->ops) && (output->driver != NULL) &&
         (output->motor_count > 0U) &&
         (output->motor_count <= MOTOR_OUTPUT_MAX_MOTORS);
}

bool MotorOutput_IsStarted(const MotorOutput *output)
{
  return MotorOutput_IsReady(output) &&
         output->ops->is_started(output->driver);
}

float MotorOutput_GetThrottle(const MotorOutput *output,
                              uint8_t motor_index)
{
  if (!MotorOutput_IsReady(output) ||
      (motor_index >= output->motor_count))
  {
    return 0.0f;
  }
  return output->ops->get_throttle(output->driver, motor_index);
}

uint16_t MotorOutput_GetRawOutput(const MotorOutput *output,
                                  uint8_t motor_index)
{
  if (!MotorOutput_IsReady(output) ||
      (motor_index >= output->motor_count))
  {
    return 0U;
  }
  return output->ops->get_raw_output(output->driver, motor_index);
}

MotorProtocol MotorOutput_GetProtocol(const MotorOutput *output)
{
  if (!MotorOutput_IsReady(output))
  {
    return MOTOR_PROTOCOL_PWM;
  }
  return output->protocol;
}

static bool MotorOutput_OpsValid(const MotorOps *ops)
{
  return (ops != NULL) && (ops->start != NULL) && (ops->stop != NULL) &&
         (ops->set_throttle != NULL) &&
         (ops->set_all_throttle != NULL) && (ops->update != NULL) &&
         (ops->is_started != NULL) && (ops->get_throttle != NULL) &&
         (ops->get_raw_output != NULL);
}

static bool MotorOutput_ThrottleValid(float throttle)
{
  return isfinite(throttle) && (throttle >= 0.0f) && (throttle <= 1.0f);
}
