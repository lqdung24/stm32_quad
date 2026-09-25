#include "../Inc/motor_pwm.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static MotorStatus MotorPwm_Start(void *driver_context);
static MotorStatus MotorPwm_Stop(void *driver_context);
static MotorStatus MotorPwm_SetThrottle(void *driver_context,
                                        uint8_t motor_index,
                                        float throttle);
static MotorStatus MotorPwm_SetAllThrottle(void *driver_context,
                                           const float *throttle,
                                           uint8_t motor_count);
static MotorStatus MotorPwm_Update(void *driver_context);
static bool MotorPwm_IsStarted(const void *driver_context);
static float MotorPwm_GetThrottle(const void *driver_context,
                                  uint8_t motor_index);
static uint16_t MotorPwm_GetRawOutput(const void *driver_context,
                                      uint8_t motor_index);
static bool MotorPwm_ConfigValid(const MotorPwm_Config *config);
static uint32_t MotorPwm_ThrottleToCompare(const MotorPwm *driver,
                                           uint8_t motor_index,
                                           float throttle);
static MotorStatus MotorPwm_WriteStop(MotorPwm *driver);

static const MotorOps motor_pwm_ops = {
  .start = MotorPwm_Start,
  .stop = MotorPwm_Stop,
  .set_throttle = MotorPwm_SetThrottle,
  .set_all_throttle = MotorPwm_SetAllThrottle,
  .update = MotorPwm_Update,
  .is_started = MotorPwm_IsStarted,
  .get_throttle = MotorPwm_GetThrottle,
  .get_raw_output = MotorPwm_GetRawOutput,
};

MotorStatus MotorPwm_Init(MotorPwm *driver,
                          MotorOutput *output,
                          const MotorPwm_Config *config)
{
  MotorStatus status;

  if ((driver == NULL) || (output == NULL) ||
      !MotorPwm_ConfigValid(config))
  {
    return MOTOR_INVALID_ARGUMENT;
  }

  memset(driver, 0, sizeof(*driver));
  driver->config = *config;
  status = MotorPwm_WriteStop(driver);
  if (status != MOTOR_OK)
  {
    return status;
  }
  driver->initialized = true;

  status = MotorOutput_Init(output,
                            &motor_pwm_ops,
                            driver,
                            MOTOR_PWM_MOTOR_COUNT,
                            MOTOR_PROTOCOL_PWM);
  if (status != MOTOR_OK)
  {
    driver->initialized = false;
  }
  return status;
}

static MotorStatus MotorPwm_Start(void *driver_context)
{
  MotorPwm *driver = (MotorPwm *)driver_context;
  MotorStatus status;

  if ((driver == NULL) || !driver->initialized)
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  status = MotorPwm_WriteStop(driver);
  if (status == MOTOR_OK)
  {
    driver->started = true;
  }
  return status;
}

static MotorStatus MotorPwm_Stop(void *driver_context)
{
  MotorPwm *driver = (MotorPwm *)driver_context;
  MotorStatus status;

  if ((driver == NULL) || !driver->initialized)
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  status = MotorPwm_WriteStop(driver);
  driver->started = false;
  return status;
}

static MotorStatus MotorPwm_SetThrottle(void *driver_context,
                                        uint8_t motor_index,
                                        float throttle)
{
  MotorPwm *driver = (MotorPwm *)driver_context;

  if ((driver == NULL) || !driver->initialized || !driver->started ||
      (motor_index >= MOTOR_PWM_MOTOR_COUNT) || !isfinite(throttle) ||
      (throttle < 0.0f) || (throttle > 1.0f))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  driver->requested_throttle[motor_index] = throttle;
  return MOTOR_OK;
}

static MotorStatus MotorPwm_SetAllThrottle(void *driver_context,
                                           const float *throttle,
                                           uint8_t motor_count)
{
  MotorPwm *driver = (MotorPwm *)driver_context;
  uint8_t motor;

  if ((driver == NULL) || !driver->initialized || !driver->started ||
      (throttle == NULL) || (motor_count != MOTOR_PWM_MOTOR_COUNT))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
  {
    if (!isfinite(throttle[motor]) || (throttle[motor] < 0.0f) ||
        (throttle[motor] > 1.0f))
    {
      return MOTOR_INVALID_ARGUMENT;
    }
  }
  memcpy(driver->requested_throttle,
         throttle,
         sizeof(driver->requested_throttle));
  return MOTOR_OK;
}

static MotorStatus MotorPwm_Update(void *driver_context)
{
  MotorPwm *driver = (MotorPwm *)driver_context;
  uint32_t compare[MOTOR_PWM_MOTOR_COUNT];
  uint8_t motor;

  if ((driver == NULL) || !driver->initialized || !driver->started)
  {
    return MOTOR_ERROR;
  }

  for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
  {
    compare[motor] = MotorPwm_ThrottleToCompare(
        driver, motor, driver->requested_throttle[motor]);
  }
  for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
  {
    if (PwmChannel_SetCompare(driver->config.channel[motor],
                              compare[motor]) != PWM_TIMER_OK)
    {
      return MOTOR_ERROR;
    }
  }
  for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
  {
    driver->compare[motor] = (uint16_t)compare[motor];
    driver->applied_throttle[motor] = driver->requested_throttle[motor];
  }
  return MOTOR_OK;
}

static bool MotorPwm_IsStarted(const void *driver_context)
{
  const MotorPwm *driver = (const MotorPwm *)driver_context;

  return (driver != NULL) && driver->initialized && driver->started;
}

static float MotorPwm_GetThrottle(const void *driver_context,
                                  uint8_t motor_index)
{
  const MotorPwm *driver = (const MotorPwm *)driver_context;

  if ((driver == NULL) || !driver->initialized ||
      (motor_index >= MOTOR_PWM_MOTOR_COUNT))
  {
    return 0.0f;
  }
  return driver->applied_throttle[motor_index];
}

static uint16_t MotorPwm_GetRawOutput(const void *driver_context,
                                      uint8_t motor_index)
{
  const MotorPwm *driver = (const MotorPwm *)driver_context;

  if ((driver == NULL) || !driver->initialized ||
      (motor_index >= MOTOR_PWM_MOTOR_COUNT))
  {
    return 0U;
  }
  return driver->compare[motor_index];
}

static bool MotorPwm_ConfigValid(const MotorPwm_Config *config)
{
  PwmTimer *shared_timer;
  uint8_t motor;

  if ((config == NULL) || (config->channel[0] == NULL) ||
      !config->channel[0]->initialized ||
      (config->stop_compare > config->minimum_compare) ||
      (config->minimum_compare >= config->maximum_compare) ||
      (config->maximum_compare > UINT16_MAX))
  {
    return false;
  }

  shared_timer = config->channel[0]->config.timer;
  for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
  {
    if ((config->channel[motor] == NULL) ||
        !config->channel[motor]->initialized ||
        (config->channel[motor]->config.timer != shared_timer) ||
        (config->idle_compare[motor] < config->minimum_compare) ||
        (config->idle_compare[motor] > config->maximum_compare))
    {
      return false;
    }
  }
  return config->maximum_compare < shared_timer->config.period_ticks;
}

static uint32_t MotorPwm_ThrottleToCompare(const MotorPwm *driver,
                                           uint8_t motor_index,
                                           float throttle)
{
  float mapped;
  uint32_t compare;

  if (throttle <= 0.0f)
  {
    return driver->config.stop_compare;
  }
  mapped = (float)driver->config.minimum_compare +
      throttle * (float)(driver->config.maximum_compare -
                         driver->config.minimum_compare);
  compare = (uint32_t)(mapped + 0.5f);
  if (compare < driver->config.idle_compare[motor_index])
  {
    compare = driver->config.idle_compare[motor_index];
  }
  return compare;
}

static MotorStatus MotorPwm_WriteStop(MotorPwm *driver)
{
  uint8_t motor;
  bool write_ok = true;

  for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
  {
    if (PwmChannel_SetCompare(driver->config.channel[motor],
                              driver->config.stop_compare) != PWM_TIMER_OK)
    {
      write_ok = false;
    }
    driver->requested_throttle[motor] = 0.0f;
    driver->applied_throttle[motor] = 0.0f;
    driver->compare[motor] = (uint16_t)driver->config.stop_compare;
  }
  return write_ok ? MOTOR_OK : MOTOR_ERROR;
}
