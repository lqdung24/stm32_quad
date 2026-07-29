#include "motor_pwm.h"

#include <stddef.h>

static bool MotorPwm_ConfigValid(const MotorPwm_Config_t *config);
static void MotorPwm_WriteCompare(const MotorPwm_Handle_t *motors,
                                  uint8_t motor_index,
                                  uint16_t pulse_us);
static void MotorPwm_WriteDisarmed(MotorPwm_Handle_t *motors);

bool MotorPwm_Init(MotorPwm_Handle_t *motors,
                   const MotorPwm_Config_t *config)
{
  uint8_t i;

  if ((motors == NULL) || !MotorPwm_ConfigValid(config))
  {
    return false;
  }

  motors->config = *config;
  motors->initialized = false;
  motors->armed = false;

  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    motors->pulse_us[i] = config->disarmed_pulse_us;
    __HAL_TIM_SET_COMPARE(config->timer,
                          config->channel[i],
                          config->disarmed_pulse_us);
  }

  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    if (HAL_TIM_PWM_Start(config->timer, config->channel[i]) != HAL_OK)
    {
      while (i > 0U)
      {
        --i;
        (void)HAL_TIM_PWM_Stop(config->timer, config->channel[i]);
      }
      return false;
    }
  }

  motors->initialized = true;
  return true;
}

bool MotorPwm_Arm(MotorPwm_Handle_t *motors)
{
  if ((motors == NULL) || !motors->initialized)
  {
    return false;
  }

  MotorPwm_WriteDisarmed(motors);
  motors->armed = true;
  return true;
}

void MotorPwm_Disarm(MotorPwm_Handle_t *motors)
{
  if ((motors == NULL) || !motors->initialized)
  {
    return;
  }

  MotorPwm_WriteDisarmed(motors);
  motors->armed = false;
}

bool MotorPwm_SetPulseUs(MotorPwm_Handle_t *motors,
                         uint8_t motor_index,
                         uint16_t pulse_us)
{
  if ((motors == NULL) ||
      !motors->initialized ||
      !motors->armed ||
      (motor_index >= MOTOR_PWM_MOTOR_COUNT) ||
      (pulse_us < motors->config.minimum_pulse_us) ||
      (pulse_us > motors->config.maximum_pulse_us))
  {
    return false;
  }

  MotorPwm_WriteCompare(motors, motor_index, pulse_us);
  motors->pulse_us[motor_index] = pulse_us;
  return true;
}

bool MotorPwm_SetAllPulseUs(MotorPwm_Handle_t *motors,
                            const uint16_t pulse_us[MOTOR_PWM_MOTOR_COUNT])
{
  uint8_t i;

  if ((motors == NULL) ||
      (pulse_us == NULL) ||
      !motors->initialized ||
      !motors->armed)
  {
    return false;
  }

  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    if ((pulse_us[i] < motors->config.minimum_pulse_us) ||
        (pulse_us[i] > motors->config.maximum_pulse_us))
    {
      return false;
    }
  }

  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    MotorPwm_WriteCompare(motors, i, pulse_us[i]);
    motors->pulse_us[i] = pulse_us[i];
  }

  return true;
}

uint16_t MotorPwm_GetPulseUs(const MotorPwm_Handle_t *motors,
                             uint8_t motor_index)
{
  if ((motors == NULL) ||
      !motors->initialized ||
      (motor_index >= MOTOR_PWM_MOTOR_COUNT))
  {
    return 0U;
  }

  return motors->pulse_us[motor_index];
}

bool MotorPwm_IsInitialized(const MotorPwm_Handle_t *motors)
{
  return (motors != NULL) && motors->initialized;
}

bool MotorPwm_IsArmed(const MotorPwm_Handle_t *motors)
{
  return (motors != NULL) && motors->initialized && motors->armed;
}

static bool MotorPwm_ConfigValid(const MotorPwm_Config_t *config)
{
  uint8_t i;

  if ((config == NULL) ||
      (config->timer == NULL) ||
      (config->disarmed_pulse_us > config->minimum_pulse_us) ||
      (config->minimum_pulse_us >= config->maximum_pulse_us) ||
      (config->maximum_pulse_us > config->timer->Init.Period))
  {
    return false;
  }

  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    if ((config->channel[i] != TIM_CHANNEL_1) &&
        (config->channel[i] != TIM_CHANNEL_2) &&
        (config->channel[i] != TIM_CHANNEL_3) &&
        (config->channel[i] != TIM_CHANNEL_4))
    {
      return false;
    }
  }

  return true;
}

static void MotorPwm_WriteCompare(const MotorPwm_Handle_t *motors,
                                  uint8_t motor_index,
                                  uint16_t pulse_us)
{
  __HAL_TIM_SET_COMPARE(motors->config.timer,
                        motors->config.channel[motor_index],
                        pulse_us);
}

static void MotorPwm_WriteDisarmed(MotorPwm_Handle_t *motors)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    MotorPwm_WriteCompare(motors, i, motors->config.disarmed_pulse_us);
    motors->pulse_us[i] = motors->config.disarmed_pulse_us;
  }
}
