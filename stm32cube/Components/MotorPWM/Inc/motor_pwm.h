#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_MOTOR_COUNT 4U

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel[MOTOR_PWM_MOTOR_COUNT];
  uint16_t disarmed_pulse_us;
  uint16_t minimum_pulse_us;
  uint16_t maximum_pulse_us;
} MotorPwm_Config_t;

typedef struct
{
  MotorPwm_Config_t config;
  uint16_t pulse_us[MOTOR_PWM_MOTOR_COUNT];
  bool attached;
  bool armed;
} MotorPwm_Handle_t;

/*
 * The timer must count at exactly 1 MHz, so one compare count equals 1 us.
 * The application owns timer/PWM initialization and channel start. Attach only
 * binds an already-running timer and writes the disarmed compare values.
 */
bool MotorPwm_Attach(MotorPwm_Handle_t *motors,
                     const MotorPwm_Config_t *config);

/* Arm only removes the software write lock; it does not raise motor pulses. */
bool MotorPwm_Arm(MotorPwm_Handle_t *motors);

/* Immediately writes the disarmed pulse to all motors and restores the lock. */
void MotorPwm_Disarm(MotorPwm_Handle_t *motors);

/* motor_index is zero-based: 0..3. Values outside configured limits fail. */
bool MotorPwm_SetPulseUs(MotorPwm_Handle_t *motors,
                         uint8_t motor_index,
                         uint16_t pulse_us);

bool MotorPwm_SetAllPulseUs(MotorPwm_Handle_t *motors,
                            const uint16_t pulse_us[MOTOR_PWM_MOTOR_COUNT]);

uint16_t MotorPwm_GetPulseUs(const MotorPwm_Handle_t *motors,
                             uint8_t motor_index);

bool MotorPwm_IsAttached(const MotorPwm_Handle_t *motors);
bool MotorPwm_IsArmed(const MotorPwm_Handle_t *motors);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */
