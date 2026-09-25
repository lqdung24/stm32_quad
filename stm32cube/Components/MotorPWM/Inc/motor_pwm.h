#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../../Motor/Inc/motor.h"
#include "../../PwmTimer/Inc/pwm_timer.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_MOTOR_COUNT 4U

typedef struct
{
  PwmChannel *channel[MOTOR_PWM_MOTOR_COUNT];
  uint32_t stop_compare;
  uint32_t minimum_compare;
  uint32_t maximum_compare;
  uint32_t idle_compare[MOTOR_PWM_MOTOR_COUNT];
} MotorPwm_Config;

typedef struct
{
  MotorPwm_Config config;
  float requested_throttle[MOTOR_PWM_MOTOR_COUNT];
  float applied_throttle[MOTOR_PWM_MOTOR_COUNT];
  uint16_t compare[MOTOR_PWM_MOTOR_COUNT];
  bool initialized;
  bool started;
} MotorPwm;

/*
 * Bind four already-initialized PwmChannel objects to the generic motor
 * interface. Timer/channel configuration remains exclusively in PwmTimer.
 */
MotorStatus MotorPwm_Init(MotorPwm *driver,
                          MotorOutput *output,
                          const MotorPwm_Config *config);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */
