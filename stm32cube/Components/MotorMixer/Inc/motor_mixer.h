#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_MIXER_MOTOR_COUNT 4U
#define MOTOR_MIXER_COMMAND_MAX 1000.0f

typedef enum
{
  MOTOR_MIXER_M1_FRONT_LEFT = 0,
  MOTOR_MIXER_M2_REAR_LEFT = 1,
  MOTOR_MIXER_M3_FRONT_RIGHT = 2,
  MOTOR_MIXER_M4_REAR_RIGHT = 3
} MotorMixerMotor;

typedef struct
{
  uint16_t disarmed_pulse_us;
  uint16_t idle_pulse_us[MOTOR_MIXER_MOTOR_COUNT];
  uint16_t maximum_pulse_us;
} MotorMixerOutputConfig;

typedef struct
{
  float command[MOTOR_MIXER_MOTOR_COUNT];
  float collective_command;
  float correction_scale;
  bool collective_shifted;
  bool correction_scaled;
} MotorMixerResult;

/*
 * Quad-X allocation in BODY FRD, viewed from above. Positive BODY yaw is
 * clockwise when viewed from above:
 *
 *   M1 front-left  CW   (+roll, +pitch, -yaw)
 *   M2 rear-left   CCW  (+roll, -pitch, +yaw)
 *   M3 front-right CCW  (-roll, +pitch, +yaw)
 *   M4 rear-right  CW   (-roll, -pitch, -yaw)
 *
 * Inputs and result.command[] use the same logical 0..1000 command scale.
 * The mixer shifts collective equally before scaling corrections, preserving
 * requested torque whenever actuator range permits it.
 */
bool MotorMixer_MixQuadX(float collective,
                          float roll,
                          float pitch,
                          float yaw,
                          MotorMixerResult *result);

/*
 * Converts logical 0..1000 motor commands to ESC pulses. Active outputs are
 * floored at their individual calibrated idle pulse; inactive outputs use the
 * common disarmed pulse.
 */
bool MotorMixer_MapToPulseUs(
    const MotorMixerOutputConfig *config,
    const float command[MOTOR_MIXER_MOTOR_COUNT],
    bool active,
    uint16_t pulse_us[MOTOR_MIXER_MOTOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MIXER_H */
