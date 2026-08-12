#include "../Inc/motor_mixer.h"

#include <math.h>
#include <stddef.h>

bool MotorMixer_MixQuadX(float collective,
                          float roll,
                          float pitch,
                          float yaw,
                          MotorMixerResult *result)
{
  static const float roll_coefficient[MOTOR_MIXER_MOTOR_COUNT] = {
      1.0f, 1.0f, -1.0f, -1.0f,
  };
  static const float pitch_coefficient[MOTOR_MIXER_MOTOR_COUNT] = {
      1.0f, -1.0f, 1.0f, -1.0f,
  };
  static const float yaw_coefficient[MOTOR_MIXER_MOTOR_COUNT] = {
      -1.0f, 1.0f, 1.0f, -1.0f,
  };
  float correction[MOTOR_MIXER_MOTOR_COUNT];
  float minimum_correction;
  float maximum_correction;
  float correction_span;
  float correction_scale = 1.0f;
  float minimum_collective;
  float maximum_collective;
  float applied_collective;
  uint8_t motor;

  if ((result == NULL) ||
      !isfinite(collective) ||
      !isfinite(roll) ||
      !isfinite(pitch) ||
      !isfinite(yaw) ||
      (collective < 0.0f) ||
      (collective > MOTOR_MIXER_COMMAND_MAX))
  {
    return false;
  }

  for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    correction[motor] =
        (roll_coefficient[motor] * roll) +
        (pitch_coefficient[motor] * pitch) +
        (yaw_coefficient[motor] * yaw);
  }

  minimum_correction = correction[0];
  maximum_correction = correction[0];
  for (motor = 1U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    if (correction[motor] < minimum_correction)
    {
      minimum_correction = correction[motor];
    }
    if (correction[motor] > maximum_correction)
    {
      maximum_correction = correction[motor];
    }
  }

  correction_span = maximum_correction - minimum_correction;
  if (correction_span > MOTOR_MIXER_COMMAND_MAX)
  {
    correction_scale = MOTOR_MIXER_COMMAND_MAX / correction_span;
    minimum_correction *= correction_scale;
    maximum_correction *= correction_scale;
    for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
    {
      correction[motor] *= correction_scale;
    }
  }

  minimum_collective = -minimum_correction;
  maximum_collective =
      MOTOR_MIXER_COMMAND_MAX - maximum_correction;
  applied_collective = collective;
  if (applied_collective < minimum_collective)
  {
    applied_collective = minimum_collective;
  }
  if (applied_collective > maximum_collective)
  {
    applied_collective = maximum_collective;
  }

  for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    float command = applied_collective + correction[motor];
    if (command < 0.0f)
    {
      command = 0.0f;
    }
    if (command > MOTOR_MIXER_COMMAND_MAX)
    {
      command = MOTOR_MIXER_COMMAND_MAX;
    }
    result->command[motor] = command;
  }
  result->collective_command = applied_collective;
  result->correction_scale = correction_scale;
  result->collective_shifted = (applied_collective != collective);
  result->correction_scaled = (correction_scale < 1.0f);
  return true;
}

bool MotorMixer_MapToPulseUs(
    const MotorMixerOutputConfig *config,
    const float command[MOTOR_MIXER_MOTOR_COUNT],
    bool active,
    uint16_t pulse_us[MOTOR_MIXER_MOTOR_COUNT])
{
  uint8_t motor;

  if ((config == NULL) || (command == NULL) || (pulse_us == NULL) ||
      (config->disarmed_pulse_us >= config->maximum_pulse_us))
  {
    return false;
  }

  for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    if ((config->idle_pulse_us[motor] < config->disarmed_pulse_us) ||
        (config->idle_pulse_us[motor] >= config->maximum_pulse_us) ||
        !isfinite(command[motor]) ||
        (command[motor] < 0.0f) ||
        (command[motor] > MOTOR_MIXER_COMMAND_MAX))
    {
      return false;
    }
  }

  for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    if (!active)
    {
      pulse_us[motor] = config->disarmed_pulse_us;
    }
    else
    {
      float pulse =
          (float)config->disarmed_pulse_us +
          ((command[motor] / MOTOR_MIXER_COMMAND_MAX) *
           (float)(config->maximum_pulse_us -
                   config->disarmed_pulse_us));
      if (pulse < (float)config->idle_pulse_us[motor])
      {
        pulse = (float)config->idle_pulse_us[motor];
      }
      pulse_us[motor] = (uint16_t)(pulse + 0.5f);
    }
  }
  return true;
}
