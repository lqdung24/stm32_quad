#include "motor_mixer.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define EPSILON 0.001f

static void assert_close(float actual, float expected)
{
  assert(fabsf(actual - expected) < EPSILON);
}

static void test_collective(void)
{
  MotorMixerResult result;
  unsigned int motor;

  assert(MotorMixer_MixQuadX(400.0f, 0.0f, 0.0f, 0.0f, &result));
  for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    assert_close(result.command[motor], 400.0f);
  }
  assert(!result.collective_shifted);
  assert(!result.correction_scaled);
}

static void test_pure_roll(void)
{
  MotorMixerResult result;

  assert(MotorMixer_MixQuadX(500.0f, 100.0f, 0.0f, 0.0f, &result));
  assert_close(result.command[MOTOR_MIXER_M1_FRONT_LEFT], 600.0f);
  assert_close(result.command[MOTOR_MIXER_M2_REAR_LEFT], 600.0f);
  assert_close(result.command[MOTOR_MIXER_M3_FRONT_RIGHT], 400.0f);
  assert_close(result.command[MOTOR_MIXER_M4_REAR_RIGHT], 400.0f);
}

static void test_pure_pitch(void)
{
  MotorMixerResult result;

  assert(MotorMixer_MixQuadX(500.0f, 0.0f, 100.0f, 0.0f, &result));
  assert_close(result.command[MOTOR_MIXER_M1_FRONT_LEFT], 600.0f);
  assert_close(result.command[MOTOR_MIXER_M2_REAR_LEFT], 400.0f);
  assert_close(result.command[MOTOR_MIXER_M3_FRONT_RIGHT], 600.0f);
  assert_close(result.command[MOTOR_MIXER_M4_REAR_RIGHT], 400.0f);
}

static void test_pure_yaw(void)
{
  MotorMixerResult result;

  assert(MotorMixer_MixQuadX(500.0f, 0.0f, 0.0f, 100.0f, &result));
  assert_close(result.command[MOTOR_MIXER_M1_FRONT_LEFT], 400.0f);
  assert_close(result.command[MOTOR_MIXER_M2_REAR_LEFT], 600.0f);
  assert_close(result.command[MOTOR_MIXER_M3_FRONT_RIGHT], 600.0f);
  assert_close(result.command[MOTOR_MIXER_M4_REAR_RIGHT], 400.0f);
}

static void test_desaturation(void)
{
  MotorMixerResult result;

  assert(MotorMixer_MixQuadX(950.0f, 100.0f, 0.0f, 0.0f, &result));
  assert(result.collective_shifted);
  assert(!result.correction_scaled);
  assert_close(result.collective_command, 900.0f);
  assert_close(result.command[MOTOR_MIXER_M1_FRONT_LEFT], 1000.0f);
  assert_close(result.command[MOTOR_MIXER_M3_FRONT_RIGHT], 800.0f);

  assert(MotorMixer_MixQuadX(500.0f, 600.0f, 0.0f, 0.0f, &result));
  assert(result.correction_scaled);
  assert_close(result.command[MOTOR_MIXER_M1_FRONT_LEFT], 1000.0f);
  assert_close(result.command[MOTOR_MIXER_M3_FRONT_RIGHT], 0.0f);
}

static void test_per_motor_idle_mapping(void)
{
  const MotorMixerOutputConfig config = {
      .disarmed_pulse_us = 1000U,
      .idle_pulse_us = {1220U, 1225U, 1210U, 1225U},
      .maximum_pulse_us = 2000U,
  };
  const float command[MOTOR_MIXER_MOTOR_COUNT] = {
      200.0f, 205.0f, 190.0f, 205.0f,
  };
  const float zero_command[MOTOR_MIXER_MOTOR_COUNT] = {
      0.0f, 0.0f, 0.0f, 0.0f,
  };
  uint16_t pulse_us[MOTOR_MIXER_MOTOR_COUNT];

  assert(MotorMixer_MapToPulseUs(&config, command, false, pulse_us));
  assert(pulse_us[0] == 1000U);
  assert(pulse_us[1] == 1000U);
  assert(pulse_us[2] == 1000U);
  assert(pulse_us[3] == 1000U);

  assert(MotorMixer_MapToPulseUs(&config, command, true, pulse_us));
  assert(pulse_us[MOTOR_MIXER_M1_FRONT_LEFT] == 1220U);
  assert(pulse_us[MOTOR_MIXER_M2_REAR_LEFT] == 1225U);
  assert(pulse_us[MOTOR_MIXER_M3_FRONT_RIGHT] == 1210U);
  assert(pulse_us[MOTOR_MIXER_M4_REAR_RIGHT] == 1225U);

  assert(MotorMixer_MapToPulseUs(&config,
                                 zero_command,
                                 true,
                                 pulse_us));
  assert(pulse_us[MOTOR_MIXER_M1_FRONT_LEFT] == 1220U);
  assert(pulse_us[MOTOR_MIXER_M2_REAR_LEFT] == 1225U);
  assert(pulse_us[MOTOR_MIXER_M3_FRONT_RIGHT] == 1210U);
  assert(pulse_us[MOTOR_MIXER_M4_REAR_RIGHT] == 1225U);
}

static void test_command_retains_esc_pulse_scale(void)
{
  const MotorMixerOutputConfig config = {
      .disarmed_pulse_us = 1000U,
      .idle_pulse_us = {1220U, 1225U, 1210U, 1225U},
      .maximum_pulse_us = 2000U,
  };
  const float half_command[MOTOR_MIXER_MOTOR_COUNT] = {
      500.0f, 500.0f, 500.0f, 500.0f,
  };
  const float maximum_command[MOTOR_MIXER_MOTOR_COUNT] = {
      1000.0f, 1000.0f, 1000.0f, 1000.0f,
  };
  uint16_t pulse_us[MOTOR_MIXER_MOTOR_COUNT];
  unsigned int motor;

  assert(MotorMixer_MapToPulseUs(&config,
                                 half_command,
                                 true,
                                 pulse_us));
  for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    assert(pulse_us[motor] == 1500U);
  }

  assert(MotorMixer_MapToPulseUs(&config,
                                 maximum_command,
                                 true,
                                 pulse_us));
  for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    assert(pulse_us[motor] == 2000U);
  }
}

static void test_raw_threshold_mapping(void)
{
  const MotorMixerOutputConfig config = {
      .disarmed_pulse_us = 1000U,
      .idle_pulse_us = {1000U, 1000U, 1000U, 1000U},
      .maximum_pulse_us = 2000U,
  };
  const float command[MOTOR_MIXER_MOTOR_COUNT] = {
      5.0f, 5.0f, 5.0f, 5.0f,
  };
  uint16_t pulse_us[MOTOR_MIXER_MOTOR_COUNT];
  unsigned int motor;

  assert(MotorMixer_MapToPulseUs(&config, command, true, pulse_us));
  for (motor = 0U; motor < MOTOR_MIXER_MOTOR_COUNT; ++motor)
  {
    assert(pulse_us[motor] == 1005U);
  }
}

int main(void)
{
  test_collective();
  test_pure_roll();
  test_pure_pitch();
  test_pure_yaw();
  test_desaturation();
  test_per_motor_idle_mapping();
  test_command_retains_esc_pulse_scale();
  test_raw_threshold_mapping();
  puts("motor mixer tests: PASS");
  return 0;
}
