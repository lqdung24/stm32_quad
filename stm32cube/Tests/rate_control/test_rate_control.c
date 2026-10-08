#include "rate_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_EPSILON 0.0001f

static const RateControlConfig test_config = {
    .pid = {
        [RATE_CONTROL_ROLL] = {
            .kp = 2.0f,
            .ki = 1.0f,
            .kd = 0.5f,
            .integral_limit = 1.0f,
            .output_limit = 10.0f,
            .derivative_cutoff_hz = 20.0f,
        },
        [RATE_CONTROL_PITCH] = {
            .kp = 2.0f,
            .ki = 1.0f,
            .kd = 0.5f,
            .integral_limit = 1.0f,
            .output_limit = 10.0f,
            .derivative_cutoff_hz = 20.0f,
        },
        [RATE_CONTROL_YAW] = {
            .kp = 2.0f,
            .ki = 1.0f,
            .kd = 0.0f,
            .integral_limit = 1.0f,
            .output_limit = 10.0f,
            .derivative_cutoff_hz = 0.0f,
        },
    },
    .maximum_rate_rad_s = {4.0f, 4.0f, 2.0f},
};

static void assert_near(float actual, float expected)
{
  assert(fabsf(actual - expected) < TEST_EPSILON);
}

static void test_command_scaling_and_clamping(void)
{
  RateControl control;
  RateControlDebug debug;

  assert(RateControl_Init(&control, &test_config));
  RateControl_SetCommand(&control, 500, -1000, 2000);
  assert(RateControl_GetDebug(&control, &debug));
  assert_near(debug.target_rad_s[RATE_CONTROL_ROLL], 2.0f);
  assert_near(debug.target_rad_s[RATE_CONTROL_PITCH], -4.0f);
  assert_near(debug.target_rad_s[RATE_CONTROL_YAW], 2.0f);
}

static void test_pid_direction_and_derivative_on_measurement(void)
{
  RateControl control;
  RateControlDebug debug;
  const float stationary[RATE_CONTROL_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
  const float moving[RATE_CONTROL_AXIS_COUNT] = {1.0f, 0.0f, 0.0f};

  assert(RateControl_Init(&control, &test_config));
  RateControl_SetCommand(&control, 500, 0, 0);
  assert(RateControl_Update(&control, stationary, 0.01f));
  assert(RateControl_GetDebug(&control, &debug));
  assert(debug.output[RATE_CONTROL_ROLL] > 0.0f);

  /* A positive measured-rate step must make the D contribution negative. */
  assert(RateControl_Update(&control, moving, 0.01f));
  assert(RateControl_GetDebug(&control, &debug));
  assert(debug.output[RATE_CONTROL_ROLL] <
         (2.0f * (debug.target_rad_s[RATE_CONTROL_ROLL] - 1.0f)));
}

static void test_integral_and_output_limits(void)
{
  RateControl control;
  RateControlDebug debug;
  const float measured[RATE_CONTROL_AXIS_COUNT] = {-100.0f, 0.0f, 0.0f};
  unsigned int i;

  assert(RateControl_Init(&control, &test_config));
  RateControl_SetCommand(&control, 1000, 0, 0);
  for (i = 0U; i < 1000U; ++i)
  {
    assert(RateControl_Update(&control, measured, 0.01f));
  }
  assert(RateControl_GetDebug(&control, &debug));
  assert_near(debug.output[RATE_CONTROL_ROLL], 10.0f);
  assert(fabsf(control.integral[RATE_CONTROL_ROLL]) <= 1.0f);
}

static void test_reset_and_bad_sample(void)
{
  RateControl control;
  RateControlDebug debug;
  const float measured[RATE_CONTROL_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};

  assert(RateControl_Init(&control, &test_config));
  RateControl_SetCommand(&control, 1000, 1000, 1000);
  assert(RateControl_Update(&control, measured, 0.01f));
  RateControl_Reset(&control);
  assert(RateControl_GetDebug(&control, &debug));
  assert_near(debug.target_rad_s[RATE_CONTROL_ROLL], 0.0f);
  assert_near(debug.output[RATE_CONTROL_ROLL], 0.0f);

  RateControl_SetCommand(&control, 1000, 0, 0);
  assert(!RateControl_Update(&control, measured, 0.1f));
  assert(RateControl_GetDebug(&control, &debug));
  assert_near(debug.output[RATE_CONTROL_ROLL], 0.0f);
}

static void test_1125_hz_dt_is_accepted(void)
{
  RateControl control;
  const float measured[RATE_CONTROL_AXIS_COUNT] = {0.1f, -0.1f, 0.05f};

  assert(RateControl_Init(&control, &test_config));
  assert(RateControl_Update(&control, measured, 1.0f / 1125.0f));
}

static void test_direct_rate_targets_validate_atomically(void)
{
  RateControl control;
  const float targets[3] = {5.0f, -5.0f, 1.0f};
  const float invalid[3] = {1.0f, NAN, 1.0f};
  const float infinite[3] = {1.0f, 1.0f, INFINITY};
  unsigned int axis;

  assert(RateControl_Init(&control, &test_config));
  assert(RateControl_SetTargetRates(&control, targets));
  assert_near(control.debug.target_rad_s[0], 4.0f);
  assert_near(control.debug.target_rad_s[1], -4.0f);
  assert_near(control.debug.target_rad_s[2], 1.0f);
  assert(!RateControl_SetTargetRates(&control, invalid));
  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    assert_near(control.debug.target_rad_s[axis], 0.0f);
  }
  assert(!RateControl_SetTargetRates(&control, infinite));
  assert(!RateControl_SetTargetRates(&control, NULL));
  assert(!RateControl_SetTargetRates(NULL, targets));
}

static void test_angle_leveling_signs_and_pilot_yaw(void)
{
  RateControl control;
  const float ten_degrees_half_rad = 0.0872664626f;

  assert(RateControl_Init(&control, &test_config));
  assert(RateControl_SetAngleCommand(&control, 0, 0, 500, 0.0f, 0.0f));
  assert_near(control.debug.target_rad_s[0], 0.0f);
  assert_near(control.debug.target_rad_s[1], 0.0f);
  assert_near(control.debug.target_rad_s[2], 1.0f);
  assert(RateControl_SetAngleCommand(&control, 0, 0, 0, 10.0f, 0.0f));
  assert_near(control.debug.target_rad_s[0],
              -2.0f * RATE_CONTROL_ANGLE_KP_PER_S *
                  sinf(ten_degrees_half_rad));
  assert_near(control.debug.target_rad_s[1], 0.0f);
  assert(RateControl_SetAngleCommand(&control, 0, 0, 0, 0.0f, 10.0f));
  assert_near(control.debug.target_rad_s[0], 0.0f);
  assert(control.debug.target_rad_s[1] < 0.0f);
  assert(RateControl_SetAngleCommand(&control, 500, -500, -1000,
                                     15.0f, -15.0f));
  assert_near(control.debug.target_rad_s[0], 0.0f);
  assert_near(control.debug.target_rad_s[1], 0.0f);
  assert_near(control.debug.target_rad_s[2], -2.0f);
}

static void test_angle_body_frame_coupling_and_joint_limit(void)
{
  RateControl control;
  const float degrees_to_rad = 0.01745329252f;
  float target_roll;
  float target_pitch;

  assert(RateControl_Init(&control, &test_config));
  /* With target roll equal to current roll, a pitch-only attitude change
   * rotates about BODY y with cos(roll) projection, not an Euler y rate. */
  assert(RateControl_SetAngleCommand(&control, 1000, 0, 0, 30.0f, 10.0f));
  assert_near(control.debug.target_rad_s[0], 0.0f);
  assert_near(control.debug.target_rad_s[1],
              -8.0f * sinf(5.0f * degrees_to_rad) *
                  cosf(30.0f * degrees_to_rad));

  assert(RateControl_SetAngleCommand(&control, 1000, 1000, 2000,
                                     -30.0f, -30.0f));
  assert_near(hypotf(control.debug.target_rad_s[0],
                     control.debug.target_rad_s[1]),
              RATE_CONTROL_ANGLE_MAX_RATE_RAD_S);
  assert(control.debug.target_rad_s[0] > 0.0f);
  assert(control.debug.target_rad_s[1] > 0.0f);
  assert_near(control.debug.target_rad_s[2], 2.0f);
  target_roll = control.debug.target_rad_s[0];
  target_pitch = control.debug.target_rad_s[1];
  assert(RateControl_SetAngleCommand(&control, 2000, 2000, 2000,
                                     -30.0f, -30.0f));
  assert_near(control.debug.target_rad_s[0], target_roll);
  assert_near(control.debug.target_rad_s[1], target_pitch);

  /* -170 to +30 degrees must take the shorter negative roll rotation. */
  assert(RateControl_SetAngleCommand(&control, 1000, 0, 0, -170.0f, 0.0f));
  assert_near(control.debug.target_rad_s[0],
              -RATE_CONTROL_ANGLE_MAX_RATE_RAD_S);
}

static void test_angle_invalid_attitude_clears_targets(void)
{
  RateControl control;

  assert(RateControl_Init(&control, &test_config));
  assert(RateControl_SetAngleCommand(&control, 1000, 0, 0, 0.0f, 0.0f));
  assert(!RateControl_SetAngleCommand(&control, 0, 0, 0, NAN, 0.0f));
  assert_near(control.debug.target_rad_s[0], 0.0f);
  assert(!RateControl_SetAngleCommand(&control, 0, 0, 0, 0.0f, INFINITY));
  assert(!RateControl_SetAngleCommand(&control, 0, 0, 0, 181.0f, 0.0f));
  assert(!RateControl_SetAngleCommand(&control, 0, 0, 0, 0.0f, 91.0f));
}

int main(void)
{
  test_command_scaling_and_clamping();
  test_pid_direction_and_derivative_on_measurement();
  test_integral_and_output_limits();
  test_reset_and_bad_sample();
  test_1125_hz_dt_is_accepted();
  test_direct_rate_targets_validate_atomically();
  test_angle_leveling_signs_and_pilot_yaw();
  test_angle_body_frame_coupling_and_joint_limit();
  test_angle_invalid_attitude_clears_targets();
  puts("rate control tests: PASS");
  return 0;
}
