#include "mahony.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#define DEG_TO_RAD 0.017453292519943295f

static const Mahony_Config_t test_config = {
    .kp = 2.0f,
    .ki = 0.05f,
    .integral_limit_rad_s = 0.1f,
    .accel_min_norm = 0.8f,
    .accel_max_norm = 1.2f,
};

static void assert_near(float actual, float expected, float tolerance)
{
  assert(isfinite(actual));
  assert(fabsf(actual - expected) < tolerance);
}

static Mahony_Handle_t level_filter(void)
{
  Mahony_Handle_t filter;
  Mahony_Init(&filter, &test_config);
  assert(!filter.initialized);
  assert(Mahony_InitFromAccel(&filter, 0.0f, 0.0f, 1.0f));
  return filter;
}

static void test_gravity_initialization_signs_without_magnetometer(void)
{
  const float poses[][2] = {
      {0.0f, 0.0f}, {30.0f, 0.0f}, {-30.0f, 0.0f},
      {0.0f, 30.0f}, {0.0f, -30.0f}, {25.0f, -20.0f},
  };
  size_t i;

  for (i = 0U; i < sizeof(poses) / sizeof(poses[0]); ++i)
  {
    Mahony_Handle_t filter;
    Mahony_Euler_t euler;
    const float roll = poses[i][0] * DEG_TO_RAD;
    const float pitch = poses[i][1] * DEG_TO_RAD;
    Mahony_Init(&filter, &test_config);
    /* BODY FRD gravity is +Z when level, with right wing down / nose up positive. */
    assert(Mahony_InitFromAccel(&filter, -sinf(pitch),
                               sinf(roll) * cosf(pitch),
                               cosf(roll) * cosf(pitch)));
    assert(Mahony_GetEulerDegrees(&filter, &euler));
    assert_near(euler.roll, poses[i][0], 0.0001f);
    assert_near(euler.pitch, poses[i][1], 0.0001f);
    assert_near(euler.yaw, 0.0f, 0.0001f);
  }
}

static void test_init_requires_gravity_and_valid_config(void)
{
  const float bad_gravity[][3] = {
      {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.5f},
      {0.0f, 0.0f, 0.8f}, {0.0f, 0.0f, 1.2f},
      {0.0f, 0.0f, 2.0f}, {NAN, 0.0f, 1.0f},
      {0.0f, INFINITY, 1.0f}, {0.0f, 0.0f, NAN},
  };
  Mahony_Config_t configs[7];
  Mahony_Handle_t filter;
  size_t i;

  for (i = 0U; i < sizeof(bad_gravity) / sizeof(bad_gravity[0]); ++i)
  {
    filter = level_filter();
    assert(!Mahony_InitFromAccel(&filter, bad_gravity[i][0],
                                bad_gravity[i][1], bad_gravity[i][2]));
    assert(!filter.initialized);
  }
  for (i = 0U; i < sizeof(configs) / sizeof(configs[0]); ++i)
  {
    configs[i] = test_config;
  }
  configs[0].kp = NAN;
  configs[1].ki = INFINITY;
  configs[2].kp = -1.0f;
  configs[3].integral_limit_rad_s = -1.0f;
  configs[4].accel_min_norm = NAN;
  configs[5].accel_max_norm = configs[5].accel_min_norm;
  configs[6].accel_max_norm = INFINITY;
  for (i = 0U; i < sizeof(configs) / sizeof(configs[0]); ++i)
  {
    filter = level_filter();
    Mahony_Init(&filter, &configs[i]);
    assert(!filter.initialized);
    assert(!Mahony_InitFromAccel(&filter, 0.0f, 0.0f, 1.0f));
  }
  filter = level_filter();
  Mahony_Init(&filter, NULL);
  assert(!filter.initialized);
  assert(!Mahony_InitFromAccel(&filter, 0.0f, 0.0f, 1.0f));
  Mahony_Init(NULL, &test_config);
  assert(!Mahony_InitFromAccel(NULL, 0.0f, 0.0f, 1.0f));
}

static void test_body_rate_integration_units_and_variable_dt(void)
{
  unsigned int axis;
  for (axis = 0U; axis < 3U; ++axis)
  {
    Mahony_Handle_t uniform = level_filter();
    Mahony_Handle_t variable = level_filter();
    Mahony_Euler_t uniform_euler;
    Mahony_Euler_t variable_euler;
    float rates[3] = {0.0f, 0.0f, 0.0f};
    unsigned int i;
    rates[axis] = 30.0f * DEG_TO_RAD;
    for (i = 0U; i < 1000U; ++i)
    {
      /* Zero accel skips correction, leaving pure gyro integration. */
      assert(Mahony_Update(&uniform, rates[0], rates[1], rates[2],
                           0.0f, 0.0f, 0.0f, 0.001f));
      assert(Mahony_Update(&variable, rates[0], rates[1], rates[2],
                           0.0f, 0.0f, 0.0f,
                           (i % 2U == 0U) ? 0.0005f : 0.0015f));
    }
    assert(Mahony_GetEulerDegrees(&uniform, &uniform_euler));
    assert(Mahony_GetEulerDegrees(&variable, &variable_euler));
    assert_near(uniform_euler.roll, axis == 0U ? 30.0f : 0.0f, 0.002f);
    assert_near(uniform_euler.pitch, axis == 1U ? 30.0f : 0.0f, 0.002f);
    assert_near(uniform_euler.yaw, axis == 2U ? 30.0f : 0.0f, 0.002f);
    assert_near(variable_euler.roll, uniform_euler.roll, 0.002f);
    assert_near(variable_euler.pitch, uniform_euler.pitch, 0.002f);
    assert_near(variable_euler.yaw, uniform_euler.yaw, 0.002f);
  }
}

static void test_gravity_correction_and_dynamic_accel_gate(void)
{
  Mahony_Handle_t filter;
  Mahony_Euler_t euler;
  Mahony_Config_t config = test_config;
  const float roll = 25.0f * DEG_TO_RAD;
  unsigned int i;

  config.ki = 0.0f;
  Mahony_Init(&filter, &config);
  assert(Mahony_InitFromAccel(&filter, 0.0f, sinf(roll), cosf(roll)));
  /* Strong linear acceleration must not be interpreted as tilt correction. */
  assert(Mahony_Update(&filter, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.001f));
  assert(Mahony_GetEulerDegrees(&filter, &euler));
  assert_near(euler.roll, 25.0f, 0.0001f);
  for (i = 0U; i < 4000U; ++i)
  {
    assert(Mahony_Update(&filter, 0.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 0.001f));
  }
  assert(Mahony_GetEulerDegrees(&filter, &euler));
  assert_near(euler.roll, 0.0f, 0.02f);
}

static void test_invalid_sample_and_stall_invalidate_until_reinit(void)
{
  const float bad_dt[] = {0.0f, -0.001f, 0.0004f, 0.0201f, 1.0f, NAN, INFINITY};
  const float bad_values[] = {NAN, INFINITY, -INFINITY};
  Mahony_Euler_t euler;
  Mahony_Handle_t filter;
  size_t i;
  size_t j;
  size_t k;

  for (i = 0U; i < sizeof(bad_dt) / sizeof(bad_dt[0]); ++i)
  {
    filter = level_filter();
    assert(!Mahony_Update(&filter, 0.0f, 0.0f, 0.0f,
                          0.0f, 0.0f, 1.0f, bad_dt[i]));
    assert(!filter.initialized);
    assert(!Mahony_GetEulerDegrees(&filter, &euler));
    assert(!Mahony_Update(&filter, 0.0f, 0.0f, 0.0f,
                          0.0f, 0.0f, 1.0f, 0.001f));
    assert(Mahony_InitFromAccel(&filter, 0.0f, 0.0f, 1.0f));
    assert(Mahony_Update(&filter, 0.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 0.001f));
  }
  for (j = 0U; j < sizeof(bad_values) / sizeof(bad_values[0]); ++j)
  {
    for (k = 0U; k < 6U; ++k)
    {
      float sample[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
      sample[k] = bad_values[j];
      filter = level_filter();
      assert(!Mahony_Update(&filter, sample[0], sample[1], sample[2],
                            sample[3], sample[4], sample[5], 0.001f));
      assert(!filter.initialized);
    }
  }
  /* Finite components can still overflow vector/quaternion norms. */
  filter = level_filter();
  assert(!Mahony_Update(&filter, FLT_MAX, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.001f));
  assert(!filter.initialized);
  filter = level_filter();
  assert(!Mahony_Update(&filter, 0.0f, 0.0f, 0.0f,
                        FLT_MAX, 0.0f, 1.0f, 0.001f));
  assert(!filter.initialized);
  filter = level_filter();
  assert(Mahony_Update(&filter, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                       MAHONY_MIN_DT_S));
  assert(Mahony_Update(&filter, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                       MAHONY_MAX_DT_S));
}

static void test_corrupt_state_cannot_publish_valid_euler(void)
{
  const float corrupt_quaternion[] = {NAN, INFINITY, 0.0f, 2.0f};
  Mahony_Handle_t filter;
  Mahony_Euler_t euler = {123.0f, 456.0f, 789.0f};
  size_t i;

  for (i = 0U; i < sizeof(corrupt_quaternion) / sizeof(corrupt_quaternion[0]); ++i)
  {
    filter = level_filter();
    filter.q0 = corrupt_quaternion[i];
    assert(!Mahony_GetEulerDegrees(&filter, &euler));
    assert_near(euler.roll, 123.0f, 0.001f);
    assert(!Mahony_Update(&filter, 0.0f, 0.0f, 0.0f,
                          0.0f, 0.0f, 1.0f, 0.001f));
    assert(!filter.initialized);
  }
  filter = level_filter();
  filter.integral_x = NAN;
  assert(!Mahony_GetEulerDegrees(&filter, &euler));
  filter = level_filter();
  filter.config.kp = INFINITY;
  assert(!Mahony_GetEulerDegrees(&filter, &euler));
  assert(!Mahony_GetEulerDegrees(NULL, &euler));
  assert(!Mahony_GetEulerDegrees(&filter, NULL));
}

int main(void)
{
  test_gravity_initialization_signs_without_magnetometer();
  test_init_requires_gravity_and_valid_config();
  test_body_rate_integration_units_and_variable_dt();
  test_gravity_correction_and_dynamic_accel_gate();
  test_invalid_sample_and_stall_invalidate_until_reinit();
  test_corrupt_state_cannot_publish_valid_euler();
  puts("mahony6 tests: PASS");
  return 0;
}
