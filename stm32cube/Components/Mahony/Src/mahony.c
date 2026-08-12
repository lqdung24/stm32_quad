#include "mahony.h"

#include <math.h>
#include <stddef.h>

#define MAHONY_RAD_TO_DEG 57.2957795f
#define MAHONY_MIN_VECTOR_NORM 0.000001f

static float Mahony_Clamp(float value, float minimum, float maximum);

void Mahony_Init(Mahony_Handle_t *filter, const Mahony_Config_t *config)
{
  if ((filter == NULL) || (config == NULL))
  {
    return;
  }

  filter->q0 = 1.0f;
  filter->q1 = 0.0f;
  filter->q2 = 0.0f;
  filter->q3 = 0.0f;
  filter->integral_x = 0.0f;
  filter->integral_y = 0.0f;
  filter->integral_z = 0.0f;
  filter->config = *config;
  filter->initialized = false;
}

bool Mahony_InitFromAccel(Mahony_Handle_t *filter, float ax, float ay, float az)
{
  float norm;
  float roll;
  float pitch;
  float half_roll;
  float half_pitch;
  float cr;
  float sr;
  float cp;
  float sp;

  if (filter == NULL)
  {
    return false;
  }

  norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
  if (norm < MAHONY_MIN_VECTOR_NORM)
  {
    return false;
  }

  ax /= norm;
  ay /= norm;
  az /= norm;
  roll = atan2f(ay, az);
  pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az)));
  half_roll = 0.5f * roll;
  half_pitch = 0.5f * pitch;
  cr = cosf(half_roll);
  sr = sinf(half_roll);
  cp = cosf(half_pitch);
  sp = sinf(half_pitch);

  filter->q0 = cr * cp;
  filter->q1 = sr * cp;
  filter->q2 = cr * sp;
  filter->q3 = -sr * sp;
  filter->integral_x = 0.0f;
  filter->integral_y = 0.0f;
  filter->integral_z = 0.0f;
  filter->initialized = true;

  return true;
}

bool Mahony_Update(Mahony_Handle_t *filter,
                   float gx_rad_s, float gy_rad_s, float gz_rad_s,
                   float ax, float ay, float az, float dt_s)
{
  float accel_norm;
  float quaternion_norm;
  float vx;
  float vy;
  float vz;
  float ex;
  float ey;
  float ez;
  float q0;
  float q1;
  float q2;
  float q3;

  if ((filter == NULL) || (!filter->initialized) || (dt_s <= 0.0f))
  {
    return false;
  }

  accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
  if ((accel_norm > filter->config.accel_min_norm) &&
      (accel_norm < filter->config.accel_max_norm))
  {
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;

    vx = 2.0f * ((filter->q1 * filter->q3) - (filter->q0 * filter->q2));
    vy = 2.0f * ((filter->q0 * filter->q1) + (filter->q2 * filter->q3));
    vz = (filter->q0 * filter->q0) - (filter->q1 * filter->q1) -
         (filter->q2 * filter->q2) + (filter->q3 * filter->q3);

    ex = (ay * vz) - (az * vy);
    ey = (az * vx) - (ax * vz);
    ez = (ax * vy) - (ay * vx);

    filter->integral_x = Mahony_Clamp(filter->integral_x + (filter->config.ki * ex * dt_s),
                                      -filter->config.integral_limit_rad_s,
                                      filter->config.integral_limit_rad_s);
    filter->integral_y = Mahony_Clamp(filter->integral_y + (filter->config.ki * ey * dt_s),
                                      -filter->config.integral_limit_rad_s,
                                      filter->config.integral_limit_rad_s);
    filter->integral_z = Mahony_Clamp(filter->integral_z + (filter->config.ki * ez * dt_s),
                                      -filter->config.integral_limit_rad_s,
                                      filter->config.integral_limit_rad_s);

    gx_rad_s += (filter->config.kp * ex) + filter->integral_x;
    gy_rad_s += (filter->config.kp * ey) + filter->integral_y;
    gz_rad_s += (filter->config.kp * ez) + filter->integral_z;
  }

  q0 = filter->q0;
  q1 = filter->q1;
  q2 = filter->q2;
  q3 = filter->q3;
  filter->q0 += 0.5f * ((-q1 * gx_rad_s) - (q2 * gy_rad_s) - (q3 * gz_rad_s)) * dt_s;
  filter->q1 += 0.5f * ((q0 * gx_rad_s) + (q2 * gz_rad_s) - (q3 * gy_rad_s)) * dt_s;
  filter->q2 += 0.5f * ((q0 * gy_rad_s) - (q1 * gz_rad_s) + (q3 * gx_rad_s)) * dt_s;
  filter->q3 += 0.5f * ((q0 * gz_rad_s) + (q1 * gy_rad_s) - (q2 * gx_rad_s)) * dt_s;

  quaternion_norm = sqrtf((filter->q0 * filter->q0) + (filter->q1 * filter->q1) +
                          (filter->q2 * filter->q2) + (filter->q3 * filter->q3));
  if (quaternion_norm < MAHONY_MIN_VECTOR_NORM)
  {
    filter->initialized = false;
    return false;
  }

  filter->q0 /= quaternion_norm;
  filter->q1 /= quaternion_norm;
  filter->q2 /= quaternion_norm;
  filter->q3 /= quaternion_norm;

  return true;
}

bool Mahony_GetEulerDegrees(const Mahony_Handle_t *filter, Mahony_Euler_t *euler)
{
  float sin_pitch;

  if ((filter == NULL) || (euler == NULL) || (!filter->initialized))
  {
    return false;
  }

  euler->roll = atan2f(2.0f * ((filter->q0 * filter->q1) + (filter->q2 * filter->q3)),
                       1.0f - (2.0f * ((filter->q1 * filter->q1) + (filter->q2 * filter->q2)))) *
                MAHONY_RAD_TO_DEG;

  sin_pitch = 2.0f * ((filter->q0 * filter->q2) - (filter->q3 * filter->q1));
  euler->pitch = asinf(Mahony_Clamp(sin_pitch, -1.0f, 1.0f)) * MAHONY_RAD_TO_DEG;

  euler->yaw = atan2f(2.0f * ((filter->q0 * filter->q3) + (filter->q1 * filter->q2)),
                      1.0f - (2.0f * ((filter->q2 * filter->q2) + (filter->q3 * filter->q3)))) *
               MAHONY_RAD_TO_DEG;

  return true;
}

static float Mahony_Clamp(float value, float minimum, float maximum)
{
  if (value > maximum)
  {
    return maximum;
  }
  if (value < minimum)
  {
    return minimum;
  }
  return value;
}
