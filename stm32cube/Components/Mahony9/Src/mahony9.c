#include "mahony9.h"

#include <math.h>
#include <stddef.h>

#define MAHONY9_RAD_TO_DEG       57.2957795f
#define MAHONY9_MIN_VECTOR_NORM  0.000001f

static bool Mahony9_UpdateInternal(Mahony9_Handle_t *filter,
                                   float gx_rad_s, float gy_rad_s, float gz_rad_s,
                                   float ax, float ay, float az,
                                   float mx, float my, float mz,
                                   bool allow_magnetometer,
                                   float dt_s);
static bool Mahony9_NormAccepted(float norm, float minimum, float maximum);
static float Mahony9_Clamp(float value, float minimum, float maximum);

void Mahony9_Init(Mahony9_Handle_t *filter, const Mahony9_Config_t *config)
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
  filter->magnetometer_used = false;
}

bool Mahony9_InitFromAccelMag(Mahony9_Handle_t *filter,
                              float ax, float ay, float az,
                              float mx, float my, float mz)
{
  float accel_norm;
  float mag_norm;
  float roll;
  float pitch;
  float half_roll;
  float half_pitch;
  float cr;
  float sr;
  float cp;
  float sp;
  float tilt_q0;
  float tilt_q1;
  float tilt_q2;
  float tilt_q3;
  float hx;
  float hy;
  float yaw;
  float half_yaw;
  float cy;
  float sy;
  float quaternion_norm;

  if (filter == NULL)
  {
    return false;
  }

  accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
  mag_norm = sqrtf((mx * mx) + (my * my) + (mz * mz));
  if (!Mahony9_NormAccepted(accel_norm,
                            filter->config.accel_min_norm,
                            filter->config.accel_max_norm) ||
      !Mahony9_NormAccepted(mag_norm,
                            filter->config.mag_min_norm,
                            filter->config.mag_max_norm))
  {
    return false;
  }

  ax /= accel_norm;
  ay /= accel_norm;
  az /= accel_norm;
  mx /= mag_norm;
  my /= mag_norm;
  mz /= mag_norm;

  roll = atan2f(ay, az);
  pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az)));

  half_roll = 0.5f * roll;
  half_pitch = 0.5f * pitch;
  cr = cosf(half_roll);
  sr = sinf(half_roll);
  cp = cosf(half_pitch);
  sp = sinf(half_pitch);

  /* q_nb with yaw=0, used only to tilt-compensate the magnetic vector. */
  tilt_q0 = cr * cp;
  tilt_q1 = sr * cp;
  tilt_q2 = cr * sp;
  tilt_q3 = -sr * sp;

  hx = (1.0f - (2.0f * ((tilt_q2 * tilt_q2) + (tilt_q3 * tilt_q3)))) * mx +
       (2.0f * ((tilt_q1 * tilt_q2) - (tilt_q0 * tilt_q3))) * my +
       (2.0f * ((tilt_q1 * tilt_q3) + (tilt_q0 * tilt_q2))) * mz;
  hy = (2.0f * ((tilt_q1 * tilt_q2) + (tilt_q0 * tilt_q3))) * mx +
       (1.0f - (2.0f * ((tilt_q1 * tilt_q1) + (tilt_q3 * tilt_q3)))) * my +
       (2.0f * ((tilt_q2 * tilt_q3) - (tilt_q0 * tilt_q1))) * mz;

  if (sqrtf((hx * hx) + (hy * hy)) < MAHONY9_MIN_VECTOR_NORM)
  {
    return false;
  }

  /* Positive NED yaw is clockwise; East therefore corresponds to +90 deg. */
  yaw = atan2f(-hy, hx);
  half_yaw = 0.5f * yaw;
  cy = cosf(half_yaw);
  sy = sinf(half_yaw);

  /* q_nb = q_z(yaw) * q_y(pitch) * q_x(roll). */
  filter->q0 = (cr * cp * cy) + (sr * sp * sy);
  filter->q1 = (sr * cp * cy) - (cr * sp * sy);
  filter->q2 = (cr * sp * cy) + (sr * cp * sy);
  filter->q3 = (cr * cp * sy) - (sr * sp * cy);

  quaternion_norm = sqrtf((filter->q0 * filter->q0) +
                          (filter->q1 * filter->q1) +
                          (filter->q2 * filter->q2) +
                          (filter->q3 * filter->q3));
  if (quaternion_norm < MAHONY9_MIN_VECTOR_NORM)
  {
    return false;
  }

  filter->q0 /= quaternion_norm;
  filter->q1 /= quaternion_norm;
  filter->q2 /= quaternion_norm;
  filter->q3 /= quaternion_norm;
  filter->integral_x = 0.0f;
  filter->integral_y = 0.0f;
  filter->integral_z = 0.0f;
  filter->initialized = true;
  filter->magnetometer_used = true;

  return true;
}

bool Mahony9_Update(Mahony9_Handle_t *filter,
                    float gx_rad_s, float gy_rad_s, float gz_rad_s,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float dt_s)
{
  return Mahony9_UpdateInternal(filter,
                                gx_rad_s, gy_rad_s, gz_rad_s,
                                ax, ay, az,
                                mx, my, mz,
                                true,
                                dt_s);
}

bool Mahony9_UpdateImu(Mahony9_Handle_t *filter,
                       float gx_rad_s, float gy_rad_s, float gz_rad_s,
                       float ax, float ay, float az,
                       float dt_s)
{
  return Mahony9_UpdateInternal(filter,
                                gx_rad_s, gy_rad_s, gz_rad_s,
                                ax, ay, az,
                                0.0f, 0.0f, 0.0f,
                                false,
                                dt_s);
}

bool Mahony9_GetEulerDegrees(const Mahony9_Handle_t *filter,
                             Mahony9_Euler_t *euler)
{
  float sin_pitch;

  if ((filter == NULL) || (euler == NULL) || (!filter->initialized))
  {
    return false;
  }

  euler->roll = atan2f(2.0f * ((filter->q0 * filter->q1) +
                               (filter->q2 * filter->q3)),
                       1.0f - (2.0f * ((filter->q1 * filter->q1) +
                                      (filter->q2 * filter->q2)))) *
                MAHONY9_RAD_TO_DEG;

  sin_pitch = 2.0f * ((filter->q0 * filter->q2) -
                      (filter->q3 * filter->q1));
  euler->pitch = asinf(Mahony9_Clamp(sin_pitch, -1.0f, 1.0f)) *
                 MAHONY9_RAD_TO_DEG;

  euler->yaw = atan2f(2.0f * ((filter->q0 * filter->q3) +
                              (filter->q1 * filter->q2)),
                      1.0f - (2.0f * ((filter->q2 * filter->q2) +
                                     (filter->q3 * filter->q3)))) *
               MAHONY9_RAD_TO_DEG;

  return true;
}

static bool Mahony9_UpdateInternal(Mahony9_Handle_t *filter,
                                   float gx_rad_s, float gy_rad_s, float gz_rad_s,
                                   float ax, float ay, float az,
                                   float mx, float my, float mz,
                                   bool allow_magnetometer,
                                   float dt_s)
{
  float accel_norm;
  float mag_norm;
  bool accel_valid;
  bool mag_valid;
  bool reference_used = false;
  float vx;
  float vy;
  float vz;
  float hx;
  float hy;
  float hz;
  float bx;
  float bz;
  float wx;
  float wy;
  float wz;
  float ex = 0.0f;
  float ey = 0.0f;
  float ez = 0.0f;
  float q0;
  float q1;
  float q2;
  float q3;
  float quaternion_norm;

  if ((filter == NULL) || (!filter->initialized) || (dt_s <= 0.0f))
  {
    return false;
  }

  accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
  accel_valid = Mahony9_NormAccepted(accel_norm,
                                     filter->config.accel_min_norm,
                                     filter->config.accel_max_norm);
  if (accel_valid)
  {
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;

    /* Estimated NED-down direction expressed in BODY. */
    vx = 2.0f * ((filter->q1 * filter->q3) -
                 (filter->q0 * filter->q2));
    vy = 2.0f * ((filter->q0 * filter->q1) +
                 (filter->q2 * filter->q3));
    vz = (filter->q0 * filter->q0) - (filter->q1 * filter->q1) -
         (filter->q2 * filter->q2) + (filter->q3 * filter->q3);

    ex += (ay * vz) - (az * vy);
    ey += (az * vx) - (ax * vz);
    ez += (ax * vy) - (ay * vx);
    reference_used = true;
  }

  mag_norm = sqrtf((mx * mx) + (my * my) + (mz * mz));
  mag_valid = allow_magnetometer &&
              Mahony9_NormAccepted(mag_norm,
                                   filter->config.mag_min_norm,
                                   filter->config.mag_max_norm);
  filter->magnetometer_used = mag_valid;
  if (mag_valid)
  {
    mx /= mag_norm;
    my /= mag_norm;
    mz /= mag_norm;

    /* Rotate the measured field BODY -> NED. */
    hx = (1.0f - (2.0f * ((filter->q2 * filter->q2) +
                          (filter->q3 * filter->q3)))) * mx +
         (2.0f * ((filter->q1 * filter->q2) -
                  (filter->q0 * filter->q3))) * my +
         (2.0f * ((filter->q1 * filter->q3) +
                  (filter->q0 * filter->q2))) * mz;
    hy = (2.0f * ((filter->q1 * filter->q2) +
                  (filter->q0 * filter->q3))) * mx +
         (1.0f - (2.0f * ((filter->q1 * filter->q1) +
                          (filter->q3 * filter->q3)))) * my +
         (2.0f * ((filter->q2 * filter->q3) -
                  (filter->q0 * filter->q1))) * mz;
    hz = (2.0f * ((filter->q1 * filter->q3) -
                  (filter->q0 * filter->q2))) * mx +
         (2.0f * ((filter->q2 * filter->q3) +
                  (filter->q0 * filter->q1))) * my +
         ((filter->q0 * filter->q0) - (filter->q1 * filter->q1) -
          (filter->q2 * filter->q2) + (filter->q3 * filter->q3)) * mz;

    /* Magnetic-North reference in NED: [horizontal magnitude, 0, down]. */
    bx = sqrtf((hx * hx) + (hy * hy));
    bz = hz;

    /* Expected magnetic direction transformed NED -> BODY. */
    wx = (1.0f - (2.0f * ((filter->q2 * filter->q2) +
                          (filter->q3 * filter->q3)))) * bx +
         (2.0f * ((filter->q1 * filter->q3) -
                  (filter->q0 * filter->q2))) * bz;
    wy = (2.0f * ((filter->q1 * filter->q2) -
                  (filter->q0 * filter->q3))) * bx +
         (2.0f * ((filter->q2 * filter->q3) +
                  (filter->q0 * filter->q1))) * bz;
    wz = (2.0f * ((filter->q1 * filter->q3) +
                  (filter->q0 * filter->q2))) * bx +
         ((filter->q0 * filter->q0) - (filter->q1 * filter->q1) -
          (filter->q2 * filter->q2) + (filter->q3 * filter->q3)) * bz;

    ex += (my * wz) - (mz * wy);
    ey += (mz * wx) - (mx * wz);
    ez += (mx * wy) - (my * wx);
    reference_used = true;
  }

  if (reference_used)
  {
    filter->integral_x =
        Mahony9_Clamp(filter->integral_x + (filter->config.ki * ex * dt_s),
                      -filter->config.integral_limit_rad_s,
                      filter->config.integral_limit_rad_s);
    filter->integral_y =
        Mahony9_Clamp(filter->integral_y + (filter->config.ki * ey * dt_s),
                      -filter->config.integral_limit_rad_s,
                      filter->config.integral_limit_rad_s);
    filter->integral_z =
        Mahony9_Clamp(filter->integral_z + (filter->config.ki * ez * dt_s),
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
  filter->q0 += 0.5f * ((-q1 * gx_rad_s) -
                        (q2 * gy_rad_s) -
                        (q3 * gz_rad_s)) * dt_s;
  filter->q1 += 0.5f * ((q0 * gx_rad_s) +
                        (q2 * gz_rad_s) -
                        (q3 * gy_rad_s)) * dt_s;
  filter->q2 += 0.5f * ((q0 * gy_rad_s) -
                        (q1 * gz_rad_s) +
                        (q3 * gx_rad_s)) * dt_s;
  filter->q3 += 0.5f * ((q0 * gz_rad_s) +
                        (q1 * gy_rad_s) -
                        (q2 * gx_rad_s)) * dt_s;

  quaternion_norm = sqrtf((filter->q0 * filter->q0) +
                          (filter->q1 * filter->q1) +
                          (filter->q2 * filter->q2) +
                          (filter->q3 * filter->q3));
  if (quaternion_norm < MAHONY9_MIN_VECTOR_NORM)
  {
    filter->initialized = false;
    filter->magnetometer_used = false;
    return false;
  }

  filter->q0 /= quaternion_norm;
  filter->q1 /= quaternion_norm;
  filter->q2 /= quaternion_norm;
  filter->q3 /= quaternion_norm;

  return true;
}

static bool Mahony9_NormAccepted(float norm, float minimum, float maximum)
{
  if (norm < MAHONY9_MIN_VECTOR_NORM)
  {
    return false;
  }
  if ((minimum > 0.0f) && (norm <= minimum))
  {
    return false;
  }
  if ((maximum > minimum) && (norm >= maximum))
  {
    return false;
  }
  return true;
}

static float Mahony9_Clamp(float value, float minimum, float maximum)
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
