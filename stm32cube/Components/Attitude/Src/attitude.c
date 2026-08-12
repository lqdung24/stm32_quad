#include "attitude.h"

#define ATTITUDE_DEG_TO_RAD 0.0174532925f

/*
 * Six-position accelerometer calibration in BODY FRD.
 *
 * Stable pose means in mg:
 *   X+: +1020.326, X-:  -979.584
 *   Y+:  +986.720, Y-: -1014.639
 *   Z+:  +993.538, Z-: -1026.697
 *
 * calibrated = (measured - bias) * scale
 */
static const Attitude_Vector3f_t attitude_accel_bias_g = {
  0.020371f, -0.013960f, -0.016580f
};
static const Attitude_Vector3f_t attitude_accel_scale = {
  1.000045f, 0.999321f, 0.989984f
};

/* Fitted from the 1300-sample full-3D tumble log. */
static const Attitude_Vector3f_t attitude_mag_offset_cuT = {
  5015.6891f, -7402.7585f, 11587.2894f
};
static const float attitude_mag_soft_iron[3][3] = {
  { 0.9869328f, -0.0209696f, -0.0218734f },
  {-0.0209696f,  0.9865160f, -0.0130036f },
  {-0.0218734f, -0.0130036f,  1.0258979f }
};

Attitude_VectorRaw_t Attitude_MapSensorRawToBody(Attitude_VectorRaw_t sensor_raw)
{
  Attitude_VectorRaw_t body_raw;

  body_raw.x = -sensor_raw.x;
  body_raw.y = sensor_raw.y;
  body_raw.z = -sensor_raw.z;

  return body_raw;
}

Attitude_VectorRaw_t Attitude_MapMagSensorRawToBody(Attitude_VectorRaw_t sensor_raw)
{
  Attitude_VectorRaw_t body_raw;

  body_raw.x = -sensor_raw.x;
  body_raw.y = -sensor_raw.y;
  body_raw.z = sensor_raw.z;

  return body_raw;
}

Attitude_VectorRaw_t Attitude_CalibrateMagBodyCentiUt(Attitude_VectorRaw_t mag_body_raw)
{
  float x = (float)mag_body_raw.x - attitude_mag_offset_cuT.x;
  float y = (float)mag_body_raw.y - attitude_mag_offset_cuT.y;
  float z = (float)mag_body_raw.z - attitude_mag_offset_cuT.z;
  Attitude_VectorRaw_t calibrated;

  calibrated.x = (int32_t)(attitude_mag_soft_iron[0][0] * x +
                           attitude_mag_soft_iron[0][1] * y +
                           attitude_mag_soft_iron[0][2] * z);
  calibrated.y = (int32_t)(attitude_mag_soft_iron[1][0] * x +
                           attitude_mag_soft_iron[1][1] * y +
                           attitude_mag_soft_iron[1][2] * z);
  calibrated.z = (int32_t)(attitude_mag_soft_iron[2][0] * x +
                           attitude_mag_soft_iron[2][1] * y +
                           attitude_mag_soft_iron[2][2] * z);
  return calibrated;
}

Attitude_Vector3f_t Attitude_CalibrateAccelBodyG(Attitude_Vector3f_t accel_body_g)
{
  Attitude_Vector3f_t calibrated;

  calibrated.x = (accel_body_g.x - attitude_accel_bias_g.x) *
                 attitude_accel_scale.x;
  calibrated.y = (accel_body_g.y - attitude_accel_bias_g.y) *
                 attitude_accel_scale.y;
  calibrated.z = (accel_body_g.z - attitude_accel_bias_g.z) *
                 attitude_accel_scale.z;

  return calibrated;
}

Attitude_Vector3f_t Attitude_AccelRawToBodyG(Attitude_VectorRaw_t sensor_raw,
                                             float lsb_per_g)
{
  Attitude_VectorRaw_t body_raw = Attitude_MapSensorRawToBody(sensor_raw);
  Attitude_Vector3f_t body_g = {0.0f, 0.0f, 0.0f};

  if (lsb_per_g > 0.0f)
  {
    body_g.x = (float)body_raw.x / lsb_per_g;
    body_g.y = (float)body_raw.y / lsb_per_g;
    body_g.z = (float)body_raw.z / lsb_per_g;
  }

  return Attitude_CalibrateAccelBodyG(body_g);
}

Attitude_Vector3f_t Attitude_GyroRawToBodyRadS(Attitude_VectorRaw_t sensor_raw,
                                               Attitude_VectorRaw_t sensor_bias_raw,
                                               float lsb_per_deg_s)
{
  Attitude_VectorRaw_t corrected_sensor_raw;
  Attitude_VectorRaw_t body_raw;
  Attitude_Vector3f_t body_rad_s = {0.0f, 0.0f, 0.0f};

  corrected_sensor_raw.x = sensor_raw.x - sensor_bias_raw.x;
  corrected_sensor_raw.y = sensor_raw.y - sensor_bias_raw.y;
  corrected_sensor_raw.z = sensor_raw.z - sensor_bias_raw.z;
  body_raw = Attitude_MapSensorRawToBody(corrected_sensor_raw);

  if (lsb_per_deg_s > 0.0f)
  {
    body_rad_s.x = ((float)body_raw.x / lsb_per_deg_s) * ATTITUDE_DEG_TO_RAD;
    body_rad_s.y = ((float)body_raw.y / lsb_per_deg_s) * ATTITUDE_DEG_TO_RAD;
    body_rad_s.z = ((float)body_raw.z / lsb_per_deg_s) * ATTITUDE_DEG_TO_RAD;
  }

  return body_rad_s;
}

Attitude_Vector3f_t Attitude_SpecificForceToGravity(Attitude_Vector3f_t specific_force_body)
{
  Attitude_Vector3f_t gravity_body;

  gravity_body.x = -specific_force_body.x;
  gravity_body.y = -specific_force_body.y;
  gravity_body.z = -specific_force_body.z;

  return gravity_body;
}
