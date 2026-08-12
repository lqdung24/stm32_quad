#ifndef ATTITUDE_H
#define ATTITUDE_H

/*
 * ATTITUDE CONVENTIONS
 *
 * World frame:
 * - NED: X North, Y East, Z Down
 *
 * Body frame:
 * - FRD: X Forward, Y Right, Z Down
 *
 * Handedness:
 * - Right-handed coordinate systems
 *
 * Quaternion:
 * - Hamilton convention
 * - Scalar first: [w, x, y, z]
 *
 * q_nb:
 * - Rotates vectors from BODY to NED
 *
 * Vector rotation:
 * - v_n = q_nb ⊗ v_b ⊗ conjugate(q_nb)
 *
 * Gyro:
 * - Expressed in BODY frame
 * - Units: rad/s
 *
 * Quaternion dynamics:
 * - q_dot = 0.5 * q_nb ⊗ [0, gyro_body]
 *
 * Euler:
 * - Intrinsic ZYX
 * - Positive roll: right wing down
 * - Positive pitch: nose up
 * - Positive yaw: clockwise viewed from above
 *
 * Level attitude facing North:
 * - q_nb = [1, 0, 0, 0]
 *
 * Installed IMU sensor frame -> BODY FRD:
 * Accelerometer and gyroscope:
 * - body_x = -sensor_x
 * - body_y =  sensor_y
 * - body_z = -sensor_z
 * - This is a 180-degree mounting rotation about +Y.
 *
 * Magnetometer:
 * - body_x = -sensor_x
 * - body_y = -sensor_y
 * - body_z =  sensor_z
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} Attitude_VectorRaw_t;

typedef struct
{
  float x;
  float y;
  float z;
} Attitude_Vector3f_t;

Attitude_VectorRaw_t Attitude_MapSensorRawToBody(Attitude_VectorRaw_t sensor_raw);
Attitude_VectorRaw_t Attitude_MapMagSensorRawToBody(Attitude_VectorRaw_t sensor_raw);
/* AK09916 calibration, input/output units: centi-microtesla (c_uT). */
Attitude_VectorRaw_t Attitude_CalibrateMagBodyCentiUt(Attitude_VectorRaw_t mag_body_raw);
/* Six-position diagonal calibration. Input/output units: g in BODY FRD. */
Attitude_Vector3f_t Attitude_CalibrateAccelBodyG(Attitude_Vector3f_t accel_body_g);
Attitude_Vector3f_t Attitude_AccelRawToBodyG(Attitude_VectorRaw_t sensor_raw,
                                             float lsb_per_g);
Attitude_Vector3f_t Attitude_GyroRawToBodyRadS(Attitude_VectorRaw_t sensor_raw,
                                               Attitude_VectorRaw_t sensor_bias_raw,
                                               float lsb_per_deg_s);
Attitude_Vector3f_t Attitude_SpecificForceToGravity(Attitude_Vector3f_t specific_force_body);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
