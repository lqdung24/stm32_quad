#ifndef MAHONY9_H
#define MAHONY9_H

/*
 * 9-axis Mahony attitude filter.
 *
 * Conventions:
 * - World: NED (X North, Y East, Z Down)
 * - Body:  FRD (X Forward, Y Right, Z Down)
 * - Quaternion: Hamilton, scalar first [w, x, y, z]
 * - q_nb rotates BODY vectors to NED
 * - Gyroscope input: BODY rad/s
 * - Accelerometer input: BODY gravity vector (not specific force)
 * - Magnetometer input: calibrated BODY magnetic vector
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct
{
  float kp;
  float ki;
  float integral_limit_rad_s;
  float accel_min_norm;
  float accel_max_norm;
  /* Magnetometer thresholds use the same unit supplied to Update/Init. */
  float mag_min_norm;
  float mag_max_norm;
} Mahony9_Config_t;

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
  float integral_x;
  float integral_y;
  float integral_z;
  Mahony9_Config_t config;
  bool initialized;
  bool magnetometer_used;
} Mahony9_Handle_t;

typedef struct
{
  float roll;
  float pitch;
  float yaw;
} Mahony9_Euler_t;

void Mahony9_Init(Mahony9_Handle_t *filter, const Mahony9_Config_t *config);

/*
 * Initializes roll/pitch from gravity and yaw from tilt-compensated magnetic
 * North. A level board pointing to magnetic North initializes q_nb to identity.
 */
bool Mahony9_InitFromAccelMag(Mahony9_Handle_t *filter,
                              float ax, float ay, float az,
                              float mx, float my, float mz);

/* Full 9-axis update. Invalid accel or mag vectors are rejected by norm gates. */
bool Mahony9_Update(Mahony9_Handle_t *filter,
                    float gx_rad_s, float gy_rad_s, float gz_rad_s,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float dt_s);

/* Gyro + accel fallback which preserves the same quaternion state. */
bool Mahony9_UpdateImu(Mahony9_Handle_t *filter,
                       float gx_rad_s, float gy_rad_s, float gz_rad_s,
                       float ax, float ay, float az,
                       float dt_s);

bool Mahony9_GetEulerDegrees(const Mahony9_Handle_t *filter,
                             Mahony9_Euler_t *euler);

#ifdef __cplusplus
}
#endif

#endif /* MAHONY9_H */
