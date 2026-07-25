#ifndef MAHONY_H
#define MAHONY_H

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
} Mahony_Config_t;

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
  float integral_x;
  float integral_y;
  float integral_z;
  Mahony_Config_t config;
  bool initialized;
} Mahony_Handle_t;

typedef struct
{
  float roll;
  float pitch;
  float yaw;
} Mahony_Euler_t;

void Mahony_Init(Mahony_Handle_t *filter, const Mahony_Config_t *config);
bool Mahony_InitFromAccel(Mahony_Handle_t *filter, float ax, float ay, float az);
bool Mahony_Update(Mahony_Handle_t *filter,
                   float gx_rad_s, float gy_rad_s, float gz_rad_s,
                   float ax, float ay, float az, float dt_s);
bool Mahony_GetEulerDegrees(const Mahony_Handle_t *filter, Mahony_Euler_t *euler);

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_H */
