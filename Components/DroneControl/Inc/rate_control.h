#ifndef RATE_CONTROL_H
#define RATE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define RATE_CONTROL_AXIS_COUNT 3U
#define RATE_CONTROL_COMMAND_LIMIT 1000

typedef enum
{
  RATE_CONTROL_ROLL = 0,
  RATE_CONTROL_PITCH = 1,
  RATE_CONTROL_YAW = 2
} RateControlAxis;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_limit;
  float output_limit;
  float derivative_cutoff_hz;
} RatePidConfig;

typedef struct
{
  RatePidConfig pid[RATE_CONTROL_AXIS_COUNT];
  float maximum_rate_rad_s[RATE_CONTROL_AXIS_COUNT];
} RateControlConfig;

typedef struct
{
  float target_rad_s[RATE_CONTROL_AXIS_COUNT];
  float measured_rad_s[RATE_CONTROL_AXIS_COUNT];
  float output[RATE_CONTROL_AXIS_COUNT];
} RateControlDebug;

typedef struct
{
  RateControlConfig config;
  RateControlDebug debug;
  float integral[RATE_CONTROL_AXIS_COUNT];
  float previous_measurement[RATE_CONTROL_AXIS_COUNT];
  float filtered_derivative[RATE_CONTROL_AXIS_COUNT];
  bool derivative_initialized[RATE_CONTROL_AXIS_COUNT];
  bool initialized;
} RateControl;

bool RateControl_Init(RateControl *control,
                      const RateControlConfig *config);
void RateControl_Reset(RateControl *control);
void RateControl_SetCommand(RateControl *control,
                            int16_t roll,
                            int16_t pitch,
                            int16_t yaw);
bool RateControl_Update(RateControl *control,
                        const float measured_rad_s[RATE_CONTROL_AXIS_COUNT],
                        float dt_s);
bool RateControl_GetDebug(const RateControl *control,
                          RateControlDebug *debug);

#ifdef __cplusplus
}
#endif

#endif /* RATE_CONTROL_H */
