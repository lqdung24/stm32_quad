#include "../Inc/rate_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RATE_CONTROL_MIN_DT_S 0.0005f
#define RATE_CONTROL_MAX_DT_S 0.050f
#define RATE_CONTROL_TWO_PI   6.28318530718f
#define RATE_CONTROL_DEG_TO_RAD 0.01745329252f

static bool config_valid(const RateControlConfig *config);
static float clampf(float value, float minimum, float maximum);
static void clear_dynamic_state(RateControl *control);
static float update_axis(RateControl *control,
                         uint8_t axis,
                         float measurement,
                         float dt_s);

bool RateControl_Init(RateControl *control,
                      const RateControlConfig *config)
{
  if ((control == NULL) || !config_valid(config))
  {
    return false;
  }

  memset(control, 0, sizeof(*control));
  control->config = *config;
  control->initialized = true;
  return true;
}

void RateControl_Reset(RateControl *control)
{
  if ((control == NULL) || !control->initialized)
  {
    return;
  }

  memset(&control->debug, 0, sizeof(control->debug));
  clear_dynamic_state(control);
}

void RateControl_SetCommand(RateControl *control,
                            int16_t roll,
                            int16_t pitch,
                            int16_t yaw)
{
  const int16_t command[RATE_CONTROL_AXIS_COUNT] = {
      roll,
      pitch,
      yaw,
  };
  uint8_t axis;

  if ((control == NULL) || !control->initialized)
  {
    return;
  }

  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    const float normalized =
        clampf((float)command[axis],
               -(float)RATE_CONTROL_COMMAND_LIMIT,
               (float)RATE_CONTROL_COMMAND_LIMIT) /
        (float)RATE_CONTROL_COMMAND_LIMIT;
    control->debug.target_rad_s[axis] =
        normalized * control->config.maximum_rate_rad_s[axis];
  }
}

bool RateControl_SetTargetRates(
    RateControl *control,
    const float target_rad_s[RATE_CONTROL_AXIS_COUNT])
{
  uint8_t axis;

  if ((control == NULL) || !control->initialized)
  {
    return false;
  }
  if (target_rad_s == NULL)
  {
    RateControl_Reset(control);
    return false;
  }
  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    if (!isfinite(target_rad_s[axis]))
    {
      RateControl_Reset(control);
      return false;
    }
  }
  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    control->debug.target_rad_s[axis] =
        clampf(target_rad_s[axis],
               -control->config.maximum_rate_rad_s[axis],
               control->config.maximum_rate_rad_s[axis]);
  }
  return true;
}

bool RateControl_SetAngleCommand(RateControl *control,
                                 int16_t roll,
                                 int16_t pitch,
                                 int16_t yaw,
                                 float measured_roll_deg,
                                 float measured_pitch_deg)
{
  float current[4];
  float desired[4];
  float error[4];
  float target_rad_s[RATE_CONTROL_AXIS_COUNT];
  float half_roll;
  float half_pitch;
  float cr;
  float sr;
  float cp;
  float sp;
  float norm;
  float leveling_rate;
  uint8_t component;

  if ((control == NULL) || !control->initialized)
  {
    return false;
  }
  if (!isfinite(measured_roll_deg) || !isfinite(measured_pitch_deg) ||
      (fabsf(measured_roll_deg) > 180.0f) ||
      (fabsf(measured_pitch_deg) > 90.0f))
  {
    RateControl_Reset(control);
    return false;
  }

  half_roll = 0.5f * measured_roll_deg * RATE_CONTROL_DEG_TO_RAD;
  half_pitch = 0.5f * measured_pitch_deg * RATE_CONTROL_DEG_TO_RAD;
  cr = cosf(half_roll);
  sr = sinf(half_roll);
  cp = cosf(half_pitch);
  sp = sinf(half_pitch);
  current[0] = cp * cr;
  current[1] = cp * sr;
  current[2] = sp * cr;
  current[3] = -sp * sr;

  half_roll = 0.5f * RATE_CONTROL_ANGLE_MAX_DEG * RATE_CONTROL_DEG_TO_RAD *
      clampf((float)roll / RATE_CONTROL_COMMAND_LIMIT, -1.0f, 1.0f);
  half_pitch = 0.5f * RATE_CONTROL_ANGLE_MAX_DEG * RATE_CONTROL_DEG_TO_RAD *
      clampf((float)pitch / RATE_CONTROL_COMMAND_LIMIT, -1.0f, 1.0f);
  cr = cosf(half_roll);
  sr = sinf(half_roll);
  cp = cosf(half_pitch);
  sp = sinf(half_pitch);
  desired[0] = cp * cr;
  desired[1] = cp * sr;
  desired[2] = sp * cr;
  desired[3] = -sp * sr;

  /*
   * q_error = conjugate(q_current) * q_desired: rotation in BODY axes.
   * Both attitudes use the same yaw, which cancels from their relative
   * quaternion. This avoids reliance on unobservable Mahony6 yaw heading.
   */
  error[0] = current[0] * desired[0] + current[1] * desired[1] +
             current[2] * desired[2] + current[3] * desired[3];
  error[1] = current[0] * desired[1] - current[1] * desired[0] -
             current[2] * desired[3] + current[3] * desired[2];
  error[2] = current[0] * desired[2] + current[1] * desired[3] -
             current[2] * desired[0] - current[3] * desired[1];
  error[3] = current[0] * desired[3] - current[1] * desired[2] +
             current[2] * desired[1] - current[3] * desired[0];
  norm = sqrtf(error[0] * error[0] + error[1] * error[1] +
               error[2] * error[2] + error[3] * error[3]);
  if (!isfinite(norm) || (norm <= 0.0f))
  {
    RateControl_Reset(control);
    return false;
  }
  if (error[0] < 0.0f)
  {
    norm = -norm;
  }
  for (component = 0U; component < 4U; ++component)
  {
    error[component] /= norm;
  }
  target_rad_s[RATE_CONTROL_ROLL] =
      2.0f * RATE_CONTROL_ANGLE_KP_PER_S * error[1];
  target_rad_s[RATE_CONTROL_PITCH] =
      2.0f * RATE_CONTROL_ANGLE_KP_PER_S * error[2];
  leveling_rate = hypotf(target_rad_s[RATE_CONTROL_ROLL],
                         target_rad_s[RATE_CONTROL_PITCH]);
  if (leveling_rate > RATE_CONTROL_ANGLE_MAX_RATE_RAD_S)
  {
    const float scale = RATE_CONTROL_ANGLE_MAX_RATE_RAD_S / leveling_rate;
    target_rad_s[RATE_CONTROL_ROLL] *= scale;
    target_rad_s[RATE_CONTROL_PITCH] *= scale;
  }
  target_rad_s[RATE_CONTROL_YAW] =
      clampf((float)yaw / RATE_CONTROL_COMMAND_LIMIT, -1.0f, 1.0f) *
      control->config.maximum_rate_rad_s[RATE_CONTROL_YAW];
  return RateControl_SetTargetRates(control, target_rad_s);
}

bool RateControl_Update(RateControl *control,
                        const float measured_rad_s[RATE_CONTROL_AXIS_COUNT],
                        float dt_s)
{
  uint8_t axis;

  if ((control == NULL) ||
      (measured_rad_s == NULL) ||
      !control->initialized ||
      !isfinite(dt_s) ||
      (dt_s < RATE_CONTROL_MIN_DT_S) ||
      (dt_s > RATE_CONTROL_MAX_DT_S))
  {
    if ((control != NULL) && control->initialized)
    {
      clear_dynamic_state(control);
    }
    return false;
  }

  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    if (!isfinite(measured_rad_s[axis]))
    {
      clear_dynamic_state(control);
      return false;
    }
  }

  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    control->debug.measured_rad_s[axis] = measured_rad_s[axis];
    control->debug.output[axis] =
        update_axis(control, axis, measured_rad_s[axis], dt_s);
  }
  return true;
}

bool RateControl_GetDebug(const RateControl *control,
                          RateControlDebug *debug)
{
  if ((control == NULL) || (debug == NULL) || !control->initialized)
  {
    return false;
  }

  *debug = control->debug;
  return true;
}

static bool config_valid(const RateControlConfig *config)
{
  uint8_t axis;

  if (config == NULL)
  {
    return false;
  }

  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    const RatePidConfig *pid = &config->pid[axis];
    if (!isfinite(pid->kp) || (pid->kp < 0.0f) ||
        !isfinite(pid->ki) || (pid->ki < 0.0f) ||
        !isfinite(pid->kd) || (pid->kd < 0.0f) ||
        !isfinite(pid->integral_limit) || (pid->integral_limit < 0.0f) ||
        !isfinite(pid->output_limit) || (pid->output_limit <= 0.0f) ||
        !isfinite(pid->derivative_cutoff_hz) ||
        (pid->derivative_cutoff_hz < 0.0f) ||
        !isfinite(config->maximum_rate_rad_s[axis]) ||
        (config->maximum_rate_rad_s[axis] <= 0.0f))
    {
      return false;
    }
  }
  return true;
}

static float clampf(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static void clear_dynamic_state(RateControl *control)
{
  memset(control->integral, 0, sizeof(control->integral));
  memset(control->previous_measurement, 0,
         sizeof(control->previous_measurement));
  memset(control->filtered_derivative, 0,
         sizeof(control->filtered_derivative));
  memset(control->derivative_initialized, 0,
         sizeof(control->derivative_initialized));
  memset(control->debug.output, 0, sizeof(control->debug.output));
}

static float update_axis(RateControl *control,
                         uint8_t axis,
                         float measurement,
                         float dt_s)
{
  const RatePidConfig *pid = &control->config.pid[axis];
  const float error = control->debug.target_rad_s[axis] - measurement;
  float derivative = 0.0f;
  float derivative_term;
  float candidate_integral;
  float candidate_output;
  float output;

  if (control->derivative_initialized[axis])
  {
    const float raw_derivative =
        (measurement - control->previous_measurement[axis]) / dt_s;
    if (pid->derivative_cutoff_hz > 0.0f)
    {
      const float rc =
          1.0f / (RATE_CONTROL_TWO_PI * pid->derivative_cutoff_hz);
      const float alpha = dt_s / (rc + dt_s);
      control->filtered_derivative[axis] +=
          alpha * (raw_derivative - control->filtered_derivative[axis]);
      derivative = control->filtered_derivative[axis];
    }
    else
    {
      derivative = raw_derivative;
      control->filtered_derivative[axis] = raw_derivative;
    }
  }
  else
  {
    control->derivative_initialized[axis] = true;
  }
  control->previous_measurement[axis] = measurement;

  /* Derivative-on-measurement avoids a kick when the pilot moves the stick. */
  derivative_term = -pid->kd * derivative;
  candidate_integral =
      clampf(control->integral[axis] + (pid->ki * error * dt_s),
             -pid->integral_limit,
             pid->integral_limit);
  candidate_output =
      (pid->kp * error) + candidate_integral + derivative_term;

  /*
   * Conditional integration: do not wind the integrator farther into an
   * output limit. Integration in the direction that leaves saturation is
   * still allowed.
   */
  if (!(((candidate_output > pid->output_limit) && (error > 0.0f)) ||
        ((candidate_output < -pid->output_limit) && (error < 0.0f))))
  {
    control->integral[axis] = candidate_integral;
  }

  output = (pid->kp * error) +
           control->integral[axis] +
           derivative_term;
  return clampf(output, -pid->output_limit, pid->output_limit);
}
