#ifndef MOTOR_DSHOT300_H
#define MOTOR_DSHOT300_H

#include "../../MotorDshot/Inc/motor_dshot.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DShot300 uses the shared encoder, DMA buffer and MotorOutput lifecycle.
 * Header-only adapter: no additional CubeMX-generated source list is needed. */
typedef MotorDshot MotorDshot300;
typedef MotorDshot_Config MotorDshot300_Config;

static inline MotorStatus MotorDshot300_Init(MotorDshot300 *driver,
                                             MotorOutput *output,
                                             const MotorDshot300_Config *config)
{
  if ((config == NULL) ||
      (config->bitrate_hz != MOTOR_DSHOT300_BITRATE_HZ))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  return MotorDshot_Init(driver, output, config);
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DSHOT300_H */
