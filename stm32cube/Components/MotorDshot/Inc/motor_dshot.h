#ifndef MOTOR_DSHOT_H
#define MOTOR_DSHOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../../Motor/Inc/motor.h"
#include "../../PwmTimer/Inc/pwm_timer.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_DSHOT_MOTOR_COUNT       4U
#define MOTOR_DSHOT_PACKET_BITS      16U
#define MOTOR_DSHOT_RESET_SLOTS       2U
#define MOTOR_DSHOT_FRAME_SLOTS      \
  (MOTOR_DSHOT_PACKET_BITS + MOTOR_DSHOT_RESET_SLOTS)
#define MOTOR_DSHOT_DMA_BUFFER_WORDS \
  (MOTOR_DSHOT_FRAME_SLOTS * MOTOR_DSHOT_MOTOR_COUNT)

#define MOTOR_DSHOT300_BITRATE_HZ 300000U
#define MOTOR_DSHOT600_BITRATE_HZ 600000U
#define MOTOR_DSHOT_MIN_THROTTLE_VALUE 48U
#define MOTOR_DSHOT_MAX_THROTTLE_VALUE 2047U

typedef struct
{
  PwmTimer *timer;
  PwmChannel *channel[MOTOR_DSHOT_MOTOR_COUNT];
  uint32_t bitrate_hz;
  uint16_t minimum_throttle_value;
  uint16_t maximum_throttle_value;
  bool telemetry;
} MotorDshot_Config;

typedef struct
{
  MotorDshot_Config config;
  uint32_t dma_buffer[MOTOR_DSHOT_DMA_BUFFER_WORDS]
      __attribute__((aligned(32)));
  float requested_throttle[MOTOR_DSHOT_MOTOR_COUNT];
  float applied_throttle[MOTOR_DSHOT_MOTOR_COUNT];
  uint16_t throttle_value[MOTOR_DSHOT_MOTOR_COUNT];
  uint32_t zero_compare_ticks;
  uint32_t one_compare_ticks;
  bool initialized;
  bool started;
} MotorDshot;

MotorStatus MotorDshot_Init(MotorDshot *driver,
                            MotorOutput *output,
                            const MotorDshot_Config *config);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DSHOT_H */
