#ifndef PWM_TIMER_H
#define PWM_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define PWM_TIMER_MAX_BURST_CHANNELS 4U

typedef enum
{
  PWM_TIMER_OK = 0,
  PWM_TIMER_BUSY,
  PWM_TIMER_INVALID_ARGUMENT,
  PWM_TIMER_HAL_ERROR
} PwmTimer_Status;

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t input_clock_hz;
  uint32_t counter_clock_hz;
  uint32_t period_ticks;
  bool auto_reload_preload;
} PwmTimer_Config;

typedef struct
{
  PwmTimer_Config config;
  volatile bool dma_busy;
  bool initialized;
} PwmTimer;

typedef struct
{
  PwmTimer *timer;
  uint32_t channel;
  uint32_t initial_compare_ticks;
  uint32_t polarity;
} PwmChannel_Config;

typedef struct
{
  PwmChannel_Config config;
  volatile bool dma_busy;
  bool initialized;
  bool started;
} PwmChannel;

/* Derive the APB timer kernel clock, including STM32H7 TIMPRE behavior. */
PwmTimer_Status PwmTimer_GetInputClockHz(const TIM_HandleTypeDef *htim,
                                         uint32_t *clock_hz);

/* A zero-initialized PwmTimer object may be initialized exactly once. */
PwmTimer_Status PwmTimer_Init(PwmTimer *timer,
                              const PwmTimer_Config *config);

PwmTimer_Status PwmChannel_Init(PwmChannel *channel,
                                const PwmChannel_Config *config);
PwmTimer_Status PwmChannel_Start(PwmChannel *channel);
PwmTimer_Status PwmChannel_Stop(PwmChannel *channel);
PwmTimer_Status PwmChannel_SetCompare(PwmChannel *channel,
                                      uint32_t compare_ticks);

/* The matching channel DMA request must already be configured by CubeMX. */
PwmTimer_Status PwmChannel_StartDma(PwmChannel *channel,
                                    const uint32_t *buffer,
                                    uint16_t length);
PwmTimer_Status PwmChannel_StopDma(PwmChannel *channel);
PwmTimer_Status PwmChannel_ServiceDma(PwmChannel *channel);
bool PwmChannel_IsDmaBusy(const PwmChannel *channel);

/*
 * Transfer channel_count consecutive compare values starting at CCR1 on each
 * timer update event. The buffer is interleaved by time slot:
 *   slot0_ccr1, slot0_ccr2, ..., slot1_ccr1, ...
 * This is suitable for synchronized multi-channel waveform generation and is
 * intentionally independent of any motor or protocol semantics.
 */
PwmTimer_Status PwmTimer_StartCompareBurstDma(
    PwmTimer *timer,
    const uint32_t *interleaved_buffer,
    uint16_t slot_count,
    uint8_t channel_count);
PwmTimer_Status PwmTimer_StopDma(PwmTimer *timer);
PwmTimer_Status PwmTimer_ServiceDma(PwmTimer *timer);
bool PwmTimer_IsDmaBusy(const PwmTimer *timer);

#ifdef __cplusplus
}
#endif

#endif /* PWM_TIMER_H */
