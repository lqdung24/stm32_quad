#ifndef DSHOT_BENCH_H
#define DSHOT_BENCH_H
#include "main.h"
#include <stdint.h>
/* Latched faults; reset MCU to restart. stop_requested is debugger writable. */
typedef struct {
  uint32_t timer_hz, bit_ticks, zero_ticks, one_ticks;
  uint32_t frames_completed, dma_error, fault, last_value; /* dma_error unused, always 0 */
  uint32_t stop_requested;
  uint32_t frame_cycles, max_frame_cycles;
  uint32_t beacon_command, beacon_sent, beacon_packet;
} DshotBenchStatus;
extern volatile DshotBenchStatus dshot_bench;
void DshotBench_Init(TIM_HandleTypeDef *timer);
void DshotBench_Step(void);
/* Main-loop only. Accept raw beacon 1..5; restart 3 s zero hold then 10 beeps.
 * Returns 0 on invalid command/fault/uninitialized; never sends throttle. */
int DshotBench_RequestBeacon(uint8_t command);
#endif
