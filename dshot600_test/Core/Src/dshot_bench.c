#include "dshot_bench.h"
#include "dshot_encode.h"
#include <stdbool.h>

volatile DshotBenchStatus dshot_bench;
static bool initialized;
static uint32_t boot_ms, last_frame_ms, last_beacon_ms;

static void pins_low(void)
{
  TIM3->DIER = 0;
  TIM3->CR1 = 0;
  GPIOA->BSRR = (GPIO_PIN_6 | GPIO_PIN_7) << 16U;
  GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1) << 16U;
  MODIFY_REG(GPIOA->MODER, (15U << 12), (5U << 12));
  MODIFY_REG(GPIOB->MODER, 15U, 5U);
}
static void fail(uint32_t reason)
{
  pins_low();
  dshot_bench.fault = reason;
}
static inline __attribute__((always_inline)) void write_bank(uint32_t duty)
{
  TIM3->CCR1 = duty;
  TIM3->CCR2 = duty;
  TIM3->CCR3 = duty;
  TIM3->CCR4 = duty;
}

/* PWM edges are hardware generated. CPU loads the next preload bank before
 * each overflow. O2 is required even in an otherwise O0 debug build. */
static bool __attribute__((optimize("O2"))) send_frame(const uint32_t *words)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  TIM3->CR1 = 0;
  write_bank(0);
  TIM3->EGR = TIM_EGR_UG; /* leading LOW slot; reset counter/prescaler */
  TIM3->SR = 0;
  uint32_t begin = DWT->CYCCNT;
  /* First bit starts at a hardware update, with no software edge skew. */
  write_bank(words[0]);
  TIM3->CR1 = TIM_CR1_CEN;
  for (unsigned slot = 0; slot < 18; ++slot) {
    while ((TIM3->SR & TIM_SR_UIF) == 0U) {
      if ((uint32_t)(DWT->CYCCNT - begin) > SystemCoreClock / 10000U) {
        fail(9); /* bounded 100 us wait if timer stalls */
        __set_PRIMASK(mask);
        return false;
      }
    }
    TIM3->SR = 0;
    if (slot + 1U < 18U) {
      write_bank(words[(slot + 1U) * 4U]);
      /* Must finish all four writes within the current period. */
      if ((TIM3->SR & TIM_SR_UIF) != 0U) {
        fail(11);
        __set_PRIMASK(mask);
        return false;
      }
    }
  }
  /* Both reset slots now contain zero; leave outputs LOW between frames. */
  TIM3->CR1 = 0;
  uint32_t cycles = DWT->CYCCNT - begin;
  dshot_bench.frame_cycles = cycles;
  if (cycles > dshot_bench.max_frame_cycles) dshot_bench.max_frame_cycles = cycles;
  __set_PRIMASK(mask);
  return true;
}

void DshotBench_Init(TIM_HandleTypeDef *timer)
{
  if (initialized || dshot_bench.fault) return;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  if (timer == NULL || timer->Instance != TIM3) { fail(1); return; }
  /* Generated DMA setup may remain, but no stream/request is used. */
  TIM3->DIER = 0;
  NVIC_DisableIRQ(DMA1_Stream0_IRQn);
  RCC_ClkInitTypeDef clock = {0};
  uint32_t latency;
  HAL_RCC_GetClockConfig(&clock, &latency);
  uint32_t hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_TIMPRE) == 0U) {
    if (clock.APB1CLKDivider != RCC_APB1_DIV1) hz *= 2U;
  } else if (clock.APB1CLKDivider == RCC_APB1_DIV1 ||
             clock.APB1CLKDivider == RCC_APB1_DIV2 ||
             clock.APB1CLKDivider == RCC_APB1_DIV4) {
    hz = HAL_RCC_GetHCLKFreq();
  } else hz *= 4U;
  uint32_t ticks = hz / 600000U;
  if (hz % 600000U || ticks < 8U || ticks > 65536U) { fail(4); return; }
  dshot_bench.timer_hz = hz;
  dshot_bench.bit_ticks = ticks;
  dshot_bench.zero_ticks = (ticks * 3U + 4U) / 8U;
  dshot_bench.one_ticks = (ticks * 3U + 2U) / 4U;
  pins_low();
  TIM3->CCER = 0;
  TIM3->SMCR = 0;
  TIM3->PSC = 0;
  TIM3->ARR = ticks - 1U;
  TIM3->CNT = 0;
  /* OCxM=110 (PWM1), OCxPE=1, CCxS=00 (output). */
  TIM3->CCMR1 = (6U << 4) | TIM_CCMR1_OC1PE |
                 (6U << 12) | TIM_CCMR1_OC2PE;
  TIM3->CCMR2 = (6U << 4) | TIM_CCMR2_OC3PE |
                 (6U << 12) | TIM_CCMR2_OC4PE;
  write_bank(0);
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = 0;
  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  MODIFY_REG(GPIOA->AFR[0], 0xffU << 24, 0x22U << 24);
  MODIFY_REG(GPIOB->AFR[0], 0xffU, 0x22U);
  CLEAR_BIT(GPIOA->OTYPER, GPIO_PIN_6 | GPIO_PIN_7);
  CLEAR_BIT(GPIOB->OTYPER, GPIO_PIN_0 | GPIO_PIN_1);
  CLEAR_BIT(GPIOA->PUPDR, 15U << 12);
  CLEAR_BIT(GPIOB->PUPDR, 15U);
  SET_BIT(GPIOA->OSPEEDR, 15U << 12);
  SET_BIT(GPIOB->OSPEEDR, 15U);
  MODIFY_REG(GPIOA->MODER, 15U << 12, 10U << 12);
  MODIFY_REG(GPIOB->MODER, 15U, 10U);
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR)
  DWT->LAR = 0xC5ACCE55U;
#endif
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t before = DWT->CYCCNT;
  for (volatile unsigned i = 0; i < 32; ++i) { __NOP(); }
  if (DWT->CYCCNT == before) { fail(6); return; }
  boot_ms = HAL_GetTick();
  last_frame_ms = boot_ms - 1U;
  initialized = true;
  (void)DshotBench_RequestBeacon(1);
}

int DshotBench_RequestBeacon(uint8_t command)
{
  if (!initialized || dshot_bench.fault || dshot_bench.stop_requested ||
      command < 1U || command > 5U) return 0;
  boot_ms = HAL_GetTick();
  dshot_bench.beacon_command = command;
  dshot_bench.beacon_sent = 0;
  dshot_bench.beacon_packet = Dshot_PacketWithTelemetry(command, 1);
  return 1;
}

void DshotBench_Step(void)
{
  if (!initialized || dshot_bench.fault) return;
  if (dshot_bench.stop_requested) { fail(7); return; }
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - last_frame_ms) < 1U) return;
  last_frame_ms = now;
  bool beacon = (uint32_t)(now - boot_ms) >= 3000U &&
    dshot_bench.beacon_sent < 10U &&
    (dshot_bench.beacon_sent == 0U || (uint32_t)(now - last_beacon_ms) >= 300U);
  uint16_t value = beacon ? (uint16_t)dshot_bench.beacon_command : 0U;
  uint32_t words[DSHOT_WORDS];
  Dshot_EncodeWithTelemetry(words, value, beacon, dshot_bench.zero_ticks, dshot_bench.one_ticks);
  if (send_frame(words)) {
    dshot_bench.last_value = value;
    ++dshot_bench.frames_completed;
    if (beacon) {
      ++dshot_bench.beacon_sent;
      last_beacon_ms = now;
    }
  }
}
