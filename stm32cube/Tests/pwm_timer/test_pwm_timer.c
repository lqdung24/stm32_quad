#include "pwm_timer.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

RCC_TypeDef fake_rcc;
TIM_TypeDef fake_tim1;
TIM_TypeDef fake_tim3;
TIM_TypeDef fake_tim8;
TIM_TypeDef fake_tim15;
TIM_TypeDef fake_tim16;
TIM_TypeDef fake_tim17;

static RCC_ClkInitTypeDef fake_clocks;
static uint32_t fake_hclk_hz;
static uint32_t fake_pclk1_hz;
static uint32_t fake_pclk2_hz;
static unsigned int pwm_init_count;
static unsigned int burst_start_count;
static unsigned int burst_stop_count;
static unsigned int dma_abort_count;
static uint32_t last_burst_length;
static uint32_t last_data_length;

void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *config,
                            uint32_t *flash_latency)
{
  *config = fake_clocks;
  *flash_latency = 0U;
}

uint32_t HAL_RCC_GetHCLKFreq(void)
{
  return fake_hclk_hz;
}

uint32_t HAL_RCC_GetPCLK1Freq(void)
{
  return fake_pclk1_hz;
}

uint32_t HAL_RCC_GetPCLK2Freq(void)
{
  return fake_pclk2_hz;
}

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim)
{
  assert(htim != NULL);
  ++pwm_init_count;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim,
                                            TIM_OC_InitTypeDef *config,
                                            uint32_t channel)
{
  assert(htim != NULL);
  htim->Instance->compare[channel / 4U] = config->Pulse;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim,
                                    uint32_t channel)
{
  (void)htim;
  (void)channel;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim,
                                   uint32_t channel)
{
  (void)htim;
  (void)channel;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim,
                                        uint32_t channel,
                                        const uint32_t *buffer,
                                        uint16_t length)
{
  (void)channel;
  (void)buffer;
  (void)length;
  htim->hdma[TIM_DMA_ID_CC1]->state = HAL_DMA_STATE_BUSY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim,
                                       uint32_t channel)
{
  (void)channel;
  htim->hdma[TIM_DMA_ID_CC1]->state = HAL_DMA_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_DMABurst_MultiWriteStart(
    TIM_HandleTypeDef *htim,
    uint32_t base,
    uint32_t request,
    const uint32_t *buffer,
    uint32_t burst_length,
    uint32_t data_length)
{
  assert(base == TIM_DMABASE_CCR1);
  assert(request == TIM_DMA_UPDATE);
  assert(buffer != NULL);
  ++burst_start_count;
  last_burst_length = burst_length;
  last_data_length = data_length;
  htim->hdma[TIM_DMA_ID_UPDATE]->state = HAL_DMA_STATE_BUSY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim,
                                             uint32_t request)
{
  assert(request == TIM_DMA_UPDATE);
  ++burst_stop_count;
  htim->hdma[TIM_DMA_ID_UPDATE]->state = HAL_DMA_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
  ++dma_abort_count;
  hdma->state = HAL_DMA_STATE_READY;
  return HAL_OK;
}

HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma)
{
  return hdma->state;
}

static void test_clock_derivation(void)
{
  TIM_HandleTypeDef htim = {0};
  uint32_t clock_hz;

  htim.Instance = TIM3;
  fake_hclk_hz = 120000000U;
  fake_pclk1_hz = 30000000U;
  fake_pclk2_hz = 30000000U;
  fake_clocks.APB1CLKDivider = RCC_APB1_DIV4;
  fake_clocks.APB2CLKDivider = RCC_APB2_DIV4;
  fake_rcc.CFGR = 0U;
  assert(PwmTimer_GetInputClockHz(&htim, &clock_hz) == PWM_TIMER_OK);
  assert(clock_hz == 60000000U);

  fake_rcc.CFGR = RCC_CFGR_TIMPRE;
  assert(PwmTimer_GetInputClockHz(&htim, &clock_hz) == PWM_TIMER_OK);
  assert(clock_hz == 120000000U);

  htim.Instance = TIM1;
  fake_clocks.APB2CLKDivider = RCC_APB2_DIV8;
  fake_pclk2_hz = 15000000U;
  assert(PwmTimer_GetInputClockHz(&htim, &clock_hz) == PWM_TIMER_OK);
  assert(clock_hz == 60000000U);
}

static void test_timer_channel_and_burst(void)
{
  TIM_HandleTypeDef htim = {0};
  DMA_HandleTypeDef update_dma = {0};
  PwmTimer timer = {0};
  PwmChannel channel = {0};
  const PwmTimer_Config timer_config = {
    .htim = &htim,
    .input_clock_hz = 60000000U,
    .counter_clock_hz = 1000000U,
    .period_ticks = 20000U,
    .auto_reload_preload = false,
  };
  const uint32_t burst_buffer[8] = {0U};
  PwmChannel_Config channel_config;

  memset(&fake_tim3, 0, sizeof(fake_tim3));
  htim.Instance = TIM3;
  htim.hdma[TIM_DMA_ID_UPDATE] = &update_dma;
  pwm_init_count = 0U;
  burst_start_count = 0U;
  burst_stop_count = 0U;
  dma_abort_count = 0U;

  assert(PwmTimer_Init(&timer, &timer_config) == PWM_TIMER_OK);
  assert(pwm_init_count == 1U);
  assert(htim.Init.Prescaler == 59U);
  assert(htim.Init.Period == 19999U);
  assert(PwmTimer_Init(&timer, &timer_config) ==
         PWM_TIMER_INVALID_ARGUMENT);

  memset(&channel_config, 0, sizeof(channel_config));
  channel_config.timer = &timer;
  channel_config.channel = TIM_CHANNEL_2;
  channel_config.initial_compare_ticks = 1000U;
  channel_config.polarity = TIM_OCPOLARITY_HIGH;
  assert(PwmChannel_Init(&channel, &channel_config) == PWM_TIMER_OK);
  assert(PwmChannel_Start(&channel) == PWM_TIMER_OK);
  assert(PwmChannel_SetCompare(&channel, 1500U) == PWM_TIMER_OK);
  assert(fake_tim3.compare[1] == 1500U);
  assert(PwmChannel_SetCompare(&channel, 20000U) ==
         PWM_TIMER_INVALID_ARGUMENT);

  assert(PwmTimer_StartCompareBurstDma(&timer,
                                       burst_buffer,
                                       2U,
                                       4U) == PWM_TIMER_OK);
  assert(PwmTimer_IsDmaBusy(&timer));
  assert(burst_start_count == 1U);
  assert(last_burst_length == 0x300U);
  assert(last_data_length == 8U);
  assert(PwmTimer_StartCompareBurstDma(&timer,
                                       burst_buffer,
                                       2U,
                                       4U) == PWM_TIMER_BUSY);
  update_dma.state = HAL_DMA_STATE_READY;
  assert(PwmTimer_ServiceDma(&timer) == PWM_TIMER_OK);
  assert(!PwmTimer_IsDmaBusy(&timer));
  assert(burst_stop_count == 1U);
  assert(dma_abort_count == 0U);

  assert(PwmTimer_StartCompareBurstDma(&timer,
                                       burst_buffer,
                                       2U,
                                       4U) == PWM_TIMER_OK);
  assert(PwmTimer_StopDma(&timer) == PWM_TIMER_OK);
  assert(dma_abort_count == 1U);
  assert(!PwmTimer_IsDmaBusy(&timer));
}

int main(void)
{
  test_clock_derivation();
  test_timer_channel_and_burst();
  puts("pwm timer tests: PASS");
  return 0;
}
