#include "../Inc/pwm_timer.h"

#include <stddef.h>

static bool PwmTimer_IsApb2Timer(const TIM_TypeDef *instance);
static bool PwmTimer_IsValidChannel(uint32_t channel);
static uint32_t PwmTimer_ChannelDmaId(uint32_t channel);
static uint32_t PwmTimer_BurstLength(uint8_t channel_count);
static PwmTimer_Status PwmTimer_FromHalStatus(HAL_StatusTypeDef status);

PwmTimer_Status PwmTimer_GetInputClockHz(const TIM_HandleTypeDef *htim,
                                         uint32_t *clock_hz)
{
  RCC_ClkInitTypeDef clocks = {0};
  uint32_t flash_latency;
  uint32_t pclk_hz;
  uint32_t hclk_hz;
  bool apb_div1;
  bool apb_div2;
  bool apb_div4;
  bool timpre_enabled;

  if ((htim == NULL) || (htim->Instance == NULL) || (clock_hz == NULL))
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }

  HAL_RCC_GetClockConfig(&clocks, &flash_latency);
  hclk_hz = HAL_RCC_GetHCLKFreq();
  if (PwmTimer_IsApb2Timer(htim->Instance))
  {
    pclk_hz = HAL_RCC_GetPCLK2Freq();
    apb_div1 = clocks.APB2CLKDivider == RCC_APB2_DIV1;
    apb_div2 = clocks.APB2CLKDivider == RCC_APB2_DIV2;
    apb_div4 = clocks.APB2CLKDivider == RCC_APB2_DIV4;
  }
  else
  {
    pclk_hz = HAL_RCC_GetPCLK1Freq();
    apb_div1 = clocks.APB1CLKDivider == RCC_APB1_DIV1;
    apb_div2 = clocks.APB1CLKDivider == RCC_APB1_DIV2;
    apb_div4 = clocks.APB1CLKDivider == RCC_APB1_DIV4;
  }

  if ((hclk_hz == 0U) || (pclk_hz == 0U))
  {
    return PWM_TIMER_HAL_ERROR;
  }

  timpre_enabled = (RCC->CFGR & RCC_CFGR_TIMPRE) != 0U;
  if ((!timpre_enabled &&
       (apb_div1 || apb_div2)) ||
      (timpre_enabled &&
       (apb_div1 || apb_div2 || apb_div4)))
  {
    *clock_hz = hclk_hz;
  }
  else
  {
    *clock_hz = pclk_hz * (timpre_enabled ? 4U : 2U);
  }
  return PWM_TIMER_OK;
}

PwmTimer_Status PwmTimer_Init(PwmTimer *timer,
                              const PwmTimer_Config *config)
{
  uint32_t prescaler_divisor;

  if ((timer == NULL) || (config == NULL) || (config->htim == NULL) ||
      (config->htim->Instance == NULL) || (config->input_clock_hz == 0U) ||
      (config->counter_clock_hz == 0U) || (config->period_ticks < 2U) ||
      timer->initialized ||
      ((config->input_clock_hz % config->counter_clock_hz) != 0U))
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }

  prescaler_divisor = config->input_clock_hz / config->counter_clock_hz;
  if ((prescaler_divisor == 0U) || (prescaler_divisor > 65536U) ||
      !IS_TIM_PERIOD(config->htim, config->period_ticks - 1U))
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }

  timer->config = *config;
  timer->dma_busy = false;
  timer->initialized = false;

  config->htim->Init.Prescaler = prescaler_divisor - 1U;
  config->htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  config->htim->Init.Period = config->period_ticks - 1U;
  config->htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  config->htim->Init.AutoReloadPreload = config->auto_reload_preload ?
      TIM_AUTORELOAD_PRELOAD_ENABLE : TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(config->htim) != HAL_OK)
  {
    return PWM_TIMER_HAL_ERROR;
  }

  __HAL_TIM_SET_COUNTER(config->htim, 0U);
  timer->initialized = true;
  return PWM_TIMER_OK;
}

PwmTimer_Status PwmChannel_Init(PwmChannel *channel,
                                const PwmChannel_Config *config)
{
  TIM_OC_InitTypeDef pwm = {0};

  if ((channel == NULL) || (config == NULL) || (config->timer == NULL) ||
      !config->timer->initialized || channel->initialized ||
      !PwmTimer_IsValidChannel(config->channel) ||
      (config->initial_compare_ticks >= config->timer->config.period_ticks) ||
      ((config->polarity != TIM_OCPOLARITY_HIGH) &&
       (config->polarity != TIM_OCPOLARITY_LOW)))
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }

  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = config->initial_compare_ticks;
  pwm.OCPolarity = config->polarity;
  pwm.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;
  pwm.OCIdleState = TIM_OCIDLESTATE_RESET;
  pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(config->timer->config.htim,
                                &pwm,
                                config->channel) != HAL_OK)
  {
    return PWM_TIMER_HAL_ERROR;
  }

  channel->config = *config;
  channel->dma_busy = false;
  channel->started = false;
  channel->initialized = true;
  return PWM_TIMER_OK;
}

PwmTimer_Status PwmChannel_Start(PwmChannel *channel)
{
  PwmTimer_Status status;

  if ((channel == NULL) || !channel->initialized)
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }
  if (channel->started)
  {
    return PWM_TIMER_OK;
  }

  status = PwmTimer_FromHalStatus(
      HAL_TIM_PWM_Start(channel->config.timer->config.htim,
                        channel->config.channel));
  if (status == PWM_TIMER_OK)
  {
    channel->started = true;
  }
  return status;
}

PwmTimer_Status PwmChannel_Stop(PwmChannel *channel)
{
  PwmTimer_Status status;

  if ((channel == NULL) || !channel->initialized)
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }
  if (!channel->started)
  {
    return PWM_TIMER_OK;
  }
  if (channel->dma_busy)
  {
    status = PwmChannel_StopDma(channel);
    if (status != PWM_TIMER_OK)
    {
      return status;
    }
  }

  status = PwmTimer_FromHalStatus(
      HAL_TIM_PWM_Stop(channel->config.timer->config.htim,
                       channel->config.channel));
  if (status == PWM_TIMER_OK)
  {
    channel->started = false;
  }
  return status;
}

PwmTimer_Status PwmChannel_SetCompare(PwmChannel *channel,
                                      uint32_t compare_ticks)
{
  if ((channel == NULL) || !channel->initialized ||
      (compare_ticks >= channel->config.timer->config.period_ticks))
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }

  __HAL_TIM_SET_COMPARE(channel->config.timer->config.htim,
                        channel->config.channel,
                        compare_ticks);
  return PWM_TIMER_OK;
}

PwmTimer_Status PwmChannel_StartDma(PwmChannel *channel,
                                    const uint32_t *buffer,
                                    uint16_t length)
{
  PwmTimer_Status status;
  uint16_t index;

  if ((channel == NULL) || !channel->initialized || (buffer == NULL) ||
      (length == 0U))
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }
  for (index = 0U; index < length; ++index)
  {
    if (buffer[index] >= channel->config.timer->config.period_ticks)
    {
      return PWM_TIMER_INVALID_ARGUMENT;
    }
  }
  status = PwmChannel_ServiceDma(channel);
  if (status == PWM_TIMER_BUSY)
  {
    return PWM_TIMER_BUSY;
  }
  if (status != PWM_TIMER_OK)
  {
    return status;
  }

  status = PwmTimer_FromHalStatus(
      HAL_TIM_PWM_Start_DMA(channel->config.timer->config.htim,
                            channel->config.channel,
                            buffer,
                            length));
  if (status == PWM_TIMER_OK)
  {
    channel->dma_busy = true;
    channel->started = true;
  }
  return status;
}

PwmTimer_Status PwmChannel_StopDma(PwmChannel *channel)
{
  PwmTimer_Status status;

  if ((channel == NULL) || !channel->initialized)
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }
  if (!channel->dma_busy)
  {
    return PWM_TIMER_OK;
  }

  status = PwmTimer_FromHalStatus(
      HAL_TIM_PWM_Stop_DMA(channel->config.timer->config.htim,
                           channel->config.channel));
  channel->dma_busy = false;
  channel->started = false;
  return status;
}

PwmTimer_Status PwmChannel_ServiceDma(PwmChannel *channel)
{
  DMA_HandleTypeDef *hdma;
  uint32_t dma_id;

  if ((channel == NULL) || !channel->initialized)
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }
  if (!channel->dma_busy)
  {
    return PWM_TIMER_OK;
  }

  dma_id = PwmTimer_ChannelDmaId(channel->config.channel);
  hdma = channel->config.timer->config.htim->hdma[dma_id];
  if (hdma == NULL)
  {
    channel->dma_busy = false;
    return PWM_TIMER_HAL_ERROR;
  }
  if (HAL_DMA_GetState(hdma) == HAL_DMA_STATE_READY)
  {
    return PwmChannel_StopDma(channel);
  }
  if (HAL_DMA_GetState(hdma) == HAL_DMA_STATE_ERROR)
  {
    (void)PwmChannel_StopDma(channel);
    return PWM_TIMER_HAL_ERROR;
  }
  return PWM_TIMER_BUSY;
}

bool PwmChannel_IsDmaBusy(const PwmChannel *channel)
{
  return (channel != NULL) && channel->initialized && channel->dma_busy;
}

PwmTimer_Status PwmTimer_StartCompareBurstDma(
    PwmTimer *timer,
    const uint32_t *interleaved_buffer,
    uint16_t slot_count,
    uint8_t channel_count)
{
  PwmTimer_Status status;
  uint32_t transfer_count;
  uint32_t index;

  if ((timer == NULL) || !timer->initialized ||
      (interleaved_buffer == NULL) || (slot_count == 0U) ||
      (channel_count == 0U) ||
      (channel_count > PWM_TIMER_MAX_BURST_CHANNELS) ||
      (timer->config.htim->hdma[TIM_DMA_ID_UPDATE] == NULL))
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }

  status = PwmTimer_ServiceDma(timer);
  if (status == PWM_TIMER_BUSY)
  {
    return PWM_TIMER_BUSY;
  }
  if (status != PWM_TIMER_OK)
  {
    return status;
  }

  transfer_count = (uint32_t)slot_count * (uint32_t)channel_count;
  if (transfer_count > UINT16_MAX)
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }
  for (index = 0U; index < transfer_count; ++index)
  {
    if (interleaved_buffer[index] >= timer->config.period_ticks)
    {
      return PWM_TIMER_INVALID_ARGUMENT;
    }
  }

  status = PwmTimer_FromHalStatus(HAL_TIM_DMABurst_MultiWriteStart(
      timer->config.htim,
      TIM_DMABASE_CCR1,
      TIM_DMA_UPDATE,
      interleaved_buffer,
      PwmTimer_BurstLength(channel_count),
      transfer_count));
  if (status == PWM_TIMER_OK)
  {
    timer->dma_busy = true;
  }
  return status;
}

PwmTimer_Status PwmTimer_StopDma(PwmTimer *timer)
{
  PwmTimer_Status status;
  DMA_HandleTypeDef *hdma;

  if ((timer == NULL) || !timer->initialized)
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }
  if (!timer->dma_busy)
  {
    return PWM_TIMER_OK;
  }

  hdma = timer->config.htim->hdma[TIM_DMA_ID_UPDATE];
  if (hdma == NULL)
  {
    timer->dma_busy = false;
    return PWM_TIMER_HAL_ERROR;
  }
  /*
   * HAL_TIM_DMABurst_WriteStop() uses HAL_DMA_Abort_IT(), whose stream can
   * remain in ABORT state after this function returns. Abort synchronously
   * when a transfer is active so a safety stop can immediately submit a
   * DShot-zero frame without racing the old DMA stream.
   */
  if (HAL_DMA_GetState(hdma) == HAL_DMA_STATE_BUSY)
  {
    if (HAL_DMA_Abort(hdma) != HAL_OK)
    {
      timer->dma_busy = false;
      return PWM_TIMER_HAL_ERROR;
    }
  }
  status = PwmTimer_FromHalStatus(
      HAL_TIM_DMABurst_WriteStop(timer->config.htim, TIM_DMA_UPDATE));
  timer->dma_busy = false;
  return status;
}

PwmTimer_Status PwmTimer_ServiceDma(PwmTimer *timer)
{
  DMA_HandleTypeDef *hdma;
  HAL_DMA_StateTypeDef dma_state;

  if ((timer == NULL) || !timer->initialized)
  {
    return PWM_TIMER_INVALID_ARGUMENT;
  }
  if (!timer->dma_busy)
  {
    return PWM_TIMER_OK;
  }

  hdma = timer->config.htim->hdma[TIM_DMA_ID_UPDATE];
  if (hdma == NULL)
  {
    timer->dma_busy = false;
    return PWM_TIMER_HAL_ERROR;
  }

  dma_state = HAL_DMA_GetState(hdma);
  if (dma_state == HAL_DMA_STATE_READY)
  {
    return PwmTimer_StopDma(timer);
  }
  if (dma_state == HAL_DMA_STATE_ERROR)
  {
    (void)PwmTimer_StopDma(timer);
    return PWM_TIMER_HAL_ERROR;
  }
  return PWM_TIMER_BUSY;
}

bool PwmTimer_IsDmaBusy(const PwmTimer *timer)
{
  return (timer != NULL) && timer->initialized && timer->dma_busy;
}

static bool PwmTimer_IsApb2Timer(const TIM_TypeDef *instance)
{
  return (instance == TIM1) || (instance == TIM8) ||
         (instance == TIM15) || (instance == TIM16) ||
         (instance == TIM17);
}

static bool PwmTimer_IsValidChannel(uint32_t channel)
{
  return (channel == TIM_CHANNEL_1) || (channel == TIM_CHANNEL_2) ||
         (channel == TIM_CHANNEL_3) || (channel == TIM_CHANNEL_4);
}

static uint32_t PwmTimer_ChannelDmaId(uint32_t channel)
{
  if (channel == TIM_CHANNEL_1)
  {
    return TIM_DMA_ID_CC1;
  }
  if (channel == TIM_CHANNEL_2)
  {
    return TIM_DMA_ID_CC2;
  }
  if (channel == TIM_CHANNEL_3)
  {
    return TIM_DMA_ID_CC3;
  }
  return TIM_DMA_ID_CC4;
}

static uint32_t PwmTimer_BurstLength(uint8_t channel_count)
{
  return ((uint32_t)channel_count - 1U) << 8U;
}

static PwmTimer_Status PwmTimer_FromHalStatus(HAL_StatusTypeDef status)
{
  if (status == HAL_OK)
  {
    return PWM_TIMER_OK;
  }
  if (status == HAL_BUSY)
  {
    return PWM_TIMER_BUSY;
  }
  return PWM_TIMER_HAL_ERROR;
}
