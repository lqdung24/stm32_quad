#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>

typedef enum
{
  HAL_OK = 0,
  HAL_ERROR,
  HAL_BUSY
} HAL_StatusTypeDef;

typedef enum
{
  HAL_DMA_STATE_RESET = 0,
  HAL_DMA_STATE_READY,
  HAL_DMA_STATE_BUSY,
  HAL_DMA_STATE_ERROR
} HAL_DMA_StateTypeDef;

typedef struct
{
  HAL_DMA_StateTypeDef state;
} DMA_HandleTypeDef;

typedef struct
{
  uint32_t counter;
  uint32_t compare[4];
} TIM_TypeDef;

typedef struct
{
  uint32_t Prescaler;
  uint32_t CounterMode;
  uint32_t Period;
  uint32_t ClockDivision;
  uint32_t AutoReloadPreload;
} TIM_Base_InitTypeDef;

typedef struct
{
  TIM_TypeDef *Instance;
  TIM_Base_InitTypeDef Init;
  DMA_HandleTypeDef *hdma[7];
} TIM_HandleTypeDef;

typedef struct
{
  uint32_t OCMode;
  uint32_t Pulse;
  uint32_t OCPolarity;
  uint32_t OCNPolarity;
  uint32_t OCFastMode;
  uint32_t OCIdleState;
  uint32_t OCNIdleState;
} TIM_OC_InitTypeDef;

typedef struct
{
  uint32_t APB1CLKDivider;
  uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;

typedef struct
{
  uint32_t CFGR;
} RCC_TypeDef;

extern RCC_TypeDef fake_rcc;
extern TIM_TypeDef fake_tim1;
extern TIM_TypeDef fake_tim3;
extern TIM_TypeDef fake_tim8;
extern TIM_TypeDef fake_tim15;
extern TIM_TypeDef fake_tim16;
extern TIM_TypeDef fake_tim17;

#define RCC (&fake_rcc)
#define TIM1 (&fake_tim1)
#define TIM3 (&fake_tim3)
#define TIM8 (&fake_tim8)
#define TIM15 (&fake_tim15)
#define TIM16 (&fake_tim16)
#define TIM17 (&fake_tim17)

#define RCC_CFGR_TIMPRE 0x00008000U
#define RCC_APB1_DIV1 0U
#define RCC_APB1_DIV2 4U
#define RCC_APB1_DIV4 5U
#define RCC_APB1_DIV8 6U
#define RCC_APB1_DIV16 7U
#define RCC_APB2_DIV1 0x000U
#define RCC_APB2_DIV2 0x400U
#define RCC_APB2_DIV4 0x500U
#define RCC_APB2_DIV8 0x600U
#define RCC_APB2_DIV16 0x700U

#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 1U
#define TIM_OCMODE_PWM1 1U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCPOLARITY_LOW 1U
#define TIM_OCNPOLARITY_HIGH 0U
#define TIM_OCFAST_DISABLE 0U
#define TIM_OCIDLESTATE_RESET 0U
#define TIM_OCNIDLESTATE_RESET 0U

#define TIM_CHANNEL_1 0x00000000U
#define TIM_CHANNEL_2 0x00000004U
#define TIM_CHANNEL_3 0x00000008U
#define TIM_CHANNEL_4 0x0000000CU

#define TIM_DMA_ID_UPDATE 0U
#define TIM_DMA_ID_CC1 1U
#define TIM_DMA_ID_CC2 2U
#define TIM_DMA_ID_CC3 3U
#define TIM_DMA_ID_CC4 4U
#define TIM_DMABASE_CCR1 13U
#define TIM_DMA_UPDATE 1U

#define IS_TIM_PERIOD(handle, period) ((period) <= 0xFFFFU)
#define __HAL_TIM_SET_COUNTER(handle, value) \
  ((handle)->Instance->counter = (value))
#define __HAL_TIM_SET_COMPARE(handle, channel, value) \
  ((handle)->Instance->compare[(channel) / 4U] = (value))

void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *config,
                            uint32_t *flash_latency);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim,
                                            TIM_OC_InitTypeDef *config,
                                            uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim,
                                    uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim,
                                   uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim,
                                        uint32_t channel,
                                        const uint32_t *buffer,
                                        uint16_t length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim,
                                       uint32_t channel);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiWriteStart(
    TIM_HandleTypeDef *htim,
    uint32_t base,
    uint32_t request,
    const uint32_t *buffer,
    uint32_t burst_length,
    uint32_t data_length);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim,
                                             uint32_t request);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);

#endif /* STM32H7XX_HAL_H */
