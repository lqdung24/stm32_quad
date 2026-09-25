#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>

typedef struct
{
  uint32_t unused;
} TIM_TypeDef;

typedef struct
{
  uint32_t unused;
} DMA_HandleTypeDef;

typedef struct
{
  TIM_TypeDef *Instance;
  DMA_HandleTypeDef *hdma[7];
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0x00000000U
#define TIM_CHANNEL_2 0x00000004U
#define TIM_CHANNEL_3 0x00000008U
#define TIM_CHANNEL_4 0x0000000CU
#define TIM_OCPOLARITY_HIGH 0x00000000U
#define TIM_DMA_ID_UPDATE 0U

#endif /* STM32H7XX_HAL_H */
