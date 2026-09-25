#ifndef USB_LOG_MOCK_H
#define USB_LOG_MOCK_H
#include <stdint.h>
#include <stddef.h>
typedef struct {uint32_t timer_hz,bit_ticks,zero_ticks,one_ticks,frames_completed,dma_error,fault,last_value,stop_requested,frame_cycles,max_frame_cycles,beacon_command,beacon_sent,beacon_packet;} DshotBenchStatus;
extern volatile DshotBenchStatus dshot_bench;
typedef struct { uint32_t PSC,ARR,CR1,DIER; } TIM_TypeDef;
extern TIM_TypeDef fake_tim;
#define TIM3 (&fake_tim)
typedef struct {uint32_t OscillatorType,HSI48State;struct{uint32_t PLLState;}PLL;} RCC_OscInitTypeDef;
typedef struct {uint32_t Prescaler,Source,Polarity,ReloadValue,ErrorLimitValue,HSI48CalibrationValue;} RCC_CRSInitTypeDef;
#define RCC_OSCILLATORTYPE_HSI48 1
#define RCC_HSI48_ON 1
#define RCC_PLL_NONE 0
#define HAL_OK 0
#define RCC_CRS_SYNC_DIV1 0
#define RCC_CRS_SYNC_SOURCE_USB2 0
#define RCC_CRS_SYNC_POLARITY_RISING 0
#define RCC_CRS_ERRORLIMIT_DEFAULT 34
#define RCC_CRS_HSI48CALIBRATION_DEFAULT 32
#define __HAL_RCC_CRS_CLK_ENABLE() ((void)0)
#define __HAL_RCC_CRS_RELOADVALUE_CALCULATE(a,b) ((a)/(b)-1U)
int HAL_RCC_OscConfig(RCC_OscInitTypeDef *p);
void HAL_RCCEx_CRSConfig(RCC_CRSInitTypeDef *p);
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
typedef struct {uint32_t TxState;} USBD_CDC_HandleTypeDef;
typedef struct {void *pClassData;uint32_t dev_state;} USBD_HandleTypeDef;
#define USBD_STATE_CONFIGURED 3U
void MX_USB_DEVICE_Init(void);
uint8_t CDC_Transmit_FS(uint8_t *data,uint16_t length);
#endif
