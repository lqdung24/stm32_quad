# PwmTimer - function flow

- `PwmTimer_GetInputClockHz()` selects APB1/APB2 from the timer instance and
  applies STM32H7 `TIMPRE` and APB-prescaler rules.
- `PwmTimer_Init()` validates an exact clock division, writes PSC/ARR through
  the HAL handle, initializes PWM once, and resets CNT.
- `PwmChannel_Init()` configures one PWM1 channel and its initial compare.
- `PwmChannel_Start/Stop()` control one channel through HAL.
- `PwmChannel_SetCompare()` bounds the value then writes CCR.
- `PwmChannel_StartDma/ServiceDma/StopDma()` own one channel DMA lifecycle.
- `PwmTimer_StartCompareBurstDma()` starts an update-triggered DMA burst from
  CCR1 across one to four consecutive channels.
- `PwmTimer_ServiceDma()` converts HAL DMA completion/error into component
  state and resets HAL's DMA-burst state before the next transfer.
- `PwmTimer_StopDma()` synchronously aborts an active stream before clearing
  the TIM DMA request, allowing a safety path to start a replacement frame
  immediately.
