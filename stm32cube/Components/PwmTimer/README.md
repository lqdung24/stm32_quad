# PwmTimer

Hardware abstraction for STM32 timer PWM channels and DMA. This component has
no motor, throttle, ESC, or DShot policy.

One `PwmTimer` owns the shared PSC/ARR configuration. Multiple `PwmChannel`
objects reference that timer and therefore cannot silently select different
counter frequencies or periods.

`PwmTimer_GetInputClockHz()` implements the STM32H7 APB/TIMPRE rules instead of
assuming `SystemCoreClock`. `PwmTimer_Init()` requires an exact integer divider
from timer input clock to counter clock and refuses a second initialization of
the same object.

Two DMA forms are available:

- `PwmChannel_StartDma()` uses a channel capture/compare DMA request.
- `PwmTimer_StartCompareBurstDma()` uses one update request and writes
  consecutive CCR registers from a time-slot-interleaved buffer.

The burst form is generic multi-channel waveform support. DShot uses it to
update CCR1..CCR4 on the same timer update edge.

CubeMX remains responsible for selecting a DMA stream/request, word alignment,
normal mode, priority, and NVIC handler. DMA buffers must be in DMA-accessible
RAM. A caller that enables D-cache must clean a transmit buffer before starting
DMA; `MotorDshot` performs that clean for its owned buffer.
