# MotorDshot - function flow

Shared driver for DShot300 and DShot600. `MotorDshot300_Init()` is the fixed-rate
adapter; both rates use the same `MotorOutput` operations below.

- `MotorDshot_Init()` validates one shared timer, ordered CH1..CH4, TIM3_UP DMA,
  exact bitrate division, and the DShot value range; it derives duty ticks and
  binds the `MotorOutput` vtable.
- `start()` clears requested/applied values and submits the first value-zero frame. `DroneControl` continues zero frames for a timed pre-arm interval before it reports ARMED or accepts positive throttle.
- `stop()` synchronously aborts DMA, forces CCR1..CCR4 low, closes the logical
  gate, and submits an explicit value-zero frame.
- `set_throttle/set_all_throttle()` stage normalized commands only.
- `update()` services prior DMA completion, returns `MOTOR_BUSY` without
  overwriting an active buffer, maps throttle to 0 or 48..2047, encodes four
  packets, cleans D-cache when enabled, and starts one compare burst.
- `MotorDshot_MakePacket()` combines 11-bit value, telemetry bit and checksum.
- `MotorDshot_EncodePackets()` writes MSB-first duty values interleaved by time
  slot and appends two all-zero slots.

DMA completion is observed through `HAL_DMA_GetState()` in
`PwmTimer_ServiceDma()` on the next control update. The generated DMA IRQ calls
`HAL_DMA_IRQHandler()`, which moves the HAL handle to READY; service then stops
the request and resets HAL's separate DMA-burst state.
