# MotorDshot300 - function flow

- `MotorDshot300_Init()` rejects NULL configuration or a bitrate other than
  `MOTOR_DSHOT300_BITRATE_HZ`, then delegates to `MotorDshot_Init()`.
- `MotorDshot300` and `MotorDshot300_Config` alias the shared driver types.
- All `MotorOutput` start/stop/stage/update calls use the shared DShot vtable.
- App derives the timer period from the selected 300000 bit/s rate; at 60 MHz
  this is 200 ticks per bit. The shared encoder derives high times of 75/150 ticks.
- The existing App bench sequencer owns output every 1 ms: 15 s of zero frames,
  then fixed 30% throttle. Its DMA BUSY handling and latched-error Stop are shared
  with DShot600. No ARM, radio watchdog or PID in bench mode.

See `../MotorDshot/function-flow.md` for the shared encoder and DMA lifecycle.
