# MotorPWM - function flow

- `MotorPwm_Init()` validates four initialized channels on one shared timer,
  writes stop compare values, and binds the `MotorOutput` vtable.
- `start()` writes all stop compares before opening the logical write gate.
- `stop()` writes all stop compares and closes the gate.
- `set_throttle()` stages one finite normalized command.
- `set_all_throttle()` validates and stages all four commands atomically.
- `update()` maps `0` to stop; positive commands to min..max with each
  calibrated idle floor; it then updates all CCR preloads through PwmChannel.
- Query functions return applied normalized throttle and actual compare count.

Invariant: `MotorPwm` never accesses a TIM register or HAL timer function.
