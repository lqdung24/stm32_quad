# Motor PWM

Standard four-channel ESC PWM implementation of `MotorOutput`.

`MotorPwm` owns throttle policy only: normalized mapping, per-motor idle floor,
stop compare, and cached applied output. Timer/channel configuration and every
CCR write are delegated to `PwmTimer`/`PwmChannel`.

Application configuration:

| Motor | Position | Channel/GPIO | Rotation | Idle compare |
|---|---|---|---|---:|
| M1 | front-left | TIM3_CH1/PA6 | CW | 1220 |
| M2 | rear-left | TIM3_CH2/PA7 | CCW | 1225 |
| M3 | front-right | TIM3_CH3/PB0 | CCW | 1210 |
| M4 | rear-right | TIM3_CH4/PB1 | CW | 1225 |

TIM3 uses a 60 MHz input, 1 MHz counter, and 20000-count period. Consequently
compare counts equal microseconds and the output remains 50 Hz.

Initialization outline:

```c
PwmTimer timer;
PwmChannel channel[4];
MotorPwm pwm_driver;
MotorOutput motors;

/* Initialize timer once, then initialize/start CH1..CH4. */
MotorPwm_Config config = {
  .channel = {&channel[0], &channel[1], &channel[2], &channel[3]},
  .stop_compare = 1000U,
  .minimum_compare = 1000U,
  .maximum_compare = 2000U,
  .idle_compare = {1220U, 1225U, 1210U, 1225U},
};
MotorPwm_Init(&pwm_driver, &motors, &config);
```
