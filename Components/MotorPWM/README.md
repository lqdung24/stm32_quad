# Motor PWM

Four-channel standard ESC PWM output.

## Hardware used by the application

- Motor 1: `TIM3_CH1`, `PA6`
- Motor 2: `TIM3_CH2`, `PA7`
- Motor 3: `TIM3_CH3`, `PB0`
- Motor 4: `TIM3_CH4`, `PB1`

TIM3 runs from a 60 MHz timer clock. The prescaler is 59, giving a 1 MHz
counter, and ARR is 19999, giving standard 50 Hz ESC PWM.

## Pulse convention

- 1000 us: disarmed/minimum throttle
- 2000 us: maximum throttle

`MotorPwm_Init()` starts all four channels at 1000 us. Writes above minimum are
rejected until `MotorPwm_Arm()` is called. `MotorPwm_Disarm()` immediately
returns all channels to 1000 us.

The drone control path limits collective throttle to 500/1000, corresponding
to a maximum 1500 us pulse, and writes the same pulse to all four channels.

Always remove propellers during PWM and motor-order tests.
