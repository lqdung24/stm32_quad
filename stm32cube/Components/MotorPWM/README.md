# Motor PWM

Four-channel standard ESC PWM output.

## Hardware used by the application

Rotation direction is stated while viewing the drone from above.

| Motor | Position | Timer/GPIO | Rotation | First rotation | Configured idle |
|---|---|---|---|---:|---:|
| M1 | front-left | `TIM3_CH1/PA6` | CW | `1200 us` | `1220 us` |
| M2 | rear-left | `TIM3_CH2/PA7` | CCW | `1205 us` | `1225 us` |
| M3 | front-right | `TIM3_CH3/PB0` | CCW | `1190 us` | `1210 us` |
| M4 | rear-right | `TIM3_CH4/PB1` | CW | `1205 us` | `1225 us` |

The configured idle values are `20 us` above the no-prop first-rotation
measurements.

TIM3 runs from a 60 MHz timer clock. The prescaler is 59, giving a 1 MHz
counter, and ARR is 19999, giving standard 50 Hz ESC PWM.

## Pulse convention

- 1000 us: disarmed/minimum throttle
- 2000 us: maximum throttle

The application owns TIM/PWM initialization, channel configuration and channel
start. `MotorPwm_Attach()` only binds the running timer, writes the disarmed
compare values and provides the arm/disarm write gate.
