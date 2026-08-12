# Drone rate control

The controller implements three independent body-rate PID controllers:

- roll and pitch command range: `-200 .. +200 deg/s`
- yaw command range: `-150 .. +150 deg/s`
- input measurements: calibrated BODY FRD gyroscope rates in `rad/s`
- output: normalized mixer correction, not PWM microseconds

The PID uses derivative-on-measurement, a first-order derivative low-pass
filter, conditional integration, integral limiting and output limiting.
Controller state is reset whenever the vehicle is disarmed, enters failsafe,
has zero collective throttle, or receives an invalid sample interval.

The gains in `drone_control.c` are conservative initial bench values. They are
not airframe-independent and must be tuned on the actual vehicle.

## Quad-X mixer

The verified motor allocation and rotation direction, viewed from above the
drone, is:

| Motor | Position | Output | Rotation | First rotation | Configured idle | Mixer signs |
|---|---|---|---|---:|---:|---|
| M1 | front-left | `TIM3_CH1/PA6` | CW | `1200 us` | `1220 us` | `+roll +pitch -yaw` |
| M2 | rear-left | `TIM3_CH2/PA7` | CCW | `1205 us` | `1225 us` | `+roll -pitch +yaw` |
| M3 | front-right | `TIM3_CH3/PB0` | CCW | `1190 us` | `1210 us` | `-roll +pitch +yaw` |
| M4 | rear-right | `TIM3_CH4/PB1` | CW | `1205 us` | `1225 us` | `-roll -pitch -yaw` |

Each configured idle value includes a `20 us` margin above the measured
first-rotation threshold. CW/CCW always refers to the view from above.

Mixer commands use a logical `0..1000` scale. Common collective shifting
preserves roll/pitch/yaw authority at the actuator limits. Corrections are
scaled together only if their span exceeds the complete actuator range.

Disarm and armed-zero-throttle remain `1000 us`. Logical motor commands retain
the nominal ESC pulse scale: command 200 means `1200 us`, command 500 means
`1500 us`, and command 1000 means `2000 us`. While collective is positive,
each output is floored at its calibrated idle value in the table. The pilot
collective is capped at 500, but mixer corrections may drive an individual
motor above `1500 us`, up to the `2000 us` actuator limit. Raw threshold-test
mode is disabled, so body-rate PID corrections are applied through the Quad-X
mixer.
