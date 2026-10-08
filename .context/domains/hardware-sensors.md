# Domain: hardware, sensors and scheduling

## Component pipeline

```text
ICM20948 SPI raw data
  -> Attitude frame mapping/calibration
  -> gyro body rad/s + gravity -> Mahony6 -> roll/pitch angle outer loop
  -> gyro body rad/s --------------------> rate PID -> mixer
  -> Mahony6 Euler + rate/PID/motor data -> telemetry
  -> calibrated magnetometer ----------> diagnostics only
```

`App` composes the HAL handles, creates the selected motor output in the stopped state and schedules the runtime pipeline.

## Frames and calibration

- Control and estimator body frame is FRD (+forward, +right, +down).
- Accel/gyro and magnetometer require different sensor-to-body sign mappings; do not consolidate them without verifying board orientation.
- Gyro bias is estimated during stationary startup calibration.
- Accelerometer uses stored per-axis bias/scale from six-position calibration.
- Magnetometer uses hard-iron offset plus a 3×3 soft-iron matrix.
- The accelerometer returns specific force; the attitude filter receives the negated gravity vector.

## Timing model

- Flight task: nominal 1 ms, high priority.
- Telemetry task: nominal 5 ms, below-normal priority.
- Housekeeping: nominal 100 ms in the CubeMX default task.
- Gyro/accel poll target: 1 ms.
- DShot ESC command output: nominal 2 ms (500 Hz), after the IMU/PID step;
  includes zero-throttle, MOTOR_TEST, pre-arm and optional bench frames.
  DISARM/e-stop/failsafe bypass the cadence and stop immediately. PWM stays 50 Hz.
- Magnetometer service: 10 ms and deliberately after the gyro/rate loop.
- Absolute delays resynchronize and sleep one tick after an overrun to avoid starving lower-priority tasks.

DWT cycle counters record IMU period/read time, PID period/execute time and the sensor/estimator/PID/staging pipeline time. The separate periodic DShot commit is outside that pipeline measurement; verify ESC frame timing on the pins. Statistics use a sequence-counter snapshot because writer and reader run in different task contexts.

## ICM20948/AK09916 interface

ICM20948 communicates over SPI1 with explicit bank selection. The embedded AK09916 is accessed through the ICM auxiliary-I2C master and exposed through external-sensor shadow registers. Initialization stops existing auxiliary transfers before resetting the ICM to avoid leaving the magnetometer bus stuck after an MCU-only reset.

The active estimator is `Components/Mahony` (6-axis), initialized from gravity
without a magnetometer. Mahony9 remains available as an unused component.
A fresh data-ready accel/gyro sample updates Mahony6 before the controller;
Euler degrees and calibrated gyro rad/s from the same sample go to
`DroneControl_UpdateFlightSample`. Yaw is gyro-integrated and may drift;
ANGLE uses roll/pitch feedback and body yaw-rate control, never heading hold.

Mahony6 rejects nonfinite inputs/state and sample `dt` outside 0.5..20 ms.
Initialization requires gravity norm within 0.8..1.2 g. During updates,
out-of-gate finite acceleration skips gravity correction but gyro propagation
continues. Invalid/stalled updates mark the current attitude invalid before any
later reinitialization. IMU reinitialization resets the DWT sample origin.
Before blocking runtime IMU reinitialization/calibration, App invalidates the
sensor sample through DroneControl so armed outputs stop first.
ANGLE requires a valid attitude sample younger than 20 ms to arm or run;
ACRO does not require an initialized attitude filter.
Magnetometer zero vectors and overflow remain invalid for diagnostics.

## Motor hardware contract

- TIM3 CH1..CH4 drive M1..M4. Normal operation defaults to DShot300 with `APP_DSHOT_TEST_ENABLE=0`; ARM/DISARM, watchdog, PID and mixer own output. PWM at 50 Hz and DShot600 remain explicit protocol selections. Optional `APP_DSHOT_TEST_ENABLE=1` enables the 30% bench test (15 s of zero frames first); see `control-safety.md`.
- PWM stop is 1000 µs with bounds 1000..2000 µs.
- DShot300/600 use the same channels and one TIM3_UP DMA burst. At the current 60 MHz timer clock they use 200/100 ticks per bit, respectively. App derives the period from the actual timer clock. TIM3_UP DMA1 Stream0 and its IRQ are already configured in the current generated source; verify them after CubeMX regeneration.
- Motor order/direction and calibrated idle floors are documented in `stm32cube/README.md`.
- STM32, ESP and ESC signal grounds must be common; UART is 3.3 V logic.

Never alter CubeMX-generated files casually. Prefer component/user-code regions and follow the STM32-specific agent workflow before editing under `stm32cube/`.
