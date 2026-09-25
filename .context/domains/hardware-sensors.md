# Domain: hardware, sensors and scheduling

## Component pipeline

```text
ICM20948 SPI raw data
  -> Attitude frame mapping/calibration
  -> gyro body rad/s ---------> RateControl
  -> gravity + calibrated mag -> Mahony9 -> Euler telemetry
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
- Magnetometer service: 10 ms and deliberately after the gyro/rate loop.
- Absolute delays resynchronize and sleep one tick after an overrun to avoid starving lower-priority tasks.

DWT cycle counters record IMU period/read time, PID period/execute time and full pipeline time. Statistics use a sequence-counter snapshot because writer and reader run in different task contexts.

## ICM20948/AK09916 interface

ICM20948 communicates over SPI1 with explicit bank selection. The embedded AK09916 is accessed through the ICM auxiliary-I2C master and exposed through external-sensor shadow registers. Initialization stops existing auxiliary transfers before resetting the ICM to avoid leaving the magnetometer bus stuck after an MCU-only reset.

Magnetometer zero vectors and overflow are invalid. The estimator may use `Mahony9_UpdateImu` without mag; the gyro rate loop must continue independently of attitude initialization.

## Motor hardware contract

- TIM3 CH1..CH4 drive M1..M4. Temporary `APP_DSHOT_TEST_ENABLE=1` selects a DShot300 30% bench test (15 s of zero frames first); DShot600 uses the same flow when selected; see `control-safety.md`. With that flag disabled, PWM is the default build at 50 Hz.
- PWM stop is 1000 µs with bounds 1000..2000 µs.
- DShot300/600 use the same channels and one TIM3_UP DMA burst. At the current 60 MHz timer clock they use 200/100 ticks per bit, respectively. App derives the period from the actual timer clock. TIM3_UP DMA1 Stream0 and its IRQ are already configured in the current generated source; verify them after CubeMX regeneration.
- Motor order/direction and calibrated idle floors are documented in `stm32cube/README.md`.
- STM32, ESP and ESC signal grounds must be common; UART is 3.3 V logic.

Never alter CubeMX-generated files casually. Prefer component/user-code regions and follow the STM32-specific agent workflow before editing under `stm32cube/`.
