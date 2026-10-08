# Drone angle and rate control

`ANGLE_MODE` uses the 6-axis Mahony output to self-level roll/pitch through an
outer proportional loop and the existing inner body-rate PID. `ACRO_MODE`, or
neither mode flag, selects direct body-rate control. Both flags together are
invalid. The browser joystick defaults to ANGLE and offers an ACRO selector.

ANGLE roll/pitch sticks ±1000 map to ±30 degrees per axis. The shortest
`conjugate(current) * target` quaternion error produces body roll/pitch rate
targets with gain 4/s and a joint 100 deg/s limit. The current and target yaw
are equal, so absolute heading cancels. Yaw remains pilot body-rate control
±150 deg/s; Mahony6 yaw can drift and is not used for heading hold.
Outer-loop settings live in `RateControl/Inc/rate_control.h`.

The inner controller implements three independent body-rate PID controllers.
In ACRO the pilot ranges are:

- roll and pitch command range: `-200 .. +200 deg/s`
- yaw command range: `-150 .. +150 deg/s`
- input measurements: calibrated BODY FRD gyroscope rates in `rad/s`
- output: normalized mixer correction, not PWM microseconds

The PID uses derivative-on-measurement, a first-order derivative low-pass
filter, conditional integration, integral limiting and output limiting.
Controller state is reset whenever the vehicle is disarmed, enters failsafe,
has zero collective throttle, or receives an invalid sample interval.

The gains in `drone_control.c` and the angle-loop header are initial bench values. They are
not airframe-independent and must be tuned on the actual vehicle.

## Samples and safety

App updates Mahony6 before calling `DroneControl_UpdateFlightSample` with the
same sample's Euler degrees, BODY-FRD gyro rad/s and DWT-derived seconds. The
1 ms flight task runs both loops only for fresh data-ready IMU samples (nominal
ODR 1125 Hz). The angle P gain sets a lower initial bandwidth than the inner
PID; actual timing and stability must be verified on hardware.

ANGLE requires finite valid attitude younger than 20 ms to ARM and continue
running. Invalid/stale attitude, including stalled sample intervals, stops
motors and latches failsafe. A fresh estimate alone cannot re-arm: send explicit
zero-throttle DISARM first. Mode changes while ARM remains set also fail safe;
the web selector sends DISARM with zero commands before changing mode. A
runtime IMU reinitialization invalidates the sample and stops armed output
before any blocking sensor setup/calibration, including in ACRO.

For DShot300/600, an accepted ARM with throttle zero starts a pre-arm sequence.
`MotorDshot_Start` sends the first value-zero frame; the 1 ms flight task
continues value-zero frames every 2 ms for at least 1000 ms, with at least 100 successful
frames and no gap above 20 ms. The status remains DISARMED and positive
throttle is rejected until the sequence completes. A long frame gap restarts
the continuous window; a 2000 ms total timeout latches failsafe. DISARM,
e-stop, link loss, invalid ANGLE attitude, mode change or motor error cancels
the sequence. PWM has no DShot pre-arm delay. The browser unlocks pilot inputs
only after status reports ARMED. Set the bench timing policy in
`DroneControl/Inc/drone_control.h`.

DShot300/600 commands use a nominal 500 Hz output cadence (2 ms), while the
flight task and fresh-sample PID remain at their existing 1 ms cadence. UART
and sample handlers stage the latest motor bank; App calls
`DroneControl_ServiceMotorOutput` after the IMU/PID step. The service repeats
armed-zero and MOTOR_TEST frames even without new inputs, rechecks safety, and
publishes motor telemetry only after a successful commit. DMA BUSY retries
on a later tick; delayed calls send one frame without catch-up bursts.
DISARM/e-stop/failsafe stop immediately. PWM keeps its 50 Hz timer.
`MOTOR_OUTPUT_DSHOT_PERIOD_MS` in `Motor/Inc/motor.h` defines the 2 ms cadence;
DShot bitrates are unchanged. Verify actual frame intervals on the ESC pins.

Host checks: `make -C stm32cube/Tests/drone_control test`,
`make -C stm32cube/Tests/mahony test`, and rate-control/protocol/mixer tests.
They verify units, restoring signs, quaternion coupling, command limits,
freshness, mode switching, DShot zero-frame pre-arm, re-arm sequencing, and motor stop paths.
Hardware verification still requires propellers removed: check mounting/axis
signs, physical motor order/spin, corrective response, measured loop timing,
watchdogs, and gains before any restrained-rig test.

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

`DroneControl` normalizes the four mixer results to `0.0..1.0` and commits the
complete bank through `MotorOutput`; it has no dependency on `MotorPwm` or
`MotorDshot`. PWM-specific idle floors live in the PWM driver. DShot maps a
positive normalized value to its configured `48..2047` range.

Disarm and armed-zero-throttle remain `1000 us`. The pilot throttle command is
mapped before mixing: command 1 is `1225 us`, command 100 is about `1339 us`,
command 250 is about `1512 us`, command 400 is about `1685 us`, and command
500 is `1800 us`. This removes the former `1..225` flat region at the motor
idle floor. PID corrections remain logical mixer commands and may drive an
individual motor above the pilot collective, up to the `2000 us` actuator
limit. The browser motor-test page sends MOTOR_TEST to apply the selected
motor level directly, without PID or Quad-X mixing. This is open-loop output,
not measured RPM control. ARM, e-stop and link timeout still stop the motors.
