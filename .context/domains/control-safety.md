# Domain: flight control and safety

## Interfaces and owners

- Browser command producer: `web_controller/app.js::createControl`.
- STM32 command/state owner: `stm32cube/Components/DroneControl`.
- Angle outer loop and body-rate controller: `stm32cube/Components/RateControl`.
- Quad-X allocation: `stm32cube/Components/MotorMixer`.
- Protocol-independent output gate: `stm32cube/Components/Motor`.
- Output drivers: `stm32cube/Components/MotorPWM`, `stm32cube/Components/MotorDshot` and its `MotorDshot300` adapter.
- Timer/PWM/DMA abstraction: `stm32cube/Components/PwmTimer`.

Read the `function-flow.md` in each directory before changing its behavior.

## Angle mode

`ANGLE_MODE` selects self-leveling roll/pitch; `ACRO_MODE` or neither flag
selects the legacy body-rate path. Both flags together are invalid. Browser
joystick defaults to ANGLE and provides an ACRO selector; changing either
flight mode or test/pilot layout sends zero-throttle DISARM first. Forward
stick commands negative BODY-FRD pitch (nose-down).

Normalized roll/pitch ±1000 map to ±30 degrees each. In `RateControl`, the
outer proportional loop forms the shortest relative quaternion
`conjugate(measured) * target`, expressed in body axes, using the same yaw
for both attitudes. Twice its x/y vector components times 4/s become roll/pitch
rate targets, jointly limited to 100 deg/s. Pilot yaw remains body yaw rate
up to ±150 deg/s. No heading hold is inferred from Mahony6 yaw.
The existing rate PID consumes these targets and calibrated BODY-FRD gyro
in rad/s. Both loops run per fresh sample; the outer gain sets a lower initial
bandwidth and needs airframe tuning.

`DroneControl_UpdateFlightSample` receives Mahony6 validity, roll/pitch degrees,
gyro rad/s, local sample timestamp and DWT `dt` together. ANGLE rejects ARM
without finite valid attitude younger than 20 ms; invalid/stale attitude while
armed enters failsafe, including at zero throttle. Freshness is checked before
UART processing and before applying collective. IMU reinitialization explicitly
invalidates sensor state and stops armed output before blocking calibration.
Mode changes with ARM still set enter failsafe; explicit DISARM/e-stop retain
priority. Stored pilot commands regenerate targets every active sample; zero
throttle, disarm, mode change and failsafe reset the rate PID.

## Acro/rate mode

Acro is a cascaded-input rate controller, not an angle-hold controller. Browser roll/pitch/yaw commands are normalized wire values. `RateControl_SetCommand` maps them to desired body rates; calibrated gyro body rates are the measurements.

The current controller is full PID on roll and pitch. Yaw is PI because its configured `kd` is zero:

| Axis | Kp | Ki | Kd | Maximum target rate |
|---|---:|---:|---:|---:|
| Roll | 45.0 | 20.0 | 0.6 | 200 deg/s |
| Pitch | 45.0 | 20.0 | 0.6 | 200 deg/s |
| Yaw | 35.0 | 10.0 | 0.0 | 150 deg/s |

Derivative is taken on measurement and may be low-pass filtered. Integral has a hard limit plus conditional anti-windup. These are initial bench gains and require airframe tuning before flight.

## State transition contract

Normal operation defaults to `APP_DSHOT_TEST_ENABLE=0` and
`APP_MOTOR_OUTPUT_PROTOCOL=APP_MOTOR_OUTPUT_DSHOT300` in
`Components/App/Inc/app.h`. `DroneControl` owns motor output through the
ARM/DISARM, watchdog, angle/rate cascade and mixer path described below.

Optional bench override: explicitly setting `APP_DSHOT_TEST_ENABLE=1` gives
the 1 ms flight task exclusive motor ownership: zero frames for 15 s, then
fixed 0.30 normalized throttle every 2 ms (value 648) on all four channels. Selecting
`APP_MOTOR_OUTPUT_DSHOT600` uses the same sequencer. The legacy
`APP_DSHOT600_TEST_ENABLE=0` disables bench mode; `=1` defaults to DShot600.
Bench mode bypasses ARM, radio watchdog, DISARM/e-stop, PID and runtime sensor
processing; remove propellers and power off ESCs to stop. Output errors latch
the test off and request Stop. See `Components/App/function-flow.md` for bench
configuration and debugger variables; normal flight telemetry does not report
bench output.

```text
BOOT -> DISARMED                 successful initialization
DISARMED -> ARMED               ARM requested, throttle zero, disarm cycle satisfied; ANGLE attitude fresh; DShot zero-frame pre-arm complete
ARMED -> DISARMED               explicit ARM-clear command
ARMED -> FAILSAFE               e-stop, timeout or invalid unsafe transition
FAILSAFE -> DISARMED            explicit ARM-clear; zero throttle also clears re-arm latch
any -> ERROR                    unrecoverable actuator/control initialization or output failure
```

DShot command banks are committed at nominal 500 Hz (every 2 ms), independently
of the 1 ms flight task and fresh-sample PID. UART/sample handlers stage the
latest bank; App calls `DroneControl_ServiceMotorOutput` after the IMU/PID step.
Armed zero throttle and MOTOR_TEST also repeat frames without new input. DMA
BUSY retries on a later tick, and missed deadlines do not trigger catch-up
bursts. DISARM/e-stop/failsafe stop immediately outside the periodic gate.
This is command update frequency; DShot300/600 bitrates remain unchanged.
PWM retains its 50 Hz timer and immediate compare updates.

Normal DShot ARM first starts the motor driver with value-zero frames. The
1 ms flight task sends zero every 2 ms for at least 1000 ms, requires at least
100 successful frames with no gap over 20 ms, and keeps state DISARMED and
applied throttle zero during that interval. A gap restarts the continuous
window; failure to complete in 2000 ms enters failsafe. Any positive throttle,
mode change while ARM is held, DISARM, e-stop, lost link or invalid ANGLE
attitude stops the sequence. PWM keeps its immediate ARM behavior. The browser
locks joystick/slider until STM32 status actually reports ARMED.

A new session always disarms and sets `require_disarm_cycle`. Clearing that latch requires an explicit ARM-clear command with zero throttle. Flight-mode ARM does not require roll/pitch/yaw commands to be zero; MOTOR_TEST packets require zero axes. Duplicate or old sequence numbers are rejected with wrap-aware comparison. While armed, changing motor-selection with nonzero throttle is rejected as unsafe. Changing between MOTOR_TEST and flight mode while ARM remains set enters failsafe.

## Actuator path

In flight mode, throttle is policy-clamped, mapped to a collective command, combined with the latest PID correction and passed into Quad-X mixing. MOTOR_TEST applies the clamped level directly to all or one selected motor without PID/mixing or an IMU freshness requirement; ARM, DShot zero pre-arm, e-stop and watchdog remain active. Mixer saturation first shifts collective to preserve correction authority; it scales corrections only if their span cannot fit the actuator range. `DroneControl` validates/stages all four normalized outputs through `MotorOutput` before one commit. PWM applies calibrated per-motor idle floors; DShot maps positive normalized values to configurable 48..2047 values and commits all channels with one TIM update DMA burst.

## Watchdogs and defense in depth

- Browser sends every 25 ms and enters local safe state when ACK/status becomes stale.
- Browser Esc key and STOP button share the emergency-stop path: cancel pending ARM, zero throttle/axes, latch emergency and send E-STOP immediately when a serial writer is available. Esc works in test/pilot mode and while form controls have focus; capture-phase handling precedes throttle hotkey gates.
- Ground sends fresh explicit DISARM keepalives if browser control stops.
- Air reports a synthetic UART-loss status if Ground is alive but STM32 status is stale.
- STM32 command watchdog is authoritative and drives the selected actuator output to its stop representation on timeout.

Do not weaken a downstream check because an upstream layer already checks the same condition.

## Required verification for changes

- Protocol decode/range/sequence tests for command changes.
- Rate-control quaternion/cascade tests, Mahony6 tests, and DroneControl UART-to-motor safety tests for angle, PID or timing changes.
- Mixer tests for signs, saturation and bounds.
- Confirm stopped actuator output after startup, session change, e-stop, malformed packet and timeout.
- Hardware motor tests must be performed without propellers first.
