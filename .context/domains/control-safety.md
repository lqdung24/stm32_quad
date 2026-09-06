# Domain: flight control and safety

## Interfaces and owners

- Browser command producer: `web_controller/app.js::createControl`.
- STM32 command/state owner: `stm32cube/Components/DroneControl`.
- Body-rate controller: `stm32cube/Components/RateControl`.
- Quad-X allocation: `stm32cube/Components/MotorMixer`.
- Timer output gate: `stm32cube/Components/MotorPWM`.

Read the `function-flow.md` in each directory before changing its behavior.

## Acro/rate mode

Acro is a cascaded-input rate controller, not an angle-hold controller. Browser roll/pitch/yaw commands are normalized wire values. `RateControl_SetCommand` maps them to desired body rates; calibrated gyro body rates are the measurements.

The current controller is full PID on roll and pitch. Yaw is PI because its configured `kd` is zero:

| Axis | Kp | Ki | Kd | Maximum target rate |
|---|---:|---:|---:|---:|
| Roll | 45.0 | 20.0 | 0.8 | 200 deg/s |
| Pitch | 45.0 | 20.0 | 0.8 | 200 deg/s |
| Yaw | 35.0 | 10.0 | 0.0 | 150 deg/s |

Derivative is taken on measurement and may be low-pass filtered. Integral has a hard limit plus conditional anti-windup. These are initial bench gains and require airframe tuning before flight.

## State transition contract

```text
BOOT -> DISARMED                 successful initialization
DISARMED -> ARMED               ARM requested, throttle/axes zero, disarm cycle satisfied
ARMED -> DISARMED               explicit ARM-clear command
ARMED -> FAILSAFE               e-stop, timeout or invalid unsafe transition
FAILSAFE -> DISARMED            explicit ARM-clear + zero throttle
any -> ERROR                    unrecoverable PWM/control initialization or output failure
```

A new session always disarms and sets `require_disarm_cycle`. Duplicate or old sequence numbers are rejected with wrap-aware comparison. While armed, changing motor-selection with nonzero throttle is rejected as unsafe.

## Actuator path

Throttle is policy-clamped, mapped to a collective PWM range, combined with the latest PID correction and passed into Quad-X mixing. Mixer saturation first shifts collective to preserve correction authority; it scales corrections only if their span cannot fit the actuator range. Active outputs receive per-motor idle floors. `MotorPwm_SetAllPulseUs` validates all four values before touching hardware.

## Watchdogs and defense in depth

- Browser sends every 25 ms and enters local safe state when ACK/status becomes stale.
- Ground sends fresh explicit DISARM keepalives if browser control stops.
- Air reports a synthetic UART-loss status if Ground is alive but STM32 status is stale.
- STM32 command watchdog is authoritative and physically disarms PWM on timeout.

Do not weaken a downstream check because an upstream layer already checks the same condition.

## Required verification for changes

- Protocol decode/range/sequence tests for command changes.
- Rate-control unit tests for PID math or timing changes.
- Mixer tests for signs, saturation and bounds.
- Confirm disarmed PWM after startup, session change, e-stop, malformed packet and timeout.
- Hardware motor tests must be performed without propellers first.
