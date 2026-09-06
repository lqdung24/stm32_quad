# Architecture and conventions

## Technology stack

- STM32: C, STM32 HAL, CMSIS-RTOS2/FreeRTOS, CubeMX-generated project structure.
- ESP: C, ESP-IDF 6.x, FreeRTOS, ESP-NOW, native USB Serial/JTAG and UART.
- Browser: static HTML/CSS and vanilla JavaScript using Web Serial.
- Host diagnostics: Python 3, `websocket-client`, CSV and optional Matplotlib.
- Wire contract: canonical C implementation in `stm32cube/Components/DroneProtocol`.

## Ownership boundaries

- The browser owns user interaction and applies conservative local safety gates, but it is not the final safety authority.
- Ground and Air ESP projects are validating transports. They forward raw DroneProtocol packets without translating fields.
- STM32 `DroneControl` is the final owner of command validity, session/sequence acceptance, state transitions, command watchdog and motor output.
- `App` owns hardware composition and scheduling; leaf components should not depend on CubeMX application globals.
- `MotorPWM` is the last software gate before timer compare registers.

## Primary data flow

Control travels browser → Ground USB → ESP-NOW → Air UART → STM32. Status and flight telemetry return in the opposite direction. USB and UART byte streams use COBS plus a `0x00` delimiter; one ESP-NOW message contains one complete raw DroneProtocol packet.

The fast STM32 pipeline is gyro sample → body-frame/unit conversion → body-rate PID → Quad-X mixer → PWM. Attitude estimation is parallel telemetry/support state: a missing magnetometer may degrade Mahony9 to IMU-only, but must not stop the gyro rate loop.

## Naming rules

- STM32 public C functions: `Component_Action`, for example `RateControl_Update`.
- STM32 types: component-prefixed PascalCase; enum/constants use upper snake case.
- ESP public component functions: lower snake case with module prefix, for example `espnow_transport_send_packet`.
- File-local state/functions are `static`; shared mutable state must state its ISR/task locking model.
- JavaScript state/functions use lower camel case; protocol constants use upper snake case.
- Units belong in names: `_ms`, `_us`, `_rad_s`, `_deg`, `_cdeg`, `_cuT`, `_raw`.

## Frames and units

- Body frame is FRD: +X forward, +Y right, +Z down.
- Gyroscope control input is rad/s.
- Pilot axes on the wire are signed integers in `[-1000, 1000]`; throttle is `[0, 1000]`, while current UI/test policy limits requested throttle to 500.
- PWM is expressed in microseconds. Disarmed output is 1000 µs; configured output must remain within 1000..2000 µs.
- Multi-byte protocol fields are little-endian. Raw ICM accel/gyro registers are assembled in sensor register byte order; AK09916 shadow fields use their own ordering.

## Safety invariants

- Never bypass DroneProtocol validation before a control command reaches the STM32 state machine.
- Startup, new session, emergency stop and failsafe require an explicit zero-throttle DISARM cycle before re-arming.
- Loss of fresh commands must disarm at STM32, independent of browser and ESP behavior.
- Validate an entire four-motor output before writing any channel.
- Keep high-latency magnetometer/diagnostic work after the gyro/rate-control work.
- Test motor behavior without propellers until the complete link, motor order, direction and failsafe behavior are verified.

## Source-of-truth order

1. Hardware configuration and implementation source.
2. Protocol headers plus protocol tests.
3. Component README and `function-flow.md`.
4. `.context` navigation documents.
5. Generated `repomix-output.md` snapshots.

If levels disagree, do not silently copy stale context downward. Verify the implementation and update the documentation that is stale.
