# Domain: DroneProtocol and framing

## Canonical interface

The source of truth is `stm32cube/Components/DroneProtocol/Inc/dp_protocol.h` plus its C implementation and host tests. ESP `common/CMakeLists.txt` compiles those same C files rather than maintaining a fork. Browser and Python implementations must match this contract manually.

## Packet layers

1. DroneProtocol raw packet: fixed header, fixed payload per type and CRC16.
2. USB/UART stream: COBS-encoded raw packet followed by delimiter `0x00` (implementations may also prepend a delimiter for resynchronization).
3. ESP-NOW: exactly one unmodified raw packet per radio message; no COBS layer.

## Header contract

- Magic `0xA55A`, version `1`.
- Little-endian multi-byte values.
- Type, sequence, session, flags, payload length, reserved byte and sender time are at fixed offsets.
- Packet size and payload size are exact, not minimums.
- Reserved bits/bytes must be zero unless the version explicitly assigns them.
- CRC is CRC-16/CCITT-FALSE over all bytes before the final two CRC bytes.

## Packet types in active use

- `CONTROL_COMMAND`: browser to STM32; throttle, roll, pitch, yaw, motor selection and safety flags.
- `SYSTEM_STATUS`: STM32 to browser; acknowledged control sequence, requested/applied throttle, PWM, system state, error flags and UART rate.
- `FLIGHT_TELEMETRY`: STM32 to host; attitude, gyro, rate setpoint, PID correction, motor PWM and validity/activity flags.

Consult `DroneProtocol/function-flow.md` for encode/decode flow and `dp_protocol.h` for exact sizes/offsets/enums.

## Session and sequence semantics

- Browser creates a random nonzero 16-bit session when a serial connection opens.
- A session change forces STM32 disarm and resets sequence acceptance.
- Sequence comparison is modulo 16-bit; a candidate is newer when the forward distance is nonzero and less than `0x8000`.
- Ground considers status an ACK only for the active session and when ACK lag is no more than 16 packets.
- Sequence increments only after constructing/sending the corresponding logical message as specified by each producer.

## Compatibility rule

Any field, flag, type, size, range or scale change must update together:

- STM32 header/encoder/decoder and tests.
- ESP builds that compile the canonical implementation.
- `web_controller/app.js` encoding/status decoding.
- `tools/telemetry_plot.py` decoder/self-test when telemetry changes.
- Domain/context and function-flow documentation.

Never accept unknown flags by default; version or explicitly extend the allowed mask.
