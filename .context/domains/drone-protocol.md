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

- `CONTROL_COMMAND`: browser to STM32; throttle, roll, pitch, yaw, motor selection and safety flags. ANGLE bit 2, ACRO bit 3 and MOTOR_TEST bit 5 are mutually exclusive; when all three are clear, the legacy ACRO path is used. MOTOR_TEST requires zero axes and uses AUX1=0 (all) or 1..4 (one motor); other modes require AUX1=0. Wire axes remain ±1000; firmware maps roll/pitch to ±30 degrees in ANGLE or ±200 deg/s in ACRO, yaw to ±150 deg/s in both. No packet layout, scale, or telemetry flag changes.
- `SYSTEM_STATUS`: STM32 to browser; acknowledged control sequence, requested/applied throttle, PWM, system state, error flags and UART rate.
- `FLIGHT_TELEMETRY_SYNC` (type 8): STM32 to host; 58 bytes, payload 40. First 32 payload bytes retain attitude/gyro/rate-setpoint/PID/motor layout. Uint32 sample ID and motor commit time are appended at raw offsets 48/52; CRC moves to 56. Header time is IMU sample time; bit 5 marks matched IMU/PID/output. Legacy `FLIGHT_TELEMETRY` type 7 stays 50 bytes with the old flag mask. C/browser/Python decoders accept both exact layouts; encoders emit type 8. Shared bridge capacity is 58 bytes, so both ESPs must be rebuilt/flashed before using new STM32 telemetry.

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
