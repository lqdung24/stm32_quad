# Drone control wire protocol

The same transport-independent codec is compiled by the ESP32-S3 and STM32
firmware. Multi-byte integers are little-endian. A packet ends with CRC-16
CCITT-FALSE (polynomial `0x1021`, initial value `0xFFFF`).

## Common header (16 bytes)

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | Magic `0xA55A` |
| 2 | 1 | Version `1` |
| 3 | 1 | Packet type |
| 4 | 2 | Sequence |
| 6 | 2 | Session ID |
| 8 | 2 | Flags |
| 10 | 1 | Payload length |
| 11 | 1 | Reserved, must be zero |
| 12 | 4 | Sender uptime in ms |

The CRC is the final two bytes and covers the header plus payload.

## Packet types used in phase 1

- `CONTROL_COMMAND` (`0x01`): 12-byte payload containing throttle, signed
  roll/pitch/yaw, and AUX1/AUX2. The total packet is 30 bytes.
- `SYSTEM_STATUS` (`0x02`): 14-byte payload containing the last command
  sequence, requested/applied throttle, PWM pulse, flight state, errors, and
  UART receive rate. The total packet is 32 bytes.

Throttle uses a logical range of 0-1000. The STM32 clamps collective throttle
to `DRONE_CONTROL_MAX_TEST_THROTTLE` (500) and applies the same pulse to all
four motors.

## UART framing

Every raw packet is COBS-encoded and terminated by `0x00`. The link is
460800 baud, 8 data bits, no parity, one stop bit. The WebSocket carries the
raw binary packet without COBS.

## Session and fail-safe rules

- The browser creates a new non-zero random session ID on every WebSocket
  connection.
- Duplicate and old sequences are rejected with wrap-around-safe comparison.
- A new session always disarms and requires an explicit zero-throttle disarm
  command before arming.
- Phone-to-ESP and ESP-to-STM command watchdogs are both 300 ms.
- Emergency stop and either watchdog disarm the PWM output and latch a
  fail-safe. Recovery requires a zero-throttle disarm command followed by a
  new arm request.

Run the host codec and COBS tests with:

```sh
make -C Tests/protocol clean test
```
