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

Throttle uses a logical range of 0-1000. AUX1 retains the calibration/test
selection values (`0` for all motors and `1..4` for M1..M4), and AUX2 must
remain zero. The current STM32 build has raw threshold-test mode disabled, so
normal control must send AUX1 zero and all four outputs are controlled by the
Quad-X mixer. The STM32 still rejects invalid selections and treats a selection
change with non-zero throttle as a failsafe condition.

The selector order follows the verified physical layout: M1 is front-left
(`PA6`), M2 rear-left (`PA7`), M3 front-right (`PB0`), and M4 rear-right
(`PB1`). Motor rotation and calibrated idle pulses are documented in the
[`DroneControl` README](../DroneControl/README.md).

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
