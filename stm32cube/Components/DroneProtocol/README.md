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
- `SYSTEM_STATUS` (`0x02`): 20-byte payload containing the last command
  sequence, requested/applied throttle, four PWM values, flight state, errors,
  and UART receive rate. The total packet is 38 bytes.

Control flag bit 2 selects ANGLE and bit 3 selects ACRO. Setting both is an
encode/decode error; setting neither preserves legacy ACRO. Wire roll/pitch/yaw
remain signed ±1000. ANGLE maps roll/pitch to ±30 degrees and uses fresh Mahony6
feedback; ACRO maps them to ±200 deg/s. Yaw is body rate ±150 deg/s in either
mode. Changing mode requires DISARM; ANGLE arming also requires valid attitude
less than 20 ms old. The packet version and layout are unchanged.

Throttle uses a logical range of 0-1000. Bit 5 selects MOTOR_TEST and is
mutually exclusive with ANGLE/ACRO. In MOTOR_TEST, roll/pitch/yaw must be zero;
AUX1 selects all motors (`0`) or M1..M4 (`1..4`). Outside MOTOR_TEST, AUX1
must be zero. AUX2 is always zero. Changing modes while armed or changing
motor selection with nonzero throttle enters failsafe. MOTOR_TEST applies the
selected motor level directly without PID or Quad-X mixing; ARM, DShot zero
pre-arm, e-stop and link watchdog still apply.

The selector order follows the verified physical layout: M1 is front-left
(`PA6`), M2 rear-left (`PA7`), M3 front-right (`PB0`), and M4 rear-right
(`PB1`). Motor rotation and calibrated idle pulses are documented in the
[`DroneControl` README](../DroneControl/README.md).

## Synchronized flight telemetry

`FLIGHT_TELEMETRY_SYNC` (type `0x08`, version 1) is 58 bytes: a 16-byte
header, 40-byte payload and CRC at offset 56. Control and status layouts
are unchanged. The encoder emits type 8; C, browser and Python decoders also
accept legacy type `0x07` (50 bytes) with its original flag mask.

| Raw offset | Type | Field |
|---:|---|---|
| 12 | uint32 | IMU sample time in HAL milliseconds (header) |
| 16 | int16[3] | Roll/pitch/yaw, centidegrees |
| 22 | int16[3] | Body gyro, mrad/s |
| 28 | int16[3] | Rate setpoint, mrad/s |
| 34 | int16[3] | Combined PID correction, centi-units |
| 40 | uint16[4] | Legacy motor output representation, 1000..2000 |
| 48 | uint32 | Sample ID, wraps modulo 2^32 |
| 52 | uint32 | Motor driver commit time in HAL milliseconds |
| 56 | uint16 | CRC16 |

Header flags contain state (bits 0..2), actuator-active (3), attitude-valid
(4), and output-sample-matched (5). Bit 5 means the gyro/setpoint/PID and
motor bank belong to the same flight-control sample. Test/zero/disarmed
output is independent of the sensor sample and clears bit 5. Commit time
means successful driver submission, not completion of DMA or measured RPM.
With DShot, the normalized motor field remains a compatibility mapping,
not a physical PWM pulse width.

Active DShot snapshots complete only after a successful commit; BUSY keeps
the previous complete snapshot, and newer IMU samples replace pending ones.
Telemetry sends the latest complete snapshot at most once, nominally 50 Hz.
Sample-ID gaps are expected; transport loss is measured by packet sequence.
Stop/zero-throttle/IMU invalidation flush snapshots. Samples older than 50 ms
or from a different state/session are discarded. Commit delay is unsigned
`commit_time - sample_time`, valid only when bit 5 is set.

Both ESPs must be rebuilt/flashed with the 58-byte shared packet capacity
before the updated STM32 firmware sends type 8. Updated host decoders can
still read old STM32 telemetry; old host decoders cannot read type 8.

## UART framing

Every raw packet is COBS-encoded and terminated by `0x00`. The link is
460800 baud, 8 data bits, no parity, one stop bit. Web Serial also uses COBS; ESP-NOW carries raw packets without COBS.

## Session and fail-safe rules

- The browser creates a new non-zero random session ID on every Web Serial
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
