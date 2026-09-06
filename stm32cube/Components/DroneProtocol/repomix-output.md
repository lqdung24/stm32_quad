This file is a merged representation of the entire codebase, combined into a single document by Repomix.
The content has been processed where content has been compressed (code blocks are separated by ⋮---- delimiter).

# File Summary

## Purpose
This file contains a packed representation of the entire repository's contents.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.

## File Format
The content is organized as follows:
1. This summary section
2. Repository information
3. Directory structure
4. Repository files (if enabled)
5. Multiple file entries, each consisting of:
  a. A header with the file path (## File: path/to/file)
  b. The full contents of the file in a code block

## Usage Guidelines
- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.

## Notes
- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Content has been compressed - code blocks are separated by ⋮---- delimiter
- Files are sorted by Git change count (files with more changes are at the bottom)

# Directory Structure
````
Inc/
  dp_bytes.h
  dp_cobs.h
  dp_protocol.h
Src/
  dp_bytes.c
  dp_cobs.c
  dp_protocol.c
README.md
````

# Files

## File: Inc/dp_bytes.h
````c
uint16_t DroneProtocol_ReadU16Le(const uint8_t *data);
int16_t DroneProtocol_ReadI16Le(const uint8_t *data);
uint32_t DroneProtocol_ReadU32Le(const uint8_t *data);
void DroneProtocol_WriteU16Le(uint8_t *data, uint16_t value);
void DroneProtocol_WriteI16Le(uint8_t *data, int16_t value);
void DroneProtocol_WriteU32Le(uint8_t *data, uint32_t value);
⋮----
#endif /* DP_BYTES_H */
````

## File: Inc/dp_cobs.h
````c
/*
 * Return encoded/decoded size, or 0 on invalid input/capacity.
 * The encoded result does not include the UART 0x00 frame delimiter.
 */
size_t DroneCobs_Encode(const uint8_t *input, size_t input_length,
⋮----
size_t DroneCobs_Decode(const uint8_t *input, size_t input_length,
````

## File: Inc/dp_protocol.h
````c
/* Flight telemetry flags live in the packet header, not in its payload. */
⋮----
/* AUX1 motor selection used only by the no-prop threshold-test mode. */
⋮----
} DronePacketType;
⋮----
} DroneSystemState;
⋮----
} DroneProtocolResult;
⋮----
} DronePacketHeader;
⋮----
} DroneControlCommand;
⋮----
} DroneSystemStatus;
⋮----
/*
     * Packed payload layout (all values are little-endian):
     * attitude_cdeg[3], gyro_mrad_s[3], rate_setpoint_mrad_s[3],
     * pid_command_centi[3], motor_pwm_us[4].
     * The state and validity bits are carried in header.flags.
     */
⋮----
} DroneFlightTelemetry;
⋮----
uint16_t DroneProtocol_Crc16CcittFalse(const uint8_t *data, size_t length);
⋮----
DroneProtocolResult DroneProtocol_EncodeControl(
⋮----
DroneProtocolResult DroneProtocol_DecodeControl(
⋮----
DroneProtocolResult DroneProtocol_EncodeStatus(
⋮----
DroneProtocolResult DroneProtocol_DecodeStatus(
⋮----
DroneProtocolResult DroneProtocol_EncodeFlightTelemetry(
⋮----
DroneProtocolResult DroneProtocol_DecodeFlightTelemetry(
⋮----
bool DroneProtocol_IsSequenceNewer(uint16_t candidate, uint16_t reference);
````

## File: Src/dp_bytes.c
````c
uint16_t DroneProtocol_ReadU16Le(const uint8_t *data)
⋮----
int16_t DroneProtocol_ReadI16Le(const uint8_t *data)
⋮----
uint32_t DroneProtocol_ReadU32Le(const uint8_t *data)
⋮----
void DroneProtocol_WriteU16Le(uint8_t *data, uint16_t value)
⋮----
void DroneProtocol_WriteI16Le(uint8_t *data, int16_t value)
⋮----
void DroneProtocol_WriteU32Le(uint8_t *data, uint32_t value)
````

## File: Src/dp_cobs.c
````c
size_t DroneCobs_Encode(const uint8_t *input, size_t input_length,
⋮----
size_t DroneCobs_Decode(const uint8_t *input, size_t input_length,
````

## File: Src/dp_protocol.c
````c
static void encode_header(const DronePacketHeader *header, uint8_t *output)
⋮----
static DroneProtocolResult decode_header(const uint8_t *packet,
⋮----
static void append_crc(uint8_t *packet, size_t packet_length)
⋮----
uint16_t DroneProtocol_Crc16CcittFalse(const uint8_t *data, size_t length)
⋮----
DroneProtocolResult DroneProtocol_EncodeControl(
⋮----
DroneProtocolResult DroneProtocol_DecodeControl(
⋮----
DroneProtocolResult DroneProtocol_EncodeStatus(
⋮----
DroneProtocolResult DroneProtocol_DecodeStatus(
⋮----
DroneProtocolResult DroneProtocol_EncodeFlightTelemetry(
⋮----
DroneProtocolResult DroneProtocol_DecodeFlightTelemetry(
⋮----
bool DroneProtocol_IsSequenceNewer(uint16_t candidate, uint16_t reference)
````

## File: README.md
````markdown
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
````
