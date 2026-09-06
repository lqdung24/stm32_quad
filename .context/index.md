# Repository context index

Read this file first. It is the routing map for the workspace; open only the domain documents relevant to the current task.

## System map

```text
Browser controller
  web_controller/
       | Web Serial, COBS-framed DroneProtocol
       v
Ground ESP32-S3
  esp_controller/ground_s3/
       | ESP-NOW, raw DroneProtocol packet
       v
Air ESP32
  esp_controller/air_esp32/
       | UART 460800 8-N-1, COBS-framed DroneProtocol
       v
STM32H743 flight controller
  stm32cube/Components/App
       +-> DroneControl -> RateControl -> MotorMixer -> MotorPWM -> ESCs
       +-> ICM20948 -> Attitude -> Mahony9 -> attitude/rate telemetry

RFID utility (standalone)
  esp_controller/rfid_s3/ -> RC522 over SPI + USB command console
```

## Directory map

| Path | Responsibility | Detail document |
|---|---|---|
| `web_controller/` | Browser UI, Web Serial, control packet generation, local link watchdog | `domains/control-safety.md`, `domains/drone-protocol.md` |
| `esp_controller/ground_s3/` | USB ↔ ESP-NOW ground bridge, keepalive and link health | `domains/esp-link.md` |
| `esp_controller/air_esp32/` | ESP-NOW ↔ STM32 UART air bridge, synthetic UART-loss status | `domains/esp-link.md` |
| `esp_controller/common/` | Shared ESP-NOW, COBS byte stream and status LED | `domains/esp-link.md`, `domains/drone-protocol.md` |
| `esp_controller/rfid_s3/` | Standalone RC522/MIFARE CLI | `domains/rfid.md` |
| `stm32cube/Components/App/` | Hardware composition, RTOS scheduling, IMU pipeline and diagnostics | `domains/hardware-sensors.md` |
| `stm32cube/Components/DroneControl/` | Command validation, state machine, failsafe and telemetry TX | `domains/control-safety.md` |
| `stm32cube/Components/RateControl/` | Three-axis body-rate PID | `domains/control-safety.md` |
| `stm32cube/Components/MotorMixer/` | Quad-X mixing and saturation handling | `domains/control-safety.md` |
| `stm32cube/Components/MotorPWM/` | Timer compare output and arm/disarm gate | `domains/control-safety.md` |
| `stm32cube/Components/DroneProtocol/` | Canonical packet, CRC, endian and COBS implementation | `domains/drone-protocol.md` |
| `stm32cube/Components/ICM20948/` | SPI IMU and internal-I2C AK09916 driver | `domains/hardware-sensors.md` |
| `stm32cube/Components/Attitude/` | Sensor-to-body frames, units and calibration | `domains/hardware-sensors.md` |
| `stm32cube/Components/Mahony*` | 6-axis/9-axis attitude filters | `domains/hardware-sensors.md` |
| `tools/` | Host telemetry receiver, CSV recorder and plots | `domains/telemetry.md` |
| `simulation/` | Reserved; currently empty | This index |

## Domain routing

- Architecture, technology, naming or ownership: `architecture.md`
- ARM/DISARM, acro, PID, mixer, motor output or failsafe: `domains/control-safety.md`
- Packet fields, CRC, COBS, sequence or compatibility: `domains/drone-protocol.md`
- ESP-NOW, USB/UART bridges, peer MAC or link status: `domains/esp-link.md`
- IMU, coordinate frame, units, calibration, Mahony or RTOS sampling: `domains/hardware-sensors.md`
- Flight telemetry, CSV, plots or status visualization: `domains/telemetry.md`
- RC522, MIFARE, key, sector, read/write: `domains/rfid.md`

## Function-level references

Each implemented module has a `function-flow.md` beside its source and a generated `repomix-output.md`. Prefer the smaller `function-flow.md` first; inspect source before changing behavior. Repomix files are generated snapshots and may lag behind later edits.

## Navigation tools

```sh
tools/read_context architecture.md
tools/read_context domains/drone-protocol.md
tools/search_context sequence
```
