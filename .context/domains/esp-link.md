# Domain: ESP bridge and link health

## Projects

- `ground_s3`: ESP32-S3, native USB Serial/JTAG to browser and ESP-NOW to Air.
- `air_esp32`: ESP32, ESP-NOW to Ground and UART to STM32.
- `common`: shared ESP-NOW transport, COBS packet stream and status LED.

The two projects have separate targets/build trees. Do not use `idf.py set-target`; build from the specific project directory or with `idf.py -C`.

## Forward paths

Ground validates browser control packets, records session/sequence, then sends the raw packet through ESP-NOW. Air validates the same packet again, records heartbeat, COBS-frames it and writes STM32 UART. Return status/telemetry travels unchanged from STM32 UART through Air ESP-NOW and Ground USB.

## Link health definitions

- Ground `USB online`: a valid browser control arrived within 300 ms.
- Ground `ESP-NOW online`: a matching-session status ACK with lag ≤16 arrived within 300 ms.
- Ground `STM32 online`: ESP-NOW online, fresh status and no UART-loss error flag.
- Air `Ground online`: valid control received within 300 ms.
- Air `STM32 online`: valid status received within 300 ms.

Radio send-success is transport delivery information, not proof that STM32 accepted the command.

## Failure behavior

- Browser silence: Ground synthesizes new-sequence, zero-throttle DISARM commands every 20 ms so the downstream link remains explicitly safe.
- Ground alive but STM32 silent: Air sends synthetic FAILSAFE/UART_LINK_LOST status upstream.
- UART/control silence at STM32: authoritative command watchdog disarms motors.
- RX callbacks do minimal work and enqueue/copy; application callbacks run in dedicated tasks.

## Configuration interface

Peer STA MAC addresses and channel are in `esp_controller/common/include/bridge_config.h`. Air UART port, pins and baud are project Kconfig values. Current UART contract is 460800 baud, 8-N-1, no flow control and 3.3 V logic.

ESP-NOW is currently unencrypted. Treat it as suitable only for controlled bring-up until link-layer encryption and key management are designed.
