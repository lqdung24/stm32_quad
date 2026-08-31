# ESP-NOW drone link

The radio link is split into two independent ESP-IDF projects because Ground
and Air use different chips:

```text
Laptop browser -- USB --> Ground ESP32-S3 -- ESP-NOW --> Air ESP32 -- UART --> STM32H7
```

```text
esp_controller/
├── common/       shared ESP-NOW, framing, LED, and DroneProtocol component
├── ground_s3/    ESP32-S3 project: Web Serial USB <-> ESP-NOW
├── air_esp32/    ESP32 project: ESP-NOW <-> STM32 UART
└── rfid_s3/      standalone ESP32-S3 + MFRC522 USB command tool
```

Each project fixes `IDF_TARGET` in its own root `CMakeLists.txt` and has its own
`sdkconfig`, dependency lock, and build directory. Do not run `idf.py
set-target`; build the desired project directly.

## Configuration

Set both STA MAC addresses and the shared radio channel in
[`common/include/bridge_config.h`](common/include/bridge_config.h). Ground
explicitly uses the Air MAC as its peer, while Air explicitly uses the Ground
MAC. No role is selected from Kconfig or inferred from the target.

### Air ESP32 <-> STM32 UART

| Air ESP32 | STM32H743 | Direction / purpose |
|---|---|---|
| GPIO17, UART1 TX | PA10, USART1 RX | ESP -> STM32 control commands |
| GPIO18, UART1 RX | PA9, USART1 TX | STM32 -> ESP status and flight telemetry |
| GND | GND | Common reference; required |

The link uses UART1 at **460800 baud, 8 data bits, no parity, 1 stop bit
(8-N-1)** and no hardware flow control. Both UART sides are 3.3 V logic; do
not connect a 5 V UART signal. Packets are COBS-framed and terminated with a
`0x00` delimiter.

These UART settings and the status LED GPIO remain configurable through the
corresponding project's `idf.py menuconfig`. Ground uses a WS2812 RGB status
LED by default; Air uses an ordinary active-high status LED by default.

## Build and flash

From this directory:

```sh
# Ground ESP32-S3
idf.py -C ground_s3 build
idf.py -C ground_s3 -p /dev/ttyACM0 flash monitor

# Air ESP32
idf.py -C air_esp32 build
idf.py -C air_esp32 -p /dev/ttyUSB0 flash monitor
```

Because the build trees are separate, building or flashing one board never
changes the other board's target or configuration. When using the ESP-IDF VS
Code extension, open `ground_s3/` or `air_esp32/` as the workspace folder.

## Data path and link status

`DroneProtocol` packets are not translated by either ESP. Ground validates
browser control packets before sending them over ESP-NOW. Air validates control
packets before forwarding them to STM32. Status and flight telemetry return as
the same raw protocol bytes.

Ground derives link state from valid protocol traffic, not only from
`esp_now_send()`. Air sends a synthetic `FAILSAFE` status with
`UART_LINK_LOST` if Ground control is fresh but STM32 status has been absent for
300 ms. The STM32 command watchdog remains the final motor failsafe authority.

At first boot, both projects log their local and configured peer MAC. ESP-NOW
is currently unencrypted, so use an isolated channel and add ESP-NOW
link-layer encryption before flying outside a controlled test area.

## Laptop control page

The standalone browser controller is in `../web_controller`:

```sh
cd ../web_controller
python3 -m http.server 8000
```

Open `http://localhost:8000` in Chrome or Edge and select the Ground ESP32-S3
native USB Serial/JTAG device. Do not open a serial monitor on that same USB
port while the browser controls the drone.

Keep propellers removed during bring-up. The canonical protocol implementation
and host tests remain in `../stm32cube/Components/DroneProtocol`.
