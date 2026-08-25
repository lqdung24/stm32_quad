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

Default wiring for the Air ESP32 is:

- GPIO17 (ESP TX) -> STM32 PA10 (USART1 RX)
- GPIO18 (ESP RX) <- STM32 PA9 (USART1 TX)
- Common ground
- UART 460800, 8-N-1

These UART settings and each board's WS2812 GPIO remain configurable through
that project's `idf.py menuconfig`.

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
