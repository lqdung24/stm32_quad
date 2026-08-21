# ESP32-S3 USB / ESP-NOW drone bridge

This firmware replaces the SoftAP, HTTP, and WebSocket bridge with two
ESP32-S3 boards:

```text
Laptop browser -- Web Serial over USB --> ground ESP32 -- ESP-NOW --> air ESP32 -- UART --> STM32H7
```

`DroneProtocol` packets are not translated on either ESP. The ground bridge
accepts only valid control packets (including the existing CRC), and the air
bridge accepts only valid control packets before forwarding them to STM32.
Status and flight telemetry travel back as the same raw protocol bytes.
Loss of USB or ESP-NOW stops control packets; the existing STM32 command-link
watchdog remains the motor failsafe authority.

Link state is derived from valid protocol traffic rather than from
`esp_now_send()` alone. The Air bridge treats fresh control packets as proof of
the Ground-to-Air direction. Ground treats a fresh status packet whose
`last_control_sequence` acknowledges the current browser session as proof of
the return direction. If Air stops receiving valid STM32 status for 300 ms, it
sends a synthetic `FAILSAFE` status with `UART_LINK_LOST` to Ground so the UI can
show ESP-NOW online while reporting the STM32 link offline.

## Firmware roles

Flash the same project for the two chip targets. The role is selected from the
target and is no longer stored in `sdkconfig`:

- **ESP32-S3 = Ground**: `Web Serial USB to ESP-NOW`; it needs no UART wiring.
- **ESP32 = Air**: `ESP-NOW to STM32 UART`. Connect GPIO17 to STM32 PA10, GPIO18 to
  STM32 PA9, and join grounds. UART remains 460800, 8-N-1 by default.

At first boot each ESP logs its STA MAC. Put both peer MACs and the shared
channel in [`main/bridge_config.h`](main/bridge_config.h). Editing this header
uses the normal incremental build instead of regenerating `sdkconfig`. The
receive path rejects packets from other MAC addresses. Do this with props
removed. ESP-NOW is currently unencrypted, so
operate on an isolated channel and add ESP-NOW link-layer encryption before
flying outside a controlled test area.

```sh
cd esp_controller
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Laptop control page

The standalone browser files are in [`../web_controller`](../web_controller).
Serve that directory from localhost, then open it in Chrome or Edge on the
laptop (Web Serial requires a secure context; `localhost` qualifies):

```sh
cd web_controller
python3 -m http.server 8000
```

Open `http://localhost:8000`, press **VÀO ĐIỀU KHIỂN**, and choose the ground
ESP's native USB Serial/JTAG device. The page frames bytes with COBS for USB;
each ESP-NOW message and UART packet contains the unchanged DroneProtocol
payload. Do not use a serial monitor on the same USB port while controlling.

Ground prints a one-line link summary every second on UART0 and mirrors that
single line as a zero-delimited debug frame on native USB Serial/JTAG. A serial
monitor can therefore read either port during bring-up. The browser discards
the debug frame without confusing it with a valid protocol packet.

```text
LINK ground usb=UP espnow=UP stm32=UP ctrl=40/s status=20/s tele=50/s invalid=0/s tx_ok=40/s tx_fail=0/s rx_drop=0 seq=1234 ack=1232 err=0x0000
```

`usb` means fresh valid browser control, `espnow` requires a fresh matching
control acknowledgement, and `stm32` additionally requires status without
`UART_LINK_LOST`.

Keep propellers removed during bring-up. Protocol format and its host tests
remain in [`../stm32cube/Components/DroneProtocol`](../stm32cube/Components/DroneProtocol).
