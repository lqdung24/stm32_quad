# ESP32-S3 drone control bridge

This ESP-IDF 6.0 project creates a Wi-Fi hotspot and serves a mobile web page.
The page sends binary control packets over WebSocket. The ESP32-S3 validates
and forwards them over UART to the STM32H7, which is the only controller that
can drive an ESC.

The STM32 applies collective throttle to all four motors and clamps it to
500/1000 (a maximum 1500 us pulse).

## Safety first

Remove every propeller during bring-up. Power the motors/ESC from a suitable
external supply; never from either development board. Join the ESP32, STM32,
and ESC signal grounds. Verify the selected GPIOs against the exact ESP32-S3
board before wiring power.

## Default wiring

| ESP32-S3 | STM32H743 | Purpose |
|---|---|---|
| GPIO17 (UART1 TX) | PA10 (USART1 RX) | Commands to STM32 |
| GPIO18 (UART1 RX) | PA9 (USART1 TX) | Status to ESP32 |
| GND | GND | Common signal ground |

STM32 PA6, PA7, PB0, and PB1 are TIM3 channels 1-4 for motors 1-4.

UART settings are 460800 baud, 8-N-1.

## Build and flash the ESP32-S3

Load the ESP-IDF 6.0 environment, then run:

```sh
cd esp_controller
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Defaults:

- SSID: `DRONE_TEST`
- Password: `drone1234`
- Control page: `http://192.168.4.1`

Wi-Fi, UART pins, baud rate, and watchdog timeout can be changed under
**Drone controller** in `idf.py menuconfig`.

## Build and flash the STM32H7

Import the repository root as the STM32CubeIDE project, build the `Debug`
configuration, and flash it normally. USART1 on PA9/PA10 and TIM3 are already
configured.

For a command-line verification build with STM32CubeIDE 2.1:

```sh
/opt/st/stm32cubeide_2.1.0/headless-build.sh \
  -configuration /tmp/stm32cubeide-config \
  -data /tmp/stm32cubeide-workspace \
  -import /absolute/path/to/stm32cube \
  -cleanBuild stm32_quad_dr/Debug -no-indexer -printErrorMarkers
```

## Safe test sequence

1. Keep propellers removed and the ESC motor supply disconnected.
2. Flash and start both boards; connect the UART and common ground.
3. Join `DRONE_TEST`, open `http://192.168.4.1`, and confirm STM32 telemetry.
4. Press **Disarm / Reset**, then hold **Arm** for one second at zero throttle.
5. Hold the dead-man control while moving the slider. Releasing it commands
   zero immediately.
6. Confirm that closing the page or disconnecting Wi-Fi produces fail-safe
   within 300 ms and requires disarm before re-arming.

Protocol details and host tests are in
[`Components/DroneProtocol`](../Components/DroneProtocol/README.md).
