# ESP32-S3 drone control bridge

This ESP-IDF 6.0 project creates a Wi-Fi hotspot and serves a mobile web page.
The page sends binary control packets over WebSocket. The ESP32-S3 validates
and forwards them over UART to the STM32H7, which is the only controller that
can drive an ESC.

The web page has two control modes selected by one button. **Joystick** is a
landscape, Mode-2 style controller: the left stick commands throttle/yaw and
the right stick commands pitch/roll. The left stick retains throttle when
released while yaw returns to center; the right stick returns both axes to
center. Touch the current left-stick knob before dragging so throttle cannot
jump from an accidental touch. Changing web modes always disarms and clears
all four commands.

The page also retains the no-prop motor selector used during calibration, but
the current STM32 build has raw threshold-test mode disabled. Keep **All
motors** selected for normal control; the rate PID and Quad-X mixer now drive
all four outputs. One shared ARM action gates the outputs, and changing
selection while throttle is active is rejected by the STM32.

The page is responsive in portrait and landscape and does not request an
orientation lock. The browser requires periodic `ESP_ALIVE` acknowledgements;
loss of the ESP WebSocket/heartbeat, loss of STM32 status, hiding the page, or
closing it clears the local ARM state. Independent ESP-to-STM and
phone-to-ESP watchdogs remain the authoritative stop paths.

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

Motor rotation below is viewed from above. The idle floor includes `20 us`
above the measured no-prop first-rotation pulse.

| Motor | Position | STM32 output | Rotation | First rotation | Idle floor |
|---|---|---|---|---:|---:|
| M1 | front-left | `TIM3_CH1/PA6` | CW | `1200 us` | `1220 us` |
| M2 | rear-left | `TIM3_CH2/PA7` | CCW | `1205 us` | `1225 us` |
| M3 | front-right | `TIM3_CH3/PB0` | CCW | `1190 us` | `1210 us` |
| M4 | rear-right | `TIM3_CH4/PB1` | CW | `1205 us` | `1225 us` |

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
4. Press **Mở joystick** and rotate the phone to landscape. Switching mode
   disarms the controller and initializes throttle at zero.
5. Press **Disarm**, then hold **Arm** once for one second with both sticks at
   neutral and throttle at zero. The single ARM action covers all four outputs.
6. Drag the left stick upward from its bottom position for throttle and
   sideways for yaw. Use the right stick for pitch and roll. Test all signs on
   a restrained rig before installing propellers.
7. Confirm that closing the page or disconnecting Wi-Fi produces fail-safe
   within 300 ms and requires disarm before re-arming.

Protocol details and host tests are in
[`Components/DroneProtocol`](../Components/DroneProtocol/README.md).
