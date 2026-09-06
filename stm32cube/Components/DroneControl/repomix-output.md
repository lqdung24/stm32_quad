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
```
Inc/
  drone_control.h
Src/
  drone_control.c
README.md
```

# Files

## File: Inc/drone_control.h
```c
/* Pilot input 1..500 maps to 1225..1800 us before PID/mixer corrections. */
⋮----
} DroneMixerTelemetry;
⋮----
void DroneControl_Init(UART_HandleTypeDef *uart, MotorPwm_Handle_t *motors);
void DroneControl_Process(uint32_t now_ms);
⋮----
/*
 * Supply each new calibrated gyroscope sample in BODY FRD, rad/s. A valid
 * armed update applies the rate-PID correction through the Quad-X mixer.
 */
bool DroneControl_UpdateBodyRates(float roll_rad_s,
⋮----
bool DroneControl_GetRateControlDebug(RateControlDebug *debug);
bool DroneControl_GetMixerTelemetry(DroneMixerTelemetry *telemetry);
⋮----
/*
 * Publish a fresh IMU/control-loop sample for the best-effort telemetry
 * stream. Attitude is in degrees; gyro is BODY FRD rad/s. This function never
 * changes flight state or actuator output.
 */
void DroneControl_PublishFlightTelemetrySample(uint32_t timestamp_ms,
⋮----
/* Call from the corresponding STM32 HAL callbacks. */
void DroneControl_OnUartRxEvent(UART_HandleTypeDef *uart, uint16_t size);
void DroneControl_OnUartError(UART_HandleTypeDef *uart);
⋮----
/*
 * Read a best-effort copy of bytes received from the control UART.
 * This trace buffer is separate from the safety-critical protocol RX buffer.
 */
size_t DroneControl_ReadUartRxLog(uint8_t *output, size_t capacity);
```

## File: Src/drone_control.c
```c
/*
 * No-prop first-rotation measurements, expressed above the 1000 us disarmed
 * pulse. Rotation is viewed from above: M1/M4 CW and M2/M3 CCW. The active
 * floor adds 20 us so each motor stays reliably above its measured threshold.
 */
⋮----
} DroneControlContext;
⋮----
static void start_uart_receive(void);
static void process_uart_bytes(uint32_t now_ms);
static void process_raw_packet(const uint8_t *packet, size_t length,
⋮----
static void process_control_command(const DroneControlCommand *command,
⋮----
static void enter_failsafe(uint16_t reason);
static void disarm_output(void);
static bool apply_throttle(uint16_t requested);
static float throttle_to_pwm(uint16_t throttle);
static bool apply_mixed_output(float roll_correction,
⋮----
static void publish_mixer_telemetry(const MotorMixerResult *mix,
⋮----
static void publish_disarmed_telemetry(void);
static bool send_status(uint32_t now_ms);
static bool send_flight_telemetry(void);
static bool try_send_packet(const uint8_t *raw, size_t raw_length);
static bool scale_to_i16(float value, float scale, int16_t *output);
⋮----
/*
 * Initial bench gains. Output is a normalized mixer correction, not us.
 * These values must be tuned for the actual airframe before flight.
 */
⋮----
void DroneControl_Init(UART_HandleTypeDef *uart, MotorPwm_Handle_t *motors)
⋮----
bool DroneControl_UpdateBodyRates(float roll_rad_s,
⋮----
bool DroneControl_GetRateControlDebug(RateControlDebug *debug)
⋮----
bool DroneControl_GetMixerTelemetry(DroneMixerTelemetry *telemetry)
⋮----
void DroneControl_PublishFlightTelemetrySample(uint32_t timestamp_ms,
⋮----
void DroneControl_Process(uint32_t now_ms)
⋮----
void DroneControl_OnUartRxEvent(UART_HandleTypeDef *uart, uint16_t size)
⋮----
void DroneControl_OnUartError(UART_HandleTypeDef *uart)
⋮----
size_t DroneControl_ReadUartRxLog(uint8_t *output, size_t capacity)
⋮----
static void start_uart_receive(void)
⋮----
static void process_uart_bytes(uint32_t now_ms)
⋮----
/*
     * A zero-throttle command with ARM_REQUEST clear is the explicit disarm
     * cycle required after startup, session changes, e-stop, or failsafe.
     */
⋮----
static void enter_failsafe(uint16_t reason)
⋮----
static void disarm_output(void)
⋮----
static bool apply_throttle(uint16_t requested)
⋮----
/*
     * A control packet may arrive between gyro samples. Retain the most recent
     * PID correction when applying the new collective so packet handling does
     * not briefly overwrite the stabilized output with equal motor commands.
     * RateControl_Reset() clears these values on disarm/failsafe/invalid input.
     */
⋮----
static float throttle_to_pwm(uint16_t throttle)
⋮----
static void publish_mixer_telemetry(
⋮----
static void publish_disarmed_telemetry(void)
⋮----
static bool send_status(uint32_t now_ms)
⋮----
static bool send_flight_telemetry(void)
⋮----
static bool try_send_packet(const uint8_t *raw, size_t raw_length)
⋮----
static bool scale_to_i16(float value, float scale, int16_t *output)
```

## File: README.md
```markdown
# Drone rate control

The controller implements three independent body-rate PID controllers:

- roll and pitch command range: `-200 .. +200 deg/s`
- yaw command range: `-150 .. +150 deg/s`
- input measurements: calibrated BODY FRD gyroscope rates in `rad/s`
- output: normalized mixer correction, not PWM microseconds

The PID uses derivative-on-measurement, a first-order derivative low-pass
filter, conditional integration, integral limiting and output limiting.
Controller state is reset whenever the vehicle is disarmed, enters failsafe,
has zero collective throttle, or receives an invalid sample interval.

The gains in `drone_control.c` are conservative initial bench values. They are
not airframe-independent and must be tuned on the actual vehicle.

## Quad-X mixer

The verified motor allocation and rotation direction, viewed from above the
drone, is:

| Motor | Position | Output | Rotation | First rotation | Configured idle | Mixer signs |
|---|---|---|---|---:|---:|---|
| M1 | front-left | `TIM3_CH1/PA6` | CW | `1200 us` | `1220 us` | `+roll +pitch -yaw` |
| M2 | rear-left | `TIM3_CH2/PA7` | CCW | `1205 us` | `1225 us` | `+roll -pitch +yaw` |
| M3 | front-right | `TIM3_CH3/PB0` | CCW | `1190 us` | `1210 us` | `-roll +pitch +yaw` |
| M4 | rear-right | `TIM3_CH4/PB1` | CW | `1205 us` | `1225 us` | `-roll -pitch -yaw` |

Each configured idle value includes a `20 us` margin above the measured
first-rotation threshold. CW/CCW always refers to the view from above.

Mixer commands use a logical `0..1000` scale. Common collective shifting
preserves roll/pitch/yaw authority at the actuator limits. Corrections are
scaled together only if their span exceeds the complete actuator range.

Disarm and armed-zero-throttle remain `1000 us`. The pilot throttle command is
mapped before mixing: command 1 is `1225 us`, command 100 is about `1339 us`,
command 250 is about `1512 us`, command 400 is about `1685 us`, and command
500 is `1800 us`. This removes the former `1..225` flat region at the motor
idle floor. PID corrections remain logical mixer commands and may drive an
individual motor above the pilot collective, up to the `2000 us` actuator
limit. Raw threshold-test mode is disabled, so body-rate PID corrections are
applied through the Quad-X mixer.
```
