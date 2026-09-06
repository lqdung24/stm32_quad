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
  motor_pwm.h
Src/
  motor_pwm.c
README.md
```

# Files

## File: Inc/motor_pwm.h
```c
} MotorPwm_Config_t;
⋮----
} MotorPwm_Handle_t;
⋮----
/*
 * The timer must count at exactly 1 MHz, so one compare count equals 1 us.
 * The application owns timer/PWM initialization and channel start. Attach only
 * binds an already-running timer and writes the disarmed compare values.
 */
bool MotorPwm_Attach(MotorPwm_Handle_t *motors,
⋮----
/* Arm only removes the software write lock; it does not raise motor pulses. */
bool MotorPwm_Arm(MotorPwm_Handle_t *motors);
⋮----
/* Immediately writes the disarmed pulse to all motors and restores the lock. */
void MotorPwm_Disarm(MotorPwm_Handle_t *motors);
⋮----
/* motor_index is zero-based: 0..3. Values outside configured limits fail. */
bool MotorPwm_SetPulseUs(MotorPwm_Handle_t *motors,
⋮----
bool MotorPwm_SetAllPulseUs(MotorPwm_Handle_t *motors,
⋮----
uint16_t MotorPwm_GetPulseUs(const MotorPwm_Handle_t *motors,
⋮----
bool MotorPwm_IsAttached(const MotorPwm_Handle_t *motors);
bool MotorPwm_IsArmed(const MotorPwm_Handle_t *motors);
⋮----
#endif /* MOTOR_PWM_H */
```

## File: Src/motor_pwm.c
```c
static bool MotorPwm_ConfigValid(const MotorPwm_Config_t *config);
static void MotorPwm_WriteCompare(const MotorPwm_Handle_t *motors,
⋮----
static void MotorPwm_WriteDisarmed(MotorPwm_Handle_t *motors);
⋮----
bool MotorPwm_Attach(MotorPwm_Handle_t *motors,
⋮----
bool MotorPwm_Arm(MotorPwm_Handle_t *motors)
⋮----
void MotorPwm_Disarm(MotorPwm_Handle_t *motors)
⋮----
bool MotorPwm_SetPulseUs(MotorPwm_Handle_t *motors,
⋮----
bool MotorPwm_SetAllPulseUs(MotorPwm_Handle_t *motors,
⋮----
uint16_t MotorPwm_GetPulseUs(const MotorPwm_Handle_t *motors,
⋮----
bool MotorPwm_IsAttached(const MotorPwm_Handle_t *motors)
⋮----
bool MotorPwm_IsArmed(const MotorPwm_Handle_t *motors)
⋮----
static bool MotorPwm_ConfigValid(const MotorPwm_Config_t *config)
⋮----
static void MotorPwm_WriteDisarmed(MotorPwm_Handle_t *motors)
```

## File: README.md
```markdown
# Motor PWM

Four-channel standard ESC PWM output.

## Hardware used by the application

Rotation direction is stated while viewing the drone from above.

| Motor | Position | Timer/GPIO | Rotation | First rotation | Configured idle |
|---|---|---|---|---:|---:|
| M1 | front-left | `TIM3_CH1/PA6` | CW | `1200 us` | `1220 us` |
| M2 | rear-left | `TIM3_CH2/PA7` | CCW | `1205 us` | `1225 us` |
| M3 | front-right | `TIM3_CH3/PB0` | CCW | `1190 us` | `1210 us` |
| M4 | rear-right | `TIM3_CH4/PB1` | CW | `1205 us` | `1225 us` |

The configured idle values are `20 us` above the no-prop first-rotation
measurements.

TIM3 runs from a 60 MHz timer clock. The prescaler is 59, giving a 1 MHz
counter, and ARR is 19999, giving standard 50 Hz ESC PWM.

## Pulse convention

- 1000 us: disarmed/minimum throttle
- 2000 us: maximum throttle

The application owns TIM/PWM initialization, channel configuration and channel
start. `MotorPwm_Attach()` only binds the running timer, writes the disarmed
compare values and provides the arm/disarm write gate.
```
