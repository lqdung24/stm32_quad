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
  motor_mixer.h
Src/
  motor_mixer.c
```

# Files

## File: Inc/motor_mixer.h
```c
} MotorMixerMotor;
⋮----
} MotorMixerOutputConfig;
⋮----
} MotorMixerResult;
⋮----
/*
 * Quad-X allocation in BODY FRD, viewed from above. Positive BODY yaw is
 * clockwise when viewed from above:
 *
 *   M1 front-left  CW   (+roll, +pitch, -yaw)
 *   M2 rear-left   CCW  (+roll, -pitch, +yaw)
 *   M3 front-right CCW  (-roll, +pitch, +yaw)
 *   M4 rear-right  CW   (-roll, -pitch, -yaw)
 *
 * Inputs and result.command[] use the same logical 0..1000 command scale.
 * The mixer shifts collective equally before scaling corrections, preserving
 * requested torque whenever actuator range permits it.
 */
bool MotorMixer_MixQuadX(float collective,
⋮----
/*
 * Converts logical 0..1000 motor commands to ESC pulses. Active outputs are
 * floored at their individual calibrated idle pulse; inactive outputs use the
 * common disarmed pulse.
 */
bool MotorMixer_MapToPulseUs(
⋮----
#endif /* MOTOR_MIXER_H */
```

## File: Src/motor_mixer.c
```c
bool MotorMixer_MixQuadX(float collective,
⋮----
bool MotorMixer_MapToPulseUs(
```
