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
  mahony9.h
Src/
  mahony9.c
README.md
```

# Files

## File: Inc/mahony9.h
```c
/*
 * 9-axis Mahony attitude filter.
 *
 * Conventions:
 * - World: NED (X North, Y East, Z Down)
 * - Body:  FRD (X Forward, Y Right, Z Down)
 * - Quaternion: Hamilton, scalar first [w, x, y, z]
 * - q_nb rotates BODY vectors to NED
 * - Gyroscope input: BODY rad/s
 * - Accelerometer input: BODY gravity vector (not specific force)
 * - Magnetometer input: calibrated BODY magnetic vector
 */
⋮----
/* Magnetometer thresholds use the same unit supplied to Update/Init. */
⋮----
} Mahony9_Config_t;
⋮----
} Mahony9_Handle_t;
⋮----
} Mahony9_Euler_t;
⋮----
void Mahony9_Init(Mahony9_Handle_t *filter, const Mahony9_Config_t *config);
⋮----
/*
 * Initializes roll/pitch from gravity and yaw from tilt-compensated magnetic
 * North. A level board pointing to magnetic North initializes q_nb to identity.
 */
bool Mahony9_InitFromAccelMag(Mahony9_Handle_t *filter,
⋮----
/* Full 9-axis update. Invalid accel or mag vectors are rejected by norm gates. */
bool Mahony9_Update(Mahony9_Handle_t *filter,
⋮----
/* Gyro + accel fallback which preserves the same quaternion state. */
bool Mahony9_UpdateImu(Mahony9_Handle_t *filter,
⋮----
bool Mahony9_GetEulerDegrees(const Mahony9_Handle_t *filter,
⋮----
#endif /* MAHONY9_H */
```

## File: Src/mahony9.c
```c
static bool Mahony9_UpdateInternal(Mahony9_Handle_t *filter,
⋮----
static bool Mahony9_NormAccepted(float norm, float minimum, float maximum);
static float Mahony9_Clamp(float value, float minimum, float maximum);
⋮----
void Mahony9_Init(Mahony9_Handle_t *filter, const Mahony9_Config_t *config)
⋮----
bool Mahony9_InitFromAccelMag(Mahony9_Handle_t *filter,
⋮----
/* q_nb with yaw=0, used only to tilt-compensate the magnetic vector. */
⋮----
/* Positive NED yaw is clockwise; East therefore corresponds to +90 deg. */
⋮----
/* q_nb = q_z(yaw) * q_y(pitch) * q_x(roll). */
⋮----
bool Mahony9_Update(Mahony9_Handle_t *filter,
⋮----
bool Mahony9_UpdateImu(Mahony9_Handle_t *filter,
⋮----
bool Mahony9_GetEulerDegrees(const Mahony9_Handle_t *filter,
⋮----
/* Estimated NED-down direction expressed in BODY. */
⋮----
/* Rotate the measured field BODY -> NED. */
⋮----
/* Magnetic-North reference in NED: [horizontal magnitude, 0, down]. */
⋮----
/* Expected magnetic direction transformed NED -> BODY. */
⋮----
static bool Mahony9_NormAccepted(float norm, float minimum, float maximum)
⋮----
static float Mahony9_Clamp(float value, float minimum, float maximum)
```

## File: README.md
```markdown
# Mahony9

This component is the 9-axis attitude filter. It is intentionally separate
from `Components/Mahony`, which remains the original gyro + accelerometer
6-axis filter.

Inputs follow the project attitude convention:

- NED world and FRD body frames.
- Hamilton scalar-first `q_nb = [w, x, y, z]`.
- `q_nb` rotates BODY vectors into NED.
- Gyroscope is BODY rad/s.
- Accelerometer input is the BODY gravity direction. Convert accelerometer
  specific force with `Attitude_SpecificForceToGravity`.
- Magnetometer is mapped, hard-iron corrected, and soft-iron corrected BODY
  data. Its absolute unit is arbitrary, but the configured norm gates must use
  that same unit.

`Mahony9_InitFromAccelMag` initializes yaw zero at magnetic North. Magnetic
declination is not applied, so yaw references magnetic North rather than true
North.

`Mahony9_Update` uses gyro, accel, and mag. `Mahony9_UpdateImu` preserves the
same quaternion while temporarily falling back to gyro + accel when a magnetic
sample is unavailable or rejected.
```
