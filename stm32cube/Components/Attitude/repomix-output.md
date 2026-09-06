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

````
Inc/
  attitude.h
Src/
  attitude.c
README.md
````

# Files

## File: Inc/attitude.h

````c
/*
 * ATTITUDE CONVENTIONS
 *
 * World frame:
 * - NED: X North, Y East, Z Down
 *
 * Body frame:
 * - FRD: X Forward, Y Right, Z Down
 *
 * Handedness:
 * - Right-handed coordinate systems
 *
 * Quaternion:
 * - Hamilton convention
 * - Scalar first: [w, x, y, z]
 *
 * q_nb:
 * - Rotates vectors from BODY to NED
 *
 * Vector rotation:
 * - v_n = q_nb ⊗ v_b ⊗ conjugate(q_nb)
 *
 * Gyro:
 * - Expressed in BODY frame
 * - Units: rad/s
 *
 * Quaternion dynamics:
 * - q_dot = 0.5 * q_nb ⊗ [0, gyro_body]
 *
 * Euler:
 * - Intrinsic ZYX
 * - Positive roll: right wing down
 * - Positive pitch: nose up
 * - Positive yaw: clockwise viewed from above
 *
 * Level attitude facing North:
 * - q_nb = [1, 0, 0, 0]
 *
 * Installed IMU sensor frame -> BODY FRD:
 * Accelerometer and gyroscope:
 * - body_x = -sensor_x
 * - body_y =  sensor_y
 * - body_z = -sensor_z
 * - This is a 180-degree mounting rotation about +Y.
 *
 * Magnetometer:
 * - body_x = -sensor_x
 * - body_y = -sensor_y
 * - body_z =  sensor_z
 */
⋮----
} Attitude_VectorRaw_t;
⋮----
} Attitude_Vector3f_t;
⋮----
Attitude_VectorRaw_t Attitude_MapSensorRawToBody(Attitude_VectorRaw_t sensor_raw);
Attitude_VectorRaw_t Attitude_MapMagSensorRawToBody(Attitude_VectorRaw_t sensor_raw);
/* AK09916 calibration, input/output units: centi-microtesla (c_uT). */
Attitude_VectorRaw_t Attitude_CalibrateMagBodyCentiUt(Attitude_VectorRaw_t mag_body_raw);
/* Six-position diagonal calibration. Input/output units: g in BODY FRD. */
Attitude_Vector3f_t Attitude_CalibrateAccelBodyG(Attitude_Vector3f_t accel_body_g);
Attitude_Vector3f_t Attitude_AccelRawToBodyG(Attitude_VectorRaw_t sensor_raw,
⋮----
Attitude_Vector3f_t Attitude_GyroRawToBodyRadS(Attitude_VectorRaw_t sensor_raw,
⋮----
Attitude_Vector3f_t Attitude_SpecificForceToGravity(Attitude_Vector3f_t specific_force_body);
⋮----
#endif /* ATTITUDE_H */
````

## File: Src/attitude.c

````c
/*
 * Six-position accelerometer calibration in BODY FRD.
 *
 * Stable pose means in mg:
 *   X+: +1020.326, X-:  -979.584
 *   Y+:  +986.720, Y-: -1014.639
 *   Z+:  +993.538, Z-: -1026.697
 *
 * calibrated = (measured - bias) * scale
 */
⋮----
/* Fitted from the 1300-sample full-3D tumble log. */
⋮----
Attitude_VectorRaw_t Attitude_MapSensorRawToBody(Attitude_VectorRaw_t sensor_raw)
⋮----
Attitude_VectorRaw_t Attitude_MapMagSensorRawToBody(Attitude_VectorRaw_t sensor_raw)
⋮----
Attitude_VectorRaw_t Attitude_CalibrateMagBodyCentiUt(Attitude_VectorRaw_t mag_body_raw)
⋮----
Attitude_Vector3f_t Attitude_CalibrateAccelBodyG(Attitude_Vector3f_t accel_body_g)
⋮----
Attitude_Vector3f_t Attitude_AccelRawToBodyG(Attitude_VectorRaw_t sensor_raw,
⋮----
Attitude_Vector3f_t Attitude_GyroRawToBodyRadS(Attitude_VectorRaw_t sensor_raw,
⋮----
Attitude_Vector3f_t Attitude_SpecificForceToGravity(Attitude_Vector3f_t specific_force_body)
````

## File: README.md

````markdown
# Attitude conventions and sensor mapping

## Frames and rotations

- **World frame:** NED — X North, Y East, Z Down.
- **Body frame:** FRD — X Forward, Y Right, Z Down.
- **Handedness:** Right-handed coordinate systems.
- **Quaternion:** Hamilton convention, scalar first: `[w, x, y, z]`.
- **`q_nb`:** Rotates vectors from BODY to NED.
- **Vector rotation:** `v_n = q_nb ⊗ v_b ⊗ conjugate(q_nb)`.
- **Gyro:** Expressed in the BODY frame, in `rad/s`.
- **Quaternion dynamics:** `q_dot = 0.5 * q_nb ⊗ [0, gyro_body]`.
- **Euler angles:** Intrinsic ZYX.
  - Positive roll: right wing down.
  - Positive pitch: nose up.
  - Positive yaw: clockwise viewed from above.
- **Level attitude facing North:** `q_nb = [1, 0, 0, 0]`.

## Installed accelerometer and gyroscope mapping

The tested sensor-to-BODY mounting rotation is:

```text
body_x = -sensor_x
body_y =  sensor_y
body_z = -sensor_z
```

This is a 180-degree rotation about `+Y` and preserves a right-handed frame.
The same mapping is applied to accelerometer and gyroscope samples.

## Accelerometer calibration

`Attitude_AccelRawToBodyG()` applies the sensor-to-BODY mapping first, then the
six-position diagonal calibration:

```text
calibrated = (mapped - bias) * scale

bias_g = [ 0.020371, -0.013960, -0.016580 ]
scale  = [ 1.000045,  0.999321,  0.989984 ]
```

The fit uses 1,522 stable samples from the six BODY-axis directions. The
calibrated output is in `g` and is the value supplied to the attitude filter.

## Installed magnetometer mapping

Four level headings, North/East/South/West, establish the AK09916 mapping:

```text
body_x = -sensor_x
body_y = -sensor_y
body_z =  sensor_z
```

The Z sign preserves a right-handed BODY frame. Magnetometer hard-iron and
soft-iron calibration are applied separately from this axis mapping.

`Attitude_CalibrateMagBodyCentiUt()` applies the fitted hard-iron offset and
soft-iron matrix. Input and output are in `c_uT`; normalize the calibrated
vector before using it for heading estimation.

An accelerometer measures specific force. At rest and level in the FRD body
frame, its expected output is approximately `[0, 0, -1] g`. The Mahony filter
in this project consumes the opposite vector, the gravity direction, which is
approximately `[0, 0, +1]`.
````
