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
