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
