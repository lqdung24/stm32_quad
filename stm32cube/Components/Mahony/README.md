# Mahony 6-axis attitude estimator

The flight pipeline uses this accelerometer/gyroscope filter for Angle mode and
attitude telemetry. Magnetometer initialization or measurements are not needed.
The project-wide frames, quaternion conventions, and installed sensor mapping
are defined in [`../Attitude/README.md`](../Attitude/README.md) and
`../Attitude/Inc/attitude.h`.

Inputs are calibrated BODY FRD gyro in rad/s and gravity in g. App negates
accelerometer specific force before passing gravity to the filter. Euler output
is in degrees: positive roll means right wing down and positive pitch means nose
up. Startup yaw is zero and subsequently drifts; this is not a heading reference.

Initialization requires a finite valid configuration and gravity within the
configured norm gate (App: 0.8 < norm < 1.2 g). Once initialized, measurements
outside that gate skip accelerometer correction and propagate gyro only.
Invalid state, nonfinite input, or sample intervals outside 0.5..20 ms invalidate
the filter. A later valid gravity sample must initialize it again. App sends the
invalid sample to flight control before any such recovery, so Angle mode cannot
continue on a stale estimate.

Run host regressions with `make -C stm32cube/Tests/mahony test`. The binary is
written to `/tmp/drone_test_mahony` by default; `TARGET` can override its location.
Host checks do not establish sensor mounting, gain suitability, or flight readiness.
