# Drone rate control

Phase 1 implements three independent body-rate PID controllers:

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

This phase deliberately does not apply the PID correction to the motors. All
four motors continue to receive the same collective throttle until the motor
layout, propeller directions and mixer signs are verified in phase 2.
