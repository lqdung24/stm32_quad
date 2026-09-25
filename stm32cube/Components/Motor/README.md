# Motor output interface

`MotorOutput` is the protocol-independent four-motor actuator boundary used by
`DroneControl`. Its command is normalized throttle, not measured RPM:

- `0.0f`: stopped output
- `1.0f`: maximum configured output

`MotorOutput_SetThrottle()` and `MotorOutput_SetAllThrottle()` only stage a
request. `MotorOutput_Update()` commits the complete staged bank. The bank API
preserves the existing all-four validation invariant and gives a shared-timer
DShot backend one synchronized commit point.

The implementation is static: a `MotorOutput` stores a vtable and a caller
owned driver pointer; it performs no allocation.
