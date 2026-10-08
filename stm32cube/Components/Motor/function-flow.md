# Motor - function flow

- `MotorOutput_Init()` validates the complete vtable, driver, protocol, and
  motor count before binding them.
- `MotorOutput_Start/Stop()` delegate the single arm/disarm output gate.
- `MotorOutput_SetThrottle()` validates one normalized finite command.
- `MotorOutput_SetAllThrottle()` validates the full bank before delegating, so
  invalid input cannot partially update requested outputs.
- `MOTOR_OUTPUT_DSHOT_PERIOD_MS=2` defines the App/DroneControl command cadence
  (nominal 500 Hz). The drivers themselves do not schedule frames; immediate
  Start/Stop zero frames remain available for arming and safety.
- `MotorOutput_Update()` commits the staged bank through the selected driver.
- Query functions expose logical started state, applied normalized throttle,
  raw driver value, and protocol without leaking the concrete driver type into
  the control layer.
