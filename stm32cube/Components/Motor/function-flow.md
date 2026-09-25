# Motor - function flow

- `MotorOutput_Init()` validates the complete vtable, driver, protocol, and
  motor count before binding them.
- `MotorOutput_Start/Stop()` delegate the single arm/disarm output gate.
- `MotorOutput_SetThrottle()` validates one normalized finite command.
- `MotorOutput_SetAllThrottle()` validates the full bank before delegating, so
  invalid input cannot partially update requested outputs.
- `MotorOutput_Update()` commits the staged bank through the selected driver.
- Query functions expose logical started state, applied normalized throttle,
  raw driver value, and protocol without leaking the concrete driver type into
  the control layer.
