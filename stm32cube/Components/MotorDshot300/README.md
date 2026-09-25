# Motor DShot300

DShot300 adapter for the shared `MotorDshot` driver and `MotorOutput` API.
The adapter is header-only, so the existing CubeIDE source list remains valid.
Include `Inc/motor_dshot300.h` and continue compiling `MotorDshot/Src/motor_dshot.c`.

```c
MotorDshot300 driver;
MotorOutput motors;
MotorDshot300_Config config = {
  .timer = &timer, /* 60 MHz counter, period 200 ticks */
  .channel = {&ch[0], &ch[1], &ch[2], &ch[3]},
  .bitrate_hz = MOTOR_DSHOT300_BITRATE_HZ,
  .minimum_throttle_value = MOTOR_DSHOT_MIN_THROTTLE_VALUE,
  .maximum_throttle_value = MOTOR_DSHOT_MAX_THROTTLE_VALUE,
  .telemetry = false,
};
MotorDshot300_Init(&driver, &motors, &config);
```

`MotorDshot300_Init()` rejects any bitrate other than 300000 bit/s, then uses
shared validation, encoding, synchronized four-channel TIM update DMA, stop,
and error handling. Timer period must equal counter clock / 300000 exactly.
At 60 MHz this means ARR=199, bit-0 high=75 ticks and bit-1 high=150 ticks.
The 16-bit packet and two low slots take 60 us; App still schedules updates
at nominal 1 ms intervals.

App defaults to this component with `APP_MOTOR_OUTPUT_DSHOT300` and
`APP_DSHOT_TEST_ENABLE=1`: initialize, send value 0 for 15000 ms, then send
0.30 normalized throttle (raw value 648) to all four motors. The same sequencer
runs DShot600 when selected. DMA BUSY defers to the next tick; output errors
latch the test off and request Stop. Remove propellers: this bench bypasses
ARM/DISARM, e-stop and radio watchdog; power off ESCs to stop it.

For normal command/PID/mixer control, set `APP_DSHOT_TEST_ENABLE=0` and explicitly
select `APP_MOTOR_OUTPUT_DSHOT300`. ESC support and signal timing still require
bench verification. See `../MotorDshot/README.md` for the shared DMA requirements.
