# Motor DShot300 / DShot600

Shared four-channel unidirectional DShot implementation of `MotorOutput`.
`MotorDshot300` provides a header-only adapter that requires 300000 bit/s;
DShot600 continues to use `MotorDshot_Init()` with 600000 bit/s.

## Encoding

```text
normalized throttle
  -> 0 or configured 48..2047 value
  -> [11-bit value][telemetry][4-bit checksum]
  -> 16 high-time compare values + 2 zero slots
  -> interleaved CCR1..CCR4 DMA buffer
  -> TIM update DMA burst
  -> GPIO
```

The checksum is XOR over the three payload nibbles. Bits are sent MSB first.
At the current 60 MHz TIM3 clock, timing is derived at runtime with integer
arithmetic:

| Mode | Ticks per bit | High ticks for bit 0 | High ticks for bit 1 |
|---|---:|---:|---:|
| DShot300 | 200 | 75 | 150 |
| DShot600 | 100 | 38 | 75 |

Each frame contains 16 data bits and two low slots: 60 us at DShot300 or
30 us at DShot600. The scheduling period remains nominally 1 ms for both.

One TIM update DMA burst writes four CCR values per bit period. All four motor
frames therefore share the same timer edge; four sequential channel-DMA starts
are deliberately not used.

`MotorDshot_Stop()` aborts an in-flight frame, forces every CCR low, and starts
an explicit value-zero frame. The two trailing zero slots leave the active CCR
low; the 1 kHz control loop provides a much longer inter-frame low interval.

DShot600 initialization outline (see `../MotorDshot300/README.md` for DShot300):

```c
MotorDshot dshot_driver;
MotorOutput motors;

MotorDshot_Config config = {
  .timer = &timer,                 /* 60 MHz counter, period 100 ticks */
  .channel = {&ch[0], &ch[1], &ch[2], &ch[3]},
  .bitrate_hz = MOTOR_DSHOT600_BITRATE_HZ,
  .minimum_throttle_value = 48U,
  .maximum_throttle_value = 2047U,
  .telemetry = false,
};
MotorDshot_Init(&dshot_driver, &motors, &config);
```

## Required CubeMX DMA configuration

The repository intentionally does not modify generated files or the `.ioc`.
The current generated source already configures TIM3_UP on DMA1 Stream0.
No peripheral change is needed to switch between DShot300 and DShot600.
For a fresh project or after regeneration, verify:

1. Add the `TIM3_UP` DMA request to an unused DMA1/DMA2 stream.
2. Select normal mode, peripheral increment disabled, memory increment enabled.
3. Select word width for both peripheral and memory.
4. Select high or very-high priority and enable that DMA stream IRQ.
5. Ensure CubeMX calls `MX_DMA_Init()` before `MX_TIM3_Init()`.
6. Keep TIM3 CH1..CH4 as PWM1, active high, on PA6/PA7/PB0/PB1.
7. Select the desired output (DShot600 example):

```c
#define APP_MOTOR_OUTPUT_PROTOCOL APP_MOTOR_OUTPUT_DSHOT600
```

The DMA buffer is statically allocated in normal RAM and aligned to a 32-byte
cache line. The driver cleans it before DMA whenever Cortex-M7 D-cache is
enabled. Do not place it in DTCM, which DMA1/DMA2 cannot access.
