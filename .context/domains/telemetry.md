# Domain: status and flight telemetry

## Producers and consumers

- Producer/cache: `DroneControl_PublishFlightTelemetrySample` on STM32.
- Wire encoder: `DroneProtocol_EncodeFlightTelemetry`.
- Bridge: Air and Ground forward raw telemetry without interpretation beyond packet classification/counters.
- Browser consumes `SYSTEM_STATUS` for state, ACK, rates and PWM UI, and `FLIGHT_TELEMETRY` for live plots and browser-side recording.
- `tools/telemetry_plot.py` consumes 50-byte `FLIGHT_TELEMETRY` binary WebSocket frames, records CSV and plots them.

## Flight telemetry fields

- STM32 timestamp, sequence and session.
- Roll/pitch/yaw in centidegrees on wire.
- Body gyro and target rate in milliradians/second.
- PID mixer corrections in centi-units.
- Four motor pulses in microseconds.
- State, actuator-active and attitude-valid flags.

All fixed-point values are range-checked before cache/encode. PWM must remain in 1000..2000 µs. A sample with invalid attitude may still contain useful valid gyro/rate/motor data.

## Host receiver flow

The Python receiver validates exact packet size, CRC, header, flags, state and PWM before accepting a sample. Accepted samples update packet/gap/drop statistics under a lock, append to a bounded deque and are flushed to CSV. A background thread reconnects WebSocket after network failures; plotting reads snapshots so rendering does not hold the receiver lock.

The browser validates the same 50-byte packet received through Web Serial. Its optional telemetry panel keeps a 60-second plot history, displays selected-axis setpoint/gyro/error plus combined PID output, attitude or four PWM channels, and records up to 180,000 samples for CSV/TXT download. Recording remains in browser memory until the user downloads it; navigating away loses an undownloaded recording.

Current telemetry contains only the combined PID correction, not separate P, I and D terms. Its 50 Hz transport rate is useful for step-response and low-frequency trend analysis, but is not sufficient to characterize all noise/derivative behavior of the nominal ~1 kHz rate loop.

Run parser validation without hardware:

```sh
python3 tools/telemetry_plot.py --self-test
```

## Diagnostics versus control

USB CDC diagnostics on STM32 do not accept flight control. Telemetry and logs are best-effort and must never block the high-priority flight loop. UART raw-byte logging and timing/mixer logs are compile-time gated; keep their buffer ownership and USB-busy behavior intact.

When changing telemetry, follow the compatibility checklist in `domains/drone-protocol.md`.
