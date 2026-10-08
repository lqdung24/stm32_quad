# Domain: status and flight telemetry

## Producers and consumers

- Producer/cache: `DroneControl_PublishFlightTelemetrySample` on STM32.
- Wire encoder: `DroneProtocol_EncodeFlightTelemetry`.
- Bridge: Air and Ground forward raw telemetry without interpretation beyond packet classification/counters.
- Browser consumes `SYSTEM_STATUS` for state, ACK, rates and PWM UI, and `FLIGHT_TELEMETRY` for live plots and browser-side recording.
- `tools/telemetry_plot.py` consumes synchronized 58-byte type-8 and legacy 50-byte type-7 telemetry binary WebSocket frames, records CSV and plots them.

## Flight telemetry fields

- STM32 IMU sample timestamp (header), sequence and session.
- Type 8 appends uint32 `sample_id` at offset 48 and `motor_commit_time_ms` at offset 52; CRC is at 56. Header bit 5 (`output_sample_matched`) distinguishes a committed flight sample from independent test/zero/disarmed output. Legacy type 7 has no association metadata.
- Roll/pitch/yaw in centidegrees on wire.
- Body gyro and target rate in milliradians/second.
- PID mixer corrections in centi-units.
- Four motor pulses in microseconds.
- State, actuator-active and attitude-valid flags.

All fixed-point values are range-checked before cache/encode. PWM must remain in 1000..2000 µs. A sample with invalid attitude may still contain useful valid gyro/rate/motor data.

Each UpdateFlightSample increments a uint32 sample ID. PublishFlightTelemetrySample captures attitude/gyro/setpoint/PID from that same sample. Active DShot flight waits for successful driver submission to attach the matching motor bank and commit time; BUSY keeps the previous completed snapshot. New samples replace uncommitted samples. PWM completes after its immediate successful commit. Commit time means driver submission, not DMA completion or measured RPM. Test/zero/disarmed snapshots keep motor timing but do not claim an IMU-to-output match.

TX selects the latest completed snapshot at 50 Hz, sends each snapshot at most once, and drops snapshots older than 50 ms or from a different state/session. Stop, zero throttle and IMU invalidation clear pending/completed flight snapshots. Sample-ID gaps are expected because IMU/ESC rates exceed TX rate; packet-sequence gaps retain their transport-drop meaning.

## Host receiver flow

The Python receiver validates exact type/size pairs, CRC, header, flags, state and PWM before accepting a sample. Accepted samples update packet/gap/drop statistics under a lock, append to a bounded deque and are flushed to CSV. A background thread reconnects WebSocket after network failures; plotting reads snapshots so rendering does not hold the receiver lock.

The browser validates type-8 58-byte and legacy type-7 50-byte packets received through Web Serial. Its optional telemetry panel keeps a 60-second plot history, displays selected-axis setpoint/gyro/error plus combined PID output, attitude or four PWM channels, and records up to 180,000 samples for CSV/TXT download. CSV/TXT includes sample time/ID, commit time, match flag and wrap-safe commit delay (blank when unmatched or legacy); legacy IDs/times are blank. Recording remains in browser memory until the user downloads it; navigating away loses an undownloaded recording.

Current telemetry contains only the combined PID correction, not separate P, I and D terms. Its 50 Hz transport rate is useful for step-response and low-frequency trend analysis, but is not sufficient to characterize all noise/derivative behavior of the nominal ~1 kHz rate loop.

Run parser validation without hardware:

```sh
python3 tools/telemetry_plot.py --self-test
```

## Diagnostics versus control

USB CDC diagnostics on STM32 do not accept flight control. Telemetry and logs are best-effort and must never block the high-priority flight loop. UART raw-byte logging and timing/mixer logs are compile-time gated; keep their buffer ownership and USB-busy behavior intact.

When changing telemetry, follow the compatibility checklist in `domains/drone-protocol.md`.
