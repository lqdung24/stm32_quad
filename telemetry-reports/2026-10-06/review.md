# Flight telemetry review — 2026-10-06

Source: `drone-telemetry-2026-10-06T13-54-52-530Z.csv`.
User confirmed free flight, without holding the drone. Gains assumed to match current firmware source; gains are not recorded in the CSV.

## Findings and limits

1. **P1 — Growing rate excursions and pitch PID clipping during free flight.** Roll gyro ranges -1.602..1.168 rad/s, pitch -1.451..1.455 rad/s. Tracking-error RMS is 0.3606 rad/s (roll) and 0.4135 rad/s (pitch). Roll correction ranges -120.91..164.25 mixer units; pitch -200..138.50, touching its -200 clamp once. This deserves investigation before calling the current tune flight-ready.
2. **P1 — Flight ends in FAILSAFE.** Transition occurs 8.566 s after the first recorded sample; all motor commands become zero (legacy output representation 1000). The log lacks SYSTEM_STATUS error flags/control commands, so it cannot distinguish pilot e-stop, link failure, unsafe transition or another failsafe cause. All transmitted attitude-valid flags are true; this alone cannot identify the failsafe cause.
3. **P2 — Derivative contributes materially to correction spikes.** At t=7.945 s, pitch error is -1.875 rad/s, giving P=-84.375 units with Kp=45, while total correction is clipped to -200. Since the I term is bounded at +/-60, D must contribute at least approximately -55.6 units at this event (or more negative before clipping). Across the active segment, |PID-P| exceeds the I bound at 21 roll and 20 pitch samples. This demonstrates a derivative contribution; it does not establish that D is excessive or that noise is the root cause.
4. **P2 — Data are insufficient for an optimized gain fit.** Setpoints agree with the current Angle zero-stick quaternion mapping within 0.00083 rad/s. There is no clear commanded rate step for rise-time/overshoot identification. Telemetry is 50 Hz while PID runs on fresh IMU samples, so high-frequency vibration/derivative activity is aliased and the internal D filter cannot be reconstructed faithfully.

## Verified data consistency

- 595 rows, 11.886 s, one session; no sequence gaps and no duplicate sample IDs.
- 254 active, attitude-valid, output-sample-matched rows from t=3.485..8.544 s.
- Commit delay: 242 samples at 1 ms, 12 at 0 ms.
- Logged rate errors equal setpoint minus gyro.
- Inverting the current mixer from motor output agrees with logged PID within 0.45 mixer units, consistent with motor-output quantization.
- Active motor outputs range 1177..1916 in the legacy 1000..2000 representation; no transmitted sample hits an actuator endpoint. Motor-bank span reaches 595 units. This does not rule out clipping between transmitted samples or thrust limitations.
- Failsafe samples report inactive output and zero corrections.

## Proposed first experiment (not an optimized or flight-validated tune)

Change only roll/pitch Kd from 0.8 to **0.6** (-25%). Keep Kp=45, Ki=20, integral limit=60, output limit=200 and D cutoff=20 Hz. Keep yaw at 35/10/0 and outer Angle Kp=4/s. The numerical 25% step is an experimental choice, not a fitted estimate or a copied PX4 gain.

A single-parameter change makes before/after comparisons interpretable. Reducing D may lower noisy motor corrections, but can also worsen damping/overshoot. Use a controlled test arrangement, and do not accept lower PID RMS alone as improvement: tracking-error RMS/peaks and angle recovery must not worsen, and PID clipping/oscillations should decrease. Stop the comparison if growing oscillations or loss of control appear.

Before further gain increases: identify the recorded failsafe cause, inspect propeller/motor/IMU mounting vibration, and acquire bounded roll/pitch rate-step responses with throttle reasonably constant. Record P, I, filtered D and the real PID dt at a higher local logging rate (e.g. 250–500 Hz) rather than inferring the 1 kHz derivative from 50 Hz telemetry. Flight-mode metadata, requested/applied throttle, mixer saturation flags and SYSTEM_STATUS error flags would remove important ambiguities.

Tune rate P/D before I and then outer Angle. PX4's manual guide describes this sequence and the tradeoff between derivative noise amplification and damping: https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter . Its numerical gains are not directly transferable to this firmware's 0..1000 mixer scale.

## Readiness and unresolved facts

Analysis complete; no gains or firmware files changed. The candidate is ready for a controlled comparison, not established as flight-ready. This CSV cannot verify physical motor order/spin, sensor mounting, exact loop dt, accelerometer vibration, gain values actually flashed, or failsafe cause.

Plot: `telemetry_review.png` (same directory).

Source SHA-256: `76afce6de9c3f9eba7754c445919be1fa0696ac617c114a83f65bac54e8e7208`
