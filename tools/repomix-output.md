This file is a merged representation of the entire codebase, combined into a single document by Repomix.
The content has been processed where content has been compressed (code blocks are separated by ⋮---- delimiter).

# File Summary

## Purpose
This file contains a packed representation of the entire repository's contents.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.

## File Format
The content is organized as follows:
1. This summary section
2. Repository information
3. Directory structure
4. Repository files (if enabled)
5. Multiple file entries, each consisting of:
  a. A header with the file path (## File: path/to/file)
  b. The full contents of the file in a code block

## Usage Guidelines
- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.

## Notes
- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Content has been compressed - code blocks are separated by ⋮---- delimiter
- Files are sorted by Git change count (files with more changes are at the bottom)

# Directory Structure
```
run_telemetry_plot.sh
telemetry_plot.py
```

# Files

## File: run_telemetry_plot.sh
```bash
#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${script_dir}/telemetry_plot.py" "$@"
```

## File: telemetry_plot.py
```python
#!/usr/bin/env python3
"""Receive STM32 flight telemetry, append CSV, and plot it in real time.

Install optional runtime dependencies once:
    python3 -m pip install websocket-client matplotlib

Examples:
    python3 tools/telemetry_plot.py
    python3 tools/telemetry_plot.py --url ws://192.168.4.1/telemetry --csv flight.csv
    python3 tools/telemetry_plot.py --no-plot --csv flight.csv
"""
⋮----
MAGIC = 0xA55A
VERSION = 1
PACKET_TYPE_FLIGHT_TELEMETRY = 0x07
PACKET_SIZE = 50
PAYLOAD_SIZE = 32
FLAG_STATE_MASK = 0x0007
FLAG_ACTUATORS_ACTIVE = 1 << 3
FLAG_ATTITUDE_VALID = 1 << 4
FLAG_ALLOWED_MASK = 0x001F
HEADER_FORMAT = "<HBBHHHBBI"
PAYLOAD_FORMAT = "<12h4H"
STATE_NAMES = {0: "BOOT", 1: "DISARMED", 2: "ARMED", 3: "FAILSAFE", 4: "ERROR"}
⋮----
def crc16_ccitt_false(data: bytes) -> int
⋮----
crc = 0xFFFF
⋮----
crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
⋮----
@dataclass(frozen=True)
class Telemetry
⋮----
host_time_s: float
timestamp_ms: int
sequence: int
session_id: int
roll_deg: float
pitch_deg: float
yaw_deg: float
gyro_x_rad_s: float
gyro_y_rad_s: float
gyro_z_rad_s: float
roll_setpoint_rad_s: float
pitch_setpoint_rad_s: float
yaw_setpoint_rad_s: float
pid_roll: float
pid_pitch: float
pid_yaw: float
motor1_us: int
motor2_us: int
motor3_us: int
motor4_us: int
state: int
actuators_active: bool
attitude_valid: bool
⋮----
@property
    def armed(self) -> bool
⋮----
def decode_packet(packet: bytes, host_time_s: Optional[float] = None) -> Telemetry
⋮----
"""Validate and decode one raw 50-byte STM32 telemetry packet."""
⋮----
state = flags & FLAG_STATE_MASK
⋮----
values = struct.unpack_from(PAYLOAD_FORMAT, packet, 16)
motors = values[12:16]
⋮----
timestamp = time.time() if host_time_s is None else host_time_s
⋮----
CSV_COLUMNS = tuple(Telemetry.__dataclass_fields__.keys()) + ("armed", "state_name")
⋮----
class Receiver
⋮----
def __init__(self, url: str, csv_path: Path, history_seconds: float) -> None
⋮----
def close(self) -> None
⋮----
def add_packet(self, packet: bytes) -> None
⋮----
now = time.time()
⋮----
sample = decode_packet(packet, now)
⋮----
delta = (sample.sequence - self.last_sequence) & 0xFFFF
⋮----
row = {name: getattr(sample, name) for name in Telemetry.__dataclass_fields__}
⋮----
def run(self) -> None
⋮----
ws = None
⋮----
ws = websocket.create_connection(self.url, timeout=2.0)
⋮----
frame = ws.recv()
⋮----
except Exception as exc:  # Network errors are expected during AP reconnects.
⋮----
def plot(receiver: Receiver, window_seconds: float) -> None
⋮----
def redraw(_frame: int) -> None
⋮----
samples = list(receiver.samples)
count = receiver.packet_count
invalid = receiver.invalid_count
dropped = receiver.sequence_drops
gap_ms = receiver.last_gap_ms
⋮----
t0 = samples[-1].host_time_s
filtered = [sample for sample in samples if t0 - sample.host_time_s <= window_seconds]
x = [sample.host_time_s - t0 for sample in filtered]
⋮----
latest = samples[-1]
gap_text = "n/a" if math.isnan(gap_ms) else f"{gap_ms:.1f} ms"
⋮----
animation = FuncAnimation(fig, redraw, interval=50, cache_frame_data=False)
_ = animation
⋮----
def self_test() -> None
⋮----
header = struct.pack(HEADER_FORMAT, MAGIC, VERSION, PACKET_TYPE_FLIGHT_TELEMETRY,
payload = struct.pack(PAYLOAD_FORMAT, -123, 456, 789, -1000, 2000, -3000,
raw = header + payload
packet = raw + struct.pack("<H", crc16_ccitt_false(raw))
sample = decode_packet(packet, 1.0)
⋮----
def main() -> int
⋮----
parser = argparse.ArgumentParser(description=__doc__)
⋮----
args = parser.parse_args()
⋮----
receiver = Receiver(args.url, args.csv, args.window)
thread = threading.Thread(target=receiver.run, name="telemetry-receiver", daemon=True)
```
