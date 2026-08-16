#!/usr/bin/env python3
"""Receive STM32 flight telemetry, append CSV, and plot it in real time.

Install optional runtime dependencies once:
    python3 -m pip install websocket-client matplotlib

Examples:
    python3 tools/telemetry_plot.py
    python3 tools/telemetry_plot.py --url ws://192.168.4.1/telemetry --csv flight.csv
    python3 tools/telemetry_plot.py --no-plot --csv flight.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional


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


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Telemetry:
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

    @property
    def armed(self) -> bool:
        return self.state == 2


def decode_packet(packet: bytes, host_time_s: Optional[float] = None) -> Telemetry:
    """Validate and decode one raw 50-byte STM32 telemetry packet."""
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"bad packet length {len(packet)}, expected {PACKET_SIZE}")
    if crc16_ccitt_false(packet[:-2]) != struct.unpack_from("<H", packet, 48)[0]:
        raise ValueError("CRC mismatch")

    magic, version, packet_type, sequence, session_id, flags, payload_len, reserved, timestamp_ms = (
        struct.unpack_from(HEADER_FORMAT, packet, 0)
    )
    if magic != MAGIC or version != VERSION or packet_type != PACKET_TYPE_FLIGHT_TELEMETRY:
        raise ValueError("not a v1 FLIGHT_TELEMETRY packet")
    if payload_len != PAYLOAD_SIZE or reserved != 0:
        raise ValueError("malformed telemetry header")
    if flags & ~FLAG_ALLOWED_MASK:
        raise ValueError("unknown telemetry flags")
    state = flags & FLAG_STATE_MASK
    if state not in STATE_NAMES:
        raise ValueError("invalid flight state")

    values = struct.unpack_from(PAYLOAD_FORMAT, packet, 16)
    motors = values[12:16]
    if any(pwm < 1000 or pwm > 2000 for pwm in motors):
        raise ValueError("motor PWM outside 1000..2000 us")
    timestamp = time.time() if host_time_s is None else host_time_s
    return Telemetry(
        host_time_s=timestamp,
        timestamp_ms=timestamp_ms,
        sequence=sequence,
        session_id=session_id,
        roll_deg=values[0] / 100.0,
        pitch_deg=values[1] / 100.0,
        yaw_deg=values[2] / 100.0,
        gyro_x_rad_s=values[3] / 1000.0,
        gyro_y_rad_s=values[4] / 1000.0,
        gyro_z_rad_s=values[5] / 1000.0,
        roll_setpoint_rad_s=values[6] / 1000.0,
        pitch_setpoint_rad_s=values[7] / 1000.0,
        yaw_setpoint_rad_s=values[8] / 1000.0,
        pid_roll=values[9] / 100.0,
        pid_pitch=values[10] / 100.0,
        pid_yaw=values[11] / 100.0,
        motor1_us=motors[0], motor2_us=motors[1], motor3_us=motors[2], motor4_us=motors[3],
        state=state,
        actuators_active=bool(flags & FLAG_ACTUATORS_ACTIVE),
        attitude_valid=bool(flags & FLAG_ATTITUDE_VALID),
    )


CSV_COLUMNS = tuple(Telemetry.__dataclass_fields__.keys()) + ("armed", "state_name")


class Receiver:
    def __init__(self, url: str, csv_path: Path, history_seconds: float) -> None:
        self.url = url
        self.samples: Deque[Telemetry] = deque(maxlen=max(100, int(history_seconds * 60)))
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.packet_count = 0
        self.invalid_count = 0
        self.sequence_drops = 0
        self.last_sequence: Optional[int] = None
        self.last_arrival_s: Optional[float] = None
        self.last_gap_ms = math.nan
        self.csv_file = csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_COLUMNS)
        self.csv_writer.writeheader()

    def close(self) -> None:
        self.stop.set()
        self.csv_file.close()

    def add_packet(self, packet: bytes) -> None:
        now = time.time()
        try:
            sample = decode_packet(packet, now)
        except ValueError:
            with self.lock:
                self.invalid_count += 1
            return
        with self.lock:
            if self.last_arrival_s is not None:
                self.last_gap_ms = (now - self.last_arrival_s) * 1000.0
            if self.last_sequence is not None:
                delta = (sample.sequence - self.last_sequence) & 0xFFFF
                if 1 < delta < 0x8000:
                    self.sequence_drops += delta - 1
            self.last_arrival_s = now
            self.last_sequence = sample.sequence
            self.packet_count += 1
            self.samples.append(sample)
            row = {name: getattr(sample, name) for name in Telemetry.__dataclass_fields__}
            row["armed"] = sample.armed
            row["state_name"] = STATE_NAMES[sample.state]
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def run(self) -> None:
        try:
            import websocket
        except ImportError as exc:
            raise RuntimeError("install websocket-client: python3 -m pip install websocket-client") from exc

        while not self.stop.is_set():
            ws = None
            try:
                ws = websocket.create_connection(self.url, timeout=2.0)
                print(f"connected: {self.url}", file=sys.stderr)
                while not self.stop.is_set():
                    frame = ws.recv()
                    if isinstance(frame, bytes):
                        self.add_packet(frame)
            except Exception as exc:  # Network errors are expected during AP reconnects.
                if not self.stop.is_set():
                    print(f"telemetry reconnecting after: {exc}", file=sys.stderr)
                    self.stop.wait(1.0)
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass


def plot(receiver: Receiver, window_seconds: float) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError as exc:
        raise RuntimeError("install matplotlib: python3 -m pip install matplotlib") from exc

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
    fig.canvas.manager.set_window_title("Drone flight telemetry")

    def redraw(_frame: int) -> None:
        with receiver.lock:
            samples = list(receiver.samples)
            count = receiver.packet_count
            invalid = receiver.invalid_count
            dropped = receiver.sequence_drops
            gap_ms = receiver.last_gap_ms
        for axis in axes:
            axis.clear()
            axis.grid(True, alpha=0.3)
        if not samples:
            axes[0].set_title("Waiting for telemetry…")
            return

        t0 = samples[-1].host_time_s
        filtered = [sample for sample in samples if t0 - sample.host_time_s <= window_seconds]
        x = [sample.host_time_s - t0 for sample in filtered]
        axes[0].plot(x, [s.roll_deg for s in filtered], label="roll")
        axes[0].plot(x, [s.pitch_deg for s in filtered], label="pitch")
        axes[0].plot(x, [s.yaw_deg for s in filtered], label="yaw")
        axes[0].set_ylabel("attitude (deg)")
        axes[0].legend(loc="upper left", ncol=3)

        axes[1].plot(x, [s.gyro_x_rad_s for s in filtered], label="gyro roll")
        axes[1].plot(x, [s.gyro_y_rad_s for s in filtered], label="gyro pitch")
        axes[1].plot(x, [s.gyro_z_rad_s for s in filtered], label="gyro yaw")
        axes[1].plot(x, [s.roll_setpoint_rad_s for s in filtered], "--", label="setpoint roll")
        axes[1].plot(x, [s.pitch_setpoint_rad_s for s in filtered], "--", label="setpoint pitch")
        axes[1].plot(x, [s.yaw_setpoint_rad_s for s in filtered], "--", label="setpoint yaw")
        axes[1].set_ylabel("rate (rad/s)")
        axes[1].legend(loc="upper left", ncol=3, fontsize="small")

        axes[2].plot(x, [s.pid_roll for s in filtered], label="pid roll")
        axes[2].plot(x, [s.pid_pitch for s in filtered], label="pid pitch")
        axes[2].plot(x, [s.pid_yaw for s in filtered], label="pid yaw")
        axes[2].set_ylabel("PID command")
        axes[2].legend(loc="upper left", ncol=3)

        axes[3].plot(x, [s.motor1_us for s in filtered], label="M1")
        axes[3].plot(x, [s.motor2_us for s in filtered], label="M2")
        axes[3].plot(x, [s.motor3_us for s in filtered], label="M3")
        axes[3].plot(x, [s.motor4_us for s in filtered], label="M4")
        axes[3].set_ylabel("PWM (us)")
        axes[3].set_xlabel("seconds from newest sample")
        axes[3].legend(loc="upper left", ncol=4)

        latest = samples[-1]
        gap_text = "n/a" if math.isnan(gap_ms) else f"{gap_ms:.1f} ms"
        fig.suptitle(
            f"{STATE_NAMES[latest.state]} armed={latest.armed} active={latest.actuators_active} "
            f"attitude_valid={latest.attitude_valid} | packets={count} drops={dropped} "
            f"invalid={invalid} gap={gap_text}"
        )
        fig.tight_layout()

    animation = FuncAnimation(fig, redraw, interval=50, cache_frame_data=False)
    _ = animation
    plt.show()


def self_test() -> None:
    header = struct.pack(HEADER_FORMAT, MAGIC, VERSION, PACKET_TYPE_FLIGHT_TELEMETRY,
                         7, 9, 2 | FLAG_ACTUATORS_ACTIVE | FLAG_ATTITUDE_VALID,
                         PAYLOAD_SIZE, 0, 1234)
    payload = struct.pack(PAYLOAD_FORMAT, -123, 456, 789, -1000, 2000, -3000,
                          100, -200, 300, 400, -500, 600, 1000, 1200, 1500, 2000)
    raw = header + payload
    packet = raw + struct.pack("<H", crc16_ccitt_false(raw))
    sample = decode_packet(packet, 1.0)
    assert sample.roll_deg == -1.23 and sample.gyro_z_rad_s == -3.0
    assert sample.motor4_us == 2000 and sample.armed and sample.attitude_valid
    print("telemetry parser self-test: PASS")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://192.168.4.1/telemetry")
    parser.add_argument("--csv", type=Path, default=Path("flight_telemetry.csv"))
    parser.add_argument("--window", type=float, default=10.0, help="plot history window in seconds")
    parser.add_argument("--no-plot", action="store_true", help="record CSV only")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.window <= 0:
        parser.error("--window must be positive")

    receiver = Receiver(args.url, args.csv, args.window)
    thread = threading.Thread(target=receiver.run, name="telemetry-receiver", daemon=True)
    thread.start()
    try:
        if args.no_plot:
            while thread.is_alive():
                thread.join(timeout=0.5)
        else:
            plot(receiver, args.window)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
        thread.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
