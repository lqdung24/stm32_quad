#!/usr/bin/env python3
"""Display STM32 ICM20948 roll, pitch and yaw as a 3D cube."""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
import tkinter as tk
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing pyserial. Install it with: python3 -m pip install pyserial")
    raise SystemExit(1)


RPY_PATTERN = re.compile(
    r"rpy\[cdeg\]=\s*(?P<roll>-?\d+),\s*(?P<pitch>-?\d+),\s*(?P<yaw>-?\d+)"
)
TIME_PATTERN = re.compile(r"t=\s*(?P<time>\d+)ms")


@dataclass
class ImuSample:
    time_ms: int
    roll: float
    pitch: float
    yaw: float


def parse_imu_line(line: str) -> ImuSample | None:
    rpy_match = RPY_PATTERN.search(line)
    if rpy_match is None:
        return None

    time_match = TIME_PATTERN.search(line)
    return ImuSample(
        time_ms=int(time_match.group("time")) if time_match else 0,
        roll=int(rpy_match.group("roll")) / 100.0,
        pitch=int(rpy_match.group("pitch")) / 100.0,
        yaw=int(rpy_match.group("yaw")) / 100.0,
    )


def choose_port(requested_port: str | None) -> str:
    if requested_port:
        return requested_port

    ports = list(list_ports.comports())
    preferred = [
        port.device
        for port in ports
        if "ttyACM" in port.device
        or "ttyUSB" in port.device
        or port.device.upper().startswith("COM")
    ]
    candidates = preferred or [port.device for port in ports]

    if len(candidates) == 1:
        return candidates[0]

    if not candidates:
        raise RuntimeError("No serial port found. Connect the board or pass --port.")

    available = ", ".join(candidates)
    raise RuntimeError(f"Multiple serial ports found: {available}. Select one with --port.")


def lerp_angle(current: float, target: float, amount: float) -> float:
    difference = (target - current + 180.0) % 360.0 - 180.0
    return current + difference * amount


class ImuCubeApp:
    BG = "#101820"
    GRID = "#263541"
    TEXT = "#e8e1cf"
    MUTED = "#91a2ad"
    FACE_COLORS = ("#d95d39", "#e9a23b", "#2a9d8f", "#3d7ea6", "#c6c24a", "#bc6c8e")

    VERTICES = (
        (-1.4, -0.9, -0.35),
        (1.4, -0.9, -0.35),
        (1.4, 0.9, -0.35),
        (-1.4, 0.9, -0.35),
        (-1.4, -0.9, 0.35),
        (1.4, -0.9, 0.35),
        (1.4, 0.9, 0.35),
        (-1.4, 0.9, 0.35),
    )
    FACES = (
        (0, 1, 2, 3),
        (4, 7, 6, 5),
        (0, 4, 5, 1),
        (3, 2, 6, 7),
        (0, 3, 7, 4),
        (1, 5, 6, 2),
    )

    def __init__(self, root: tk.Tk, serial_port: serial.Serial | None, demo: bool) -> None:
        self.root = root
        self.serial_port = serial_port
        self.demo = demo
        self.rx_buffer = bytearray()
        self.target = [0.0, 0.0, 0.0]
        self.display = [0.0, 0.0, 0.0]
        self.have_sample = False
        self.last_sample_wall_time = 0.0
        self.sensor_time_ms = 0
        self.lines_received = 0
        self.start_time = time.monotonic()

        root.title("ICM20948 Mahony Attitude Viewer")
        root.geometry("1000x720")
        root.minsize(720, 520)
        root.configure(bg=self.BG)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.canvas = tk.Canvas(root, bg=self.BG, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())

        self.root.after(10, self.poll_serial)
        self.root.after(16, self.animate)

    def close(self) -> None:
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()

    def poll_serial(self) -> None:
        if self.demo:
            elapsed = time.monotonic() - self.start_time
            self.target = [25.0 * math.sin(elapsed), 35.0 * math.sin(elapsed * 0.63), elapsed * 18.0]
            self.sensor_time_ms = int(elapsed * 1000.0)
            self.have_sample = True
            self.last_sample_wall_time = time.monotonic()
        elif self.serial_port is not None:
            try:
                waiting = self.serial_port.in_waiting
                if waiting:
                    self.rx_buffer.extend(self.serial_port.read(waiting))
                    self.consume_lines()
            except serial.SerialException as error:
                self.serial_port = None
                print(f"Serial disconnected: {error}", file=sys.stderr)

        self.root.after(10, self.poll_serial)

    def consume_lines(self) -> None:
        while b"\n" in self.rx_buffer:
            raw_line, _, remainder = self.rx_buffer.partition(b"\n")
            self.rx_buffer = bytearray(remainder)
            line = raw_line.decode("ascii", errors="ignore").strip()
            sample = parse_imu_line(line)
            if sample is None:
                continue

            self.target = [sample.roll, sample.pitch, sample.yaw]
            if not self.have_sample:
                self.display = self.target.copy()
            self.have_sample = True
            self.sensor_time_ms = sample.time_ms
            self.last_sample_wall_time = time.monotonic()
            self.lines_received += 1

    def animate(self) -> None:
        for index in range(3):
            self.display[index] = lerp_angle(self.display[index], self.target[index], 0.28)
        self.draw()
        self.root.after(16, self.animate)

    @staticmethod
    def rotation_matrix(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, ...], ...]:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    @staticmethod
    def transform(point: tuple[float, float, float], matrix: tuple[tuple[float, ...], ...]) -> tuple[float, float, float]:
        x, y, z = point
        return (
            matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
            matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
            matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
        )

    @staticmethod
    def project(point: tuple[float, float, float], center_x: float, center_y: float, focal: float) -> tuple[float, float]:
        x, y, z = point
        scale = focal / (6.0 - z)
        return center_x + x * scale, center_y - y * scale

    def draw(self) -> None:
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        self.canvas.delete("all")
        self.draw_background(width, height)

        center_x = width * 0.57
        center_y = height * 0.52
        focal = min(width, height) * 1.55
        matrix = self.rotation_matrix(*self.display)
        transformed = [self.transform(vertex, matrix) for vertex in self.VERTICES]
        projected = [self.project(point, center_x, center_y, focal) for point in transformed]

        face_order = sorted(
            range(len(self.FACES)),
            key=lambda index: sum(transformed[vertex][2] for vertex in self.FACES[index]) / 4.0,
        )
        for face_index in face_order:
            face = self.FACES[face_index]
            coordinates = [coordinate for vertex in face for coordinate in projected[vertex]]
            self.canvas.create_polygon(
                coordinates,
                fill=self.FACE_COLORS[face_index],
                outline="#f3ead5",
                width=2,
            )

        self.draw_axes(matrix, center_x, center_y, focal)
        self.draw_status(width, height)

    def draw_background(self, width: int, height: int) -> None:
        horizon = int(height * 0.74)
        self.canvas.create_rectangle(0, horizon, width, height, fill="#17242c", outline="")
        for offset in range(-8, 9):
            x = width * 0.55 + offset * width * 0.08
            self.canvas.create_line(width * 0.55, horizon, x, height, fill=self.GRID)
        for row in range(1, 6):
            y = horizon + (height - horizon) * (row / 6.0) ** 0.55
            self.canvas.create_line(0, y, width, y, fill=self.GRID)

    def draw_axes(self, matrix: tuple[tuple[float, ...], ...], center_x: float, center_y: float, focal: float) -> None:
        origin = self.project((0.0, 0.0, 0.0), center_x, center_y, focal)
        axes = (((2.0, 0.0, 0.0), "#ff665e", "X"), ((0.0, 2.0, 0.0), "#68d391", "Y"), ((0.0, 0.0, 2.0), "#63b3ed", "Z"))
        for endpoint, color, label in axes:
            transformed = self.transform(endpoint, matrix)
            target = self.project(transformed, center_x, center_y, focal)
            self.canvas.create_line(*origin, *target, fill=color, width=3, arrow=tk.LAST)
            self.canvas.create_text(target[0], target[1] - 12, text=label, fill=color, font=("TkDefaultFont", 11, "bold"))

    def draw_status(self, width: int, height: int) -> None:
        roll, pitch, yaw = self.target
        connected = self.demo or (self.serial_port is not None and self.serial_port.is_open)
        fresh = self.have_sample and (time.monotonic() - self.last_sample_wall_time < 0.5)
        state = "LIVE" if connected and fresh else "WAITING FOR DATA"
        state_color = "#68d391" if state == "LIVE" else "#e9a23b"

        self.canvas.create_text(40, 38, anchor="nw", text="IMU ATTITUDE", fill=self.TEXT, font=("TkDefaultFont", 22, "bold"))
        self.canvas.create_text(42, 82, anchor="nw", text=state, fill=state_color, font=("TkDefaultFont", 11, "bold"))
        self.canvas.create_text(40, 130, anchor="nw", text=f"ROLL   {roll:8.2f} deg", fill="#ff8a70", font=("TkFixedFont", 16, "bold"))
        self.canvas.create_text(40, 170, anchor="nw", text=f"PITCH  {pitch:8.2f} deg", fill="#70d6a0", font=("TkFixedFont", 16, "bold"))
        self.canvas.create_text(40, 210, anchor="nw", text=f"YAW    {yaw:8.2f} deg", fill="#72b7e8", font=("TkFixedFont", 16, "bold"))
        self.canvas.create_text(42, height - 52, anchor="sw", text=f"sensor t={self.sensor_time_ms} ms  |  frames={self.lines_received}", fill=self.MUTED, font=("TkDefaultFont", 10))
        self.canvas.create_text(width - 28, height - 28, anchor="se", text="Mahony 6DOF / yaw is not magnetically locked", fill=self.MUTED, font=("TkDefaultFont", 10))


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Visualize STM32 ICM20948 RPY serial data as a 3D cube.")
    parser.add_argument("--port", help="Serial port, for example /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--demo", action="store_true", help="Run an animated demo without a serial device")
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    serial_port = None

    if not args.demo:
        try:
            port = choose_port(args.port)
            serial_port = serial.Serial(port, args.baud, timeout=0)
            serial_port.reset_input_buffer()
            print(f"Reading {port} at {args.baud} baud")
        except (RuntimeError, serial.SerialException) as error:
            print(f"Serial error: {error}", file=sys.stderr)
            return 1

    root = tk.Tk()
    ImuCubeApp(root, serial_port, args.demo)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
