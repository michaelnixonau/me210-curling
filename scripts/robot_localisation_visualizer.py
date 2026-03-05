#!/usr/bin/env python3
"""
Live visualizer for robot_localisation telemetry.

Reads CSV lines from a network serial socket (default socket://192.168.4.1:328)
and renders the robot pose in a 4' x 16' arena.
"""

from __future__ import annotations

import argparse
import math
import queue
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

try:
    import tkinter as tk
except ModuleNotFoundError:
    tk = None  # type: ignore[assignment]


FT_TO_MM = 304.8
ARENA_WIDTH_MM = 4.0 * FT_TO_MM
ARENA_LENGTH_MM = 16.0 * FT_TO_MM


@dataclass
class PoseSample:
    t_ms: int
    x_mm: float
    y_mm: float
    x_ft: float
    y_ft: float
    heading_arena_deg: float
    heading_mag_arena_deg: float
    heading_mag_global_deg: float
    gyro_z_dps: float
    accel_norm_mps2: float
    rmse_mm: float
    used_ranges: int
    half: str
    n_mm: int
    s_mm: int
    e_mm: int
    w_mm: int
    valid_pose: bool


@dataclass
class StatusMessage:
    state: str
    text: str


def parse_socket_url(url: str) -> tuple[str, int]:
    prefix = "socket://"
    raw = url[len(prefix) :] if url.startswith(prefix) else url
    if ":" not in raw:
        raise ValueError(f"Expected host:port, got: {url}")
    host, port_str = raw.rsplit(":", 1)
    if not host:
        raise ValueError(f"Missing host in URL: {url}")
    return host, int(port_str)


def parse_pose_line(line: str) -> Optional[PoseSample]:
    if not line.startswith("POSE,"):
        return None

    parts = line.strip().split(",")
    if len(parts) < 18:
        return None

    try:
        # Skip CSV header lines like: POSE,t_ms,x_mm,...
        t_ms = int(float(parts[1]))
        x_mm = float(parts[2])
        y_mm = float(parts[3])
        x_ft = float(parts[4])
        y_ft = float(parts[5])
        heading_arena_deg = float(parts[6])
        heading_mag_arena_deg = float(parts[7])
        heading_mag_global_deg = float(parts[8])
        gyro_z_dps = float(parts[9])
        accel_norm_mps2 = float(parts[10])
        rmse_mm = float(parts[11])
        used_ranges = int(float(parts[12]))
        half = parts[13].strip()
        n_mm = int(float(parts[14]))
        s_mm = int(float(parts[15]))
        e_mm = int(float(parts[16]))
        w_mm = int(float(parts[17]))
    except ValueError:
        return None

    return PoseSample(
        t_ms=t_ms,
        x_mm=x_mm,
        y_mm=y_mm,
        x_ft=x_ft,
        y_ft=y_ft,
        heading_arena_deg=heading_arena_deg,
        heading_mag_arena_deg=heading_mag_arena_deg,
        heading_mag_global_deg=heading_mag_global_deg,
        gyro_z_dps=gyro_z_dps,
        accel_norm_mps2=accel_norm_mps2,
        rmse_mm=rmse_mm,
        used_ranges=used_ranges,
        half=half,
        n_mm=n_mm,
        s_mm=s_mm,
        e_mm=e_mm,
        w_mm=w_mm,
        valid_pose=(x_mm >= 0.0 and y_mm >= 0.0),
    )


class SocketReader(threading.Thread):
    def __init__(
        self,
        host: str,
        port: int,
        output_queue: queue.Queue[object],
        stop_event: threading.Event,
        reconnect_delay_s: float = 1.0,
    ) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.reconnect_delay_s = reconnect_delay_s

    def run(self) -> None:
        while not self.stop_event.is_set():
            self.output_queue.put(
                StatusMessage("connecting", f"Connecting to {self.host}:{self.port}...")
            )
            try:
                with socket.create_connection((self.host, self.port), timeout=2.0) as sock:
                    sock.settimeout(1.0)
                    self.output_queue.put(
                        StatusMessage("connected", f"Connected to {self.host}:{self.port}")
                    )
                    self._read_stream(sock)
            except OSError as exc:
                self.output_queue.put(StatusMessage("error", f"Connection error: {exc}"))
                if self.stop_event.wait(self.reconnect_delay_s):
                    break

    def _read_stream(self, sock: socket.socket) -> None:
        buffer = b""
        while not self.stop_event.is_set():
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue

            if not chunk:
                self.output_queue.put(StatusMessage("disconnected", "Disconnected"))
                return

            buffer += chunk
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode("utf-8", errors="replace").strip()
                sample = parse_pose_line(line)
                if sample is not None:
                    self.output_queue.put(sample)


class VisualizerApp:
    def __init__(self, root: tk.Tk, source_url: str, trail_size: int) -> None:
        self.root = root
        self.root.title("ME210 Robot Localisation Visualizer")

        self.queue: queue.Queue[object] = queue.Queue()
        self.stop_event = threading.Event()
        host, port = parse_socket_url(source_url)
        self.reader = SocketReader(host, port, self.queue, self.stop_event)

        self.latest_sample: Optional[PoseSample] = None
        self.last_valid_sample: Optional[PoseSample] = None
        self.trail: deque[tuple[float, float]] = deque(maxlen=trail_size)
        self.last_sample_wall_time = 0.0

        self.status_var = tk.StringVar(value="Starting...")
        self.status_color = "gray25"
        self.pose_var = tk.StringVar(value="x: -- ft, y: -- ft, heading: -- deg")
        self.detail_var = tk.StringVar(
            value="rmse: -- mm, used ranges: --, half: --, N/S/E/W: --/--/--/--"
        )

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.reader.start()

        self.root.after(20, self.process_queue)
        self.root.after(33, self.redraw)

    def _build_ui(self) -> None:
        container = tk.Frame(self.root, padx=10, pady=10)
        container.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(container, width=760, height=520, bg="#f7f6f2", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        status_label = tk.Label(
            container, textvariable=self.status_var, anchor="w", fg=self.status_color, font=("Menlo", 12, "bold")
        )
        status_label.pack(fill="x", pady=(8, 0))
        self.status_label = status_label

        pose_label = tk.Label(container, textvariable=self.pose_var, anchor="w", font=("Menlo", 11))
        pose_label.pack(fill="x", pady=(4, 0))

        detail_label = tk.Label(container, textvariable=self.detail_var, anchor="w", font=("Menlo", 10))
        detail_label.pack(fill="x", pady=(2, 0))

    def set_status(self, state: str, text: str) -> None:
        self.status_var.set(text)
        if state == "connected":
            self.status_color = "#1f6f3a"
        elif state in ("error", "disconnected"):
            self.status_color = "#a32121"
        else:
            self.status_color = "gray25"
        self.status_label.configure(fg=self.status_color)

    def process_queue(self) -> None:
        while True:
            try:
                item = self.queue.get_nowait()
            except queue.Empty:
                break

            if isinstance(item, StatusMessage):
                self.set_status(item.state, item.text)
                continue

            if isinstance(item, PoseSample):
                self.latest_sample = item
                self.last_sample_wall_time = time.time()

                if item.valid_pose:
                    self.last_valid_sample = item
                    self.trail.append((item.x_mm, item.y_mm))

                self.pose_var.set(
                    f"x: {item.x_ft:5.2f} ft, y: {item.y_ft:5.2f} ft, "
                    f"heading: {item.heading_arena_deg:6.1f} deg"
                )
                self.detail_var.set(
                    f"rmse: {item.rmse_mm:5.1f} mm, used ranges: {item.used_ranges}, half: {item.half}, "
                    f"N/S/E/W: {item.n_mm}/{item.s_mm}/{item.e_mm}/{item.w_mm}"
                )

        if self.latest_sample and (time.time() - self.last_sample_wall_time) > 2.0:
            self.set_status("disconnected", "No telemetry for >2 s")

        self.root.after(20, self.process_queue)

    def arena_to_canvas(
        self, x_mm: float, y_mm: float, left: float, top: float, scale: float
    ) -> tuple[float, float]:
        x_px = left + x_mm * scale
        y_px = top + (ARENA_LENGTH_MM - y_mm) * scale
        return x_px, y_px

    def redraw(self) -> None:
        self.canvas.delete("all")

        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())

        margin = 36.0
        scale = min(
            (width - 2.0 * margin) / ARENA_WIDTH_MM,
            (height - 2.0 * margin) / ARENA_LENGTH_MM,
        )
        arena_px_w = ARENA_WIDTH_MM * scale
        arena_px_h = ARENA_LENGTH_MM * scale
        left = (width - arena_px_w) / 2.0
        top = (height - arena_px_h) / 2.0
        right = left + arena_px_w
        bottom = top + arena_px_h

        # Arena boundary and half split line.
        self.canvas.create_rectangle(left, top, right, bottom, width=2, outline="#2d2d2d", fill="#faf6e8")
        _, mid_y = self.arena_to_canvas(0.0, ARENA_LENGTH_MM * 0.5, left, top, scale)
        self.canvas.create_line(left, mid_y, right, mid_y, fill="#9f9f9f", dash=(5, 5), width=2)

        # Axis labels in arena frame.
        self.canvas.create_text((left + right) / 2.0, top - 16, text="N (+Y)", fill="#333333", font=("Menlo", 10, "bold"))
        self.canvas.create_text((left + right) / 2.0, bottom + 16, text="S (0Y)", fill="#333333", font=("Menlo", 10, "bold"))
        self.canvas.create_text(left - 24, (top + bottom) / 2.0, text="W (0X)", fill="#333333", font=("Menlo", 10, "bold"), angle=90)
        self.canvas.create_text(right + 24, (top + bottom) / 2.0, text="E (+X)", fill="#333333", font=("Menlo", 10, "bold"), angle=270)

        # Draw trail.
        if len(self.trail) >= 2:
            trail_points = []
            for x_mm, y_mm in self.trail:
                px, py = self.arena_to_canvas(x_mm, y_mm, left, top, scale)
                trail_points.extend((px, py))
            self.canvas.create_line(*trail_points, fill="#3e8fcb", width=2, smooth=True)

        # Draw current robot.
        sample = self.last_valid_sample
        if sample is not None:
            cx, cy = self.arena_to_canvas(sample.x_mm, sample.y_mm, left, top, scale)
            heading_rad = math.radians(sample.heading_arena_deg)

            hx = math.sin(heading_rad)
            hy = -math.cos(heading_rad)
            px = -hy
            py = hx

            nose_len = 18.0
            tail_len = 10.0
            half_width = 9.0

            nose = (cx + hx * nose_len, cy + hy * nose_len)
            tail_center = (cx - hx * tail_len, cy - hy * tail_len)
            tail_left = (tail_center[0] + px * half_width, tail_center[1] + py * half_width)
            tail_right = (tail_center[0] - px * half_width, tail_center[1] - py * half_width)

            self.canvas.create_polygon(
                nose[0],
                nose[1],
                tail_left[0],
                tail_left[1],
                tail_right[0],
                tail_right[1],
                fill="#d83f2f",
                outline="#6d1811",
                width=2,
            )
            self.canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#101010", outline="")

        self.root.after(33, self.redraw)

    def on_close(self) -> None:
        self.stop_event.set()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Live visualizer for robot localisation telemetry.")
    parser.add_argument(
        "--url",
        default="socket://192.168.4.1:328",
        help="Source URL in host:port or socket://host:port form (default: %(default)s)",
    )
    parser.add_argument(
        "--trail-size",
        type=int,
        default=250,
        help="Number of historic valid positions to keep in the path trail (default: %(default)s)",
    )
    args = parser.parse_args()

    if tk is None:
        raise SystemExit(
            "tkinter is not available in this Python install. "
            "Install Tk support or run with a Python distribution that includes tkinter."
        )

    root = tk.Tk()
    root.minsize(700, 500)
    VisualizerApp(root, args.url, max(10, args.trail_size))
    root.mainloop()


if __name__ == "__main__":
    main()
