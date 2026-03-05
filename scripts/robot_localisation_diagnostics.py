#!/usr/bin/env python3
"""
Capture and analyze robot_localisation telemetry.

Usage examples:
  python3 scripts/robot_localisation_diagnostics.py capture --duration 90
  python3 scripts/robot_localisation_diagnostics.py analyze logs/robot_localisation_diag_20260304_151000.csv
"""

from __future__ import annotations

import argparse
import math
import socket
import statistics
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional


ARENA_WIDTH_MM = 4.0 * 304.8
ARENA_LENGTH_MM = 16.0 * 304.8
ARENA_HALF_LENGTH_MM = ARENA_LENGTH_MM * 0.5

GRID_X_COARSE_MM = 40.0
GRID_Y_COARSE_MM = 80.0
GRID_FINE_MM = 20.0
GRID_FINE_WINDOW_MM = 80.0
RESIDUAL_CLAMP_MM = 700.0

STATIONARY_GYRO_THRESH_DPS = 0.8
STATIONARY_ACCEL_THRESH_MPS2 = 0.35
ACCEL_NOMINAL_MPS2 = 9.80665

HALF_SWITCH_MARGIN_MM = 80.0

SENSOR_SOUTH = 0
SENSOR_WEST = 1
SENSOR_NORTH = 2
SENSOR_EAST = 3
SENSOR_COUNT = 4


@dataclass
class PoseSample:
    t_ms: int
    x_mm: float
    y_mm: float
    heading_arena_deg: float
    heading_mag_arena_deg: float
    gyro_z_dps: float
    accel_norm_mps2: float
    rmse_mm: float
    used_ranges: int
    half: str
    n_mm: int
    s_mm: int
    e_mm: int
    w_mm: int
    heading_err_deg: Optional[float]
    stationary_flag: Optional[int]

    @property
    def valid_pose(self) -> bool:
        return self.x_mm >= 0.0 and self.y_mm >= 0.0

    @property
    def ranges_by_sensor(self) -> list[int]:
        return [self.s_mm, self.w_mm, self.n_mm, self.e_mm]


@dataclass
class PoseEstimate:
    valid: bool
    x_mm: float = 0.0
    y_mm: float = 0.0
    rmse_mm: float = 0.0
    used_ranges: int = 0


def wrap180(angle_deg: float) -> float:
    while angle_deg >= 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def wrap360(angle_deg: float) -> float:
    while angle_deg >= 360.0:
        angle_deg -= 360.0
    while angle_deg < 0.0:
        angle_deg += 360.0
    return angle_deg


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def percentile(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    if p <= 0:
        return min(values)
    if p >= 100:
        return max(values)
    sorted_values = sorted(values)
    rank = (len(sorted_values) - 1) * (p / 100.0)
    low_i = int(math.floor(rank))
    high_i = int(math.ceil(rank))
    if low_i == high_i:
        return sorted_values[low_i]
    mix = rank - low_i
    return sorted_values[low_i] * (1.0 - mix) + sorted_values[high_i] * mix


def parse_socket_url(url: str) -> tuple[str, int]:
    raw = url[len("socket://") :] if url.startswith("socket://") else url
    if ":" not in raw:
        raise ValueError(f"Expected host:port, got {url}")
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
        t_ms = int(float(parts[1]))
        x_mm = float(parts[2])
        y_mm = float(parts[3])
        heading_arena_deg = float(parts[6])
        heading_mag_arena_deg = float(parts[7])
        gyro_z_dps = float(parts[9])
        accel_norm_mps2 = float(parts[10])
        rmse_mm = float(parts[11])
        used_ranges = int(float(parts[12]))
        half = parts[13].strip().lower()
        n_mm = int(float(parts[14]))
        s_mm = int(float(parts[15]))
        e_mm = int(float(parts[16]))
        w_mm = int(float(parts[17]))
    except ValueError:
        return None

    heading_err_deg = None
    stationary_flag = None
    if len(parts) >= 19:
        try:
            heading_err_deg = float(parts[18])
        except ValueError:
            heading_err_deg = None
    if len(parts) >= 20:
        try:
            stationary_flag = int(float(parts[19]))
        except ValueError:
            stationary_flag = None

    return PoseSample(
        t_ms=t_ms,
        x_mm=x_mm,
        y_mm=y_mm,
        heading_arena_deg=heading_arena_deg,
        heading_mag_arena_deg=heading_mag_arena_deg,
        gyro_z_dps=gyro_z_dps,
        accel_norm_mps2=accel_norm_mps2,
        rmse_mm=rmse_mm,
        used_ranges=used_ranges,
        half=half,
        n_mm=n_mm,
        s_mm=s_mm,
        e_mm=e_mm,
        w_mm=w_mm,
        heading_err_deg=heading_err_deg,
        stationary_flag=stationary_flag,
    )


def load_samples(path: Path) -> list[PoseSample]:
    samples: list[PoseSample] = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw_line in f:
            sample = parse_pose_line(raw_line.strip())
            if sample is not None:
                samples.append(sample)
    return samples


def is_stationary(sample: PoseSample) -> bool:
    if sample.stationary_flag is not None:
        return sample.stationary_flag > 0
    return (
        abs(sample.gyro_z_dps) < STATIONARY_GYRO_THRESH_DPS
        and abs(sample.accel_norm_mps2 - ACCEL_NOMINAL_MPS2) < STATIONARY_ACCEL_THRESH_MPS2
    )


def ray_distance_to_boundary_mm(x_mm: float, y_mm: float, bearing_deg: float) -> float:
    rad = math.radians(bearing_deg)
    dx = math.sin(rad)
    dy = math.cos(rad)
    eps = 1.0e-5

    t_min = 1.0e9
    if dx > eps:
        tx = (ARENA_WIDTH_MM - x_mm) / dx
        if tx >= 0.0:
            t_min = min(t_min, tx)
    elif dx < -eps:
        tx = (0.0 - x_mm) / dx
        if tx >= 0.0:
            t_min = min(t_min, tx)

    if dy > eps:
        ty = (ARENA_LENGTH_MM - y_mm) / dy
        if ty >= 0.0:
            t_min = min(t_min, ty)
    elif dy < -eps:
        ty = (0.0 - y_mm) / dy
        if ty >= 0.0:
            t_min = min(t_min, ty)

    if t_min > 9000.0:
        return -1.0
    return t_min


def compute_cost(
    x_mm: float,
    y_mm: float,
    sensor_bearings_deg: list[float],
    ranges_mm: list[int],
) -> tuple[bool, float, int]:
    total_cost = 0.0
    used = 0
    for sensor_idx in range(SENSOR_COUNT):
        measured = ranges_mm[sensor_idx]
        if measured < 0:
            continue

        predicted = ray_distance_to_boundary_mm(x_mm, y_mm, sensor_bearings_deg[sensor_idx])
        if predicted < 0.0:
            continue

        residual = clamp(float(measured) - predicted, -RESIDUAL_CLAMP_MM, RESIDUAL_CLAMP_MM)
        total_cost += residual * residual
        used += 1

    if used < 2:
        return (False, 0.0, 0)
    return (True, total_cost, used)


def frange(start: float, end: float, step: float) -> list[float]:
    values: list[float] = []
    value = start
    while value <= end + 0.5:
        values.append(value)
        value += step
    return values


def solve_pose_for_half(ranges_mm: list[int], heading_deg: float, half: str) -> PoseEstimate:
    valid_count = sum(1 for mm in ranges_mm if mm >= 0)
    if valid_count < 2:
        return PoseEstimate(False)

    sensor_bearings = [0.0] * SENSOR_COUNT
    sensor_bearings[SENSOR_NORTH] = wrap360(heading_deg)
    sensor_bearings[SENSOR_SOUTH] = wrap360(heading_deg + 180.0)
    sensor_bearings[SENSOR_EAST] = wrap360(heading_deg + 90.0)
    sensor_bearings[SENSOR_WEST] = wrap360(heading_deg + 270.0)

    y_min = 0.0
    y_max = ARENA_LENGTH_MM
    if half == "south":
        y_max = ARENA_HALF_LENGTH_MM
    elif half == "north":
        y_min = ARENA_HALF_LENGTH_MM

    best_found = False
    best_cost = 0.0
    best_x = 0.0
    best_y = 0.0
    best_used = 0

    for y in frange(y_min, y_max, GRID_Y_COARSE_MM):
        for x in frange(0.0, ARENA_WIDTH_MM, GRID_X_COARSE_MM):
            valid, cost, used = compute_cost(x, y, sensor_bearings, ranges_mm)
            if not valid:
                continue
            if not best_found or cost < best_cost:
                best_found = True
                best_cost = cost
                best_x = x
                best_y = y
                best_used = used

    if not best_found:
        return PoseEstimate(False)

    x_start = clamp(best_x - GRID_FINE_WINDOW_MM, 0.0, ARENA_WIDTH_MM)
    x_end = clamp(best_x + GRID_FINE_WINDOW_MM, 0.0, ARENA_WIDTH_MM)
    y_start = clamp(best_y - GRID_FINE_WINDOW_MM, y_min, y_max)
    y_end = clamp(best_y + GRID_FINE_WINDOW_MM, y_min, y_max)

    for y in frange(y_start, y_end, GRID_FINE_MM):
        for x in frange(x_start, x_end, GRID_FINE_MM):
            valid, cost, used = compute_cost(x, y, sensor_bearings, ranges_mm)
            if not valid:
                continue
            if cost < best_cost:
                best_cost = cost
                best_x = x
                best_y = y
                best_used = used

    rmse_mm = math.sqrt(best_cost / float(best_used))
    return PoseEstimate(True, best_x, best_y, rmse_mm, best_used)


def capture(url: str, duration_s: float, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    host, port = parse_socket_url(url)

    print(f"Connecting to {host}:{port}")
    with socket.create_connection((host, port), timeout=3.0) as sock:
        sock.settimeout(1.0)
        deadline = time.monotonic() + max(1.0, duration_s)
        count = 0
        buffer = b""

        with out_path.open("w", encoding="utf-8") as out:
            while time.monotonic() < deadline:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue

                if not chunk:
                    print("Socket closed by peer.")
                    break

                buffer += chunk
                while b"\n" in buffer:
                    raw_line, buffer = buffer.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if not line.startswith("POSE,"):
                        continue
                    out.write(line + "\n")
                    count += 1
                    if count % 50 == 0:
                        remaining = max(0.0, deadline - time.monotonic())
                        print(f"Captured {count} POSE lines ({remaining:.1f}s remaining)")

    print(f"Wrote {count} POSE lines to {out_path}")


def summarize_heading(samples: list[PoseSample]) -> None:
    errors = [abs(wrap180(s.heading_mag_arena_deg - s.heading_arena_deg)) for s in samples]
    if not errors:
        print("Heading: no samples")
        return
    print("Heading agreement (arena vs magnetometer):")
    print(f"  mean abs error: {statistics.fmean(errors):.2f} deg")
    print(f"  p95 abs error:  {percentile(errors, 95):.2f} deg")
    print(f"  max abs error:  {max(errors):.2f} deg")


def summarize_stationary(samples: list[PoseSample], min_stationary_s: float) -> None:
    if len(samples) < 2:
        print("Stationary drift: not enough samples")
        return

    segments: list[tuple[int, int]] = []
    start_idx: Optional[int] = None
    max_gap_ms = 250

    for i, sample in enumerate(samples):
        stationary_now = is_stationary(sample)
        if stationary_now and sample.valid_pose:
            if start_idx is None:
                start_idx = i
            elif samples[i].t_ms - samples[i - 1].t_ms > max_gap_ms:
                segments.append((start_idx, i - 1))
                start_idx = i
        else:
            if start_idx is not None:
                segments.append((start_idx, i - 1))
                start_idx = None

    if start_idx is not None:
        segments.append((start_idx, len(samples) - 1))

    drift_rates: list[float] = []
    drift_distances: list[float] = []
    heading_drifts: list[float] = []
    jitters: list[float] = []
    kept_segments = 0

    for start, end in segments:
        if end <= start:
            continue
        a = samples[start]
        b = samples[end]
        duration_s = (b.t_ms - a.t_ms) / 1000.0
        if duration_s < min_stationary_s:
            continue
        if not (a.valid_pose and b.valid_pose):
            continue

        dx = b.x_mm - a.x_mm
        dy = b.y_mm - a.y_mm
        drift_mm = math.hypot(dx, dy)
        drift_rate = (drift_mm / duration_s) * 60.0
        heading_drift = abs(wrap180(b.heading_arena_deg - a.heading_arena_deg))

        xs = [s.x_mm for s in samples[start : end + 1] if s.valid_pose]
        ys = [s.y_mm for s in samples[start : end + 1] if s.valid_pose]
        if xs and ys:
            mean_x = statistics.fmean(xs)
            mean_y = statistics.fmean(ys)
            radius_sq = [
                (x - mean_x) * (x - mean_x) + (y - mean_y) * (y - mean_y)
                for x, y in zip(xs, ys)
            ]
            jitter_mm = math.sqrt(statistics.fmean(radius_sq))
            jitters.append(jitter_mm)

        drift_rates.append(drift_rate)
        drift_distances.append(drift_mm)
        heading_drifts.append(heading_drift)
        kept_segments += 1

    if kept_segments == 0:
        print(f"Stationary drift: no stationary segments >= {min_stationary_s:.1f}s")
        return

    print(f"Stationary drift ({kept_segments} segments, >= {min_stationary_s:.1f}s):")
    print(f"  mean drift rate: {statistics.fmean(drift_rates):.1f} mm/min")
    print(f"  p95 drift rate:  {percentile(drift_rates, 95):.1f} mm/min")
    print(f"  max drift:       {max(drift_distances):.1f} mm")
    print(f"  max heading drift in segment: {max(heading_drifts):.2f} deg")
    if jitters:
        print(f"  mean position jitter radius: {statistics.fmean(jitters):.1f} mm")


def summarize_half_consistency(samples: list[PoseSample], max_samples: int) -> None:
    candidates = [s for s in samples if s.used_ranges >= 2]
    if not candidates:
        print("Half consistency: no samples with >=2 ranges")
        return

    if len(candidates) > max_samples:
        stride = max(1, len(candidates) // max_samples)
        sampled = candidates[::stride][:max_samples]
    else:
        sampled = candidates

    both_valid = 0
    disagree = 0
    strong_disagree = 0
    north_better = 0
    south_better = 0
    margin_values: list[float] = []

    for s in sampled:
        south = solve_pose_for_half(s.ranges_by_sensor, s.heading_arena_deg, "south")
        north = solve_pose_for_half(s.ranges_by_sensor, s.heading_arena_deg, "north")
        if not (south.valid and north.valid):
            continue

        both_valid += 1
        diff = abs(north.rmse_mm - south.rmse_mm)
        margin_values.append(diff)

        best_half = "north" if north.rmse_mm < south.rmse_mm else "south"
        if best_half == "north":
            north_better += 1
        else:
            south_better += 1

        reported_half = s.half
        if reported_half in ("north", "south") and reported_half != best_half:
            disagree += 1
            if diff > HALF_SWITCH_MARGIN_MM:
                strong_disagree += 1

    if both_valid == 0:
        print("Half consistency: insufficient samples where both halves are solvable")
        return

    print(f"Half consistency ({both_valid} comparable samples):")
    print(f"  south better: {south_better} ({100.0 * south_better / both_valid:.1f}%)")
    print(f"  north better: {north_better} ({100.0 * north_better / both_valid:.1f}%)")
    print(f"  median half RMSE gap: {percentile(margin_values, 50):.1f} mm")
    print(f"  p95 half RMSE gap:    {percentile(margin_values, 95):.1f} mm")
    if disagree > 0:
        print(
            f"  reported-vs-best mismatches: {disagree} "
            f"({100.0 * disagree / both_valid:.1f}%)"
        )
    if strong_disagree > 0:
        print(
            f"  strong mismatches (gap > {HALF_SWITCH_MARGIN_MM:.0f} mm): "
            f"{strong_disagree} ({100.0 * strong_disagree / both_valid:.1f}%)"
        )


def summarize_gyro_sign(samples: list[PoseSample]) -> None:
    if len(samples) < 2:
        print("Gyro sign check: not enough samples")
        return

    minus_errors_sq: list[float] = []
    plus_errors_sq: list[float] = []
    corr_num = 0.0
    corr_den = 0.0
    used = 0

    for i in range(1, len(samples)):
        prev = samples[i - 1]
        cur = samples[i]
        dt = (cur.t_ms - prev.t_ms) / 1000.0
        if dt <= 0.0 or dt > 0.25:
            continue
        if abs(prev.gyro_z_dps) < 3.0:
            continue

        d_heading = wrap180(cur.heading_arena_deg - prev.heading_arena_deg)
        gyro_dt = prev.gyro_z_dps * dt

        minus_pred = -gyro_dt
        plus_pred = gyro_dt
        minus_errors_sq.append(wrap180(d_heading - minus_pred) ** 2)
        plus_errors_sq.append(wrap180(d_heading - plus_pred) ** 2)

        corr_num += d_heading * gyro_dt
        corr_den += gyro_dt * gyro_dt
        used += 1

    if used == 0:
        print("Gyro sign check: no turning samples with |gyro| >= 3 dps")
        return

    rmse_minus = math.sqrt(statistics.fmean(minus_errors_sq))
    rmse_plus = math.sqrt(statistics.fmean(plus_errors_sq))
    print(f"Gyro sign check ({used} turning deltas):")
    print(f"  RMSE if sign = -1: {rmse_minus:.2f} deg")
    print(f"  RMSE if sign = +1: {rmse_plus:.2f} deg")

    if corr_den > 1.0e-6:
        gain = corr_num / corr_den
        print(f"  fitted d_heading ~= {gain:.3f} * (gyro_z * dt)")

    if rmse_plus + 1.0 < rmse_minus:
        print("  suggestion: set GYRO_BEARING_SIGN to +1")
    elif rmse_minus + 1.0 < rmse_plus:
        print("  suggestion: keep/use GYRO_BEARING_SIGN = -1")
    else:
        print("  suggestion: inconclusive sign test")


def summarize_basic(samples: list[PoseSample]) -> None:
    valid = sum(1 for s in samples if s.valid_pose)
    with_ranges = sum(1 for s in samples if s.used_ranges >= 2)
    halves: dict[str, int] = {}
    transitions = 0
    prev_half: Optional[str] = None
    for s in samples:
        halves[s.half] = halves.get(s.half, 0) + 1
        if prev_half is not None and s.half != prev_half:
            transitions += 1
        prev_half = s.half

    print("Dataset:")
    print(f"  samples: {len(samples)}")
    print(f"  valid poses: {valid} ({100.0 * valid / max(1, len(samples)):.1f}%)")
    print(f"  samples with >=2 ranges: {with_ranges} ({100.0 * with_ranges / max(1, len(samples)):.1f}%)")
    print(f"  half transitions in telemetry: {transitions}")
    print(f"  half counts: {halves}")


def analyze(path: Path, min_stationary_s: float, max_half_samples: int) -> None:
    samples = load_samples(path)
    if not samples:
        raise SystemExit(f"No parsable POSE samples found in {path}")

    summarize_basic(samples)
    summarize_stationary(samples, min_stationary_s=min_stationary_s)
    summarize_heading(samples)
    summarize_gyro_sign(samples)
    summarize_half_consistency(samples, max_samples=max_half_samples)


def default_capture_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"robot_localisation_diag_{timestamp}.csv"


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture/analyze robot_localisation telemetry.")
    subparsers = parser.add_subparsers(dest="cmd", required=True)

    capture_parser = subparsers.add_parser("capture", help="Capture POSE telemetry from socket.")
    capture_parser.add_argument(
        "--url",
        default="socket://192.168.4.1:328",
        help="Telemetry source URL in socket://host:port format.",
    )
    capture_parser.add_argument(
        "--duration",
        type=float,
        default=90.0,
        help="Capture duration in seconds (default: %(default)s).",
    )
    capture_parser.add_argument(
        "--out",
        type=Path,
        default=default_capture_path(),
        help="Output CSV/log file path (default: logs/robot_localisation_diag_<timestamp>.csv).",
    )

    analyze_parser = subparsers.add_parser("analyze", help="Analyze saved POSE telemetry file.")
    analyze_parser.add_argument("path", type=Path, help="Path to telemetry file.")
    analyze_parser.add_argument(
        "--min-stationary-s",
        type=float,
        default=3.0,
        help="Minimum stationary segment duration in seconds (default: %(default)s).",
    )
    analyze_parser.add_argument(
        "--max-half-samples",
        type=int,
        default=800,
        help="Max samples used for half-consistency recomputation (default: %(default)s).",
    )

    args = parser.parse_args()
    if args.cmd == "capture":
        capture(args.url, args.duration, args.out)
    elif args.cmd == "analyze":
        analyze(args.path, min_stationary_s=args.min_stationary_s, max_half_samples=args.max_half_samples)


if __name__ == "__main__":
    main()
