#!/usr/bin/env python3
"""
Scan 360 degrees and report angle ranges with persistent close returns.
Use this to identify lidar angles that see your own robot frame.
"""

import argparse
import math
import time
from typing import Dict, List, Optional, Tuple

from serial.tools import list_ports
from rplidar import RPLidar


def pick_port(requested: str) -> Optional[str]:
    if requested and requested != "auto":
        return requested

    ports = list(list_ports.comports())
    for p in ports:
        hay = " ".join(x for x in [p.manufacturer, p.product, p.description] if x).lower()
        if "slamtec" in hay or "rplidar" in hay:
            return p.device

    for p in ports:
        if p.device.startswith("/dev/ttyUSB") or p.device.startswith("/dev/ttyACM"):
            return p.device
    return None


def collapse_ranges(degrees: List[int]) -> List[Tuple[int, int]]:
    if not degrees:
        return []
    degrees = sorted(set(degrees))

    ranges: List[Tuple[int, int]] = []
    start = prev = degrees[0]
    for d in degrees[1:]:
        if d == prev + 1:
            prev = d
            continue
        ranges.append((start, prev))
        start = prev = d
    ranges.append((start, prev))

    # Merge wrap-around range: e.g., 350..359 and 0..8 -> 350..8
    if len(ranges) >= 2 and ranges[0][0] == 0 and ranges[-1][1] == 359:
        first = ranges[0]
        last = ranges[-1]
        merged = (last[0], first[1])
        ranges = [merged] + ranges[1:-1]
    return ranges


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else math.nan


def main():
    parser = argparse.ArgumentParser(description="Find lidar angles with close persistent returns")
    parser.add_argument("--port", default="auto", help="Lidar serial port or 'auto'")
    parser.add_argument("--duration", type=float, default=8.0, help="Scan duration in seconds")
    parser.add_argument("--threshold-mm", type=int, default=800, help="Close-range threshold in mm")
    parser.add_argument("--min-mm", type=int, default=60, help="Ignore unrealistically tiny returns")
    parser.add_argument(
        "--min-ratio",
        type=float,
        default=0.35,
        help="Minimum hit ratio per degree to classify as persistent close return",
    )
    args = parser.parse_args()

    port = pick_port(args.port)
    if not port:
        raise SystemExit("No lidar serial port found.")

    total = [0] * 360
    close_hits = [0] * 360
    min_dist = [math.inf] * 360

    lidar = None
    t0 = time.time()
    print(
        f"[SCAN] starting port={port} duration={args.duration}s "
        f"threshold={args.threshold_mm}mm min_ratio={args.min_ratio}"
    )
    try:
        lidar = RPLidar(port, timeout=1)
        # Some rplidar package builds return 3+ values from get_health(),
        # while iter_scans/start expects exactly 2. Normalize defensively.
        original_get_health = lidar.get_health

        def _safe_get_health():
            out = original_get_health()
            if isinstance(out, tuple):
                if len(out) >= 2:
                    return out[0], out[1]
            # Fallback for object-like return.
            status = getattr(out, "status", None)
            error_code = getattr(out, "error_code", 0)
            return status, error_code

        lidar.get_health = _safe_get_health

        for scan in lidar.iter_scans(max_buf_meas=1200):
            for _quality, angle, distance in scan:
                if distance <= 0:
                    continue
                deg = int(angle) % 360
                total[deg] += 1
                if distance < min_dist[deg]:
                    min_dist[deg] = distance
                if args.min_mm <= distance <= args.threshold_mm:
                    close_hits[deg] += 1

            if (time.time() - t0) >= args.duration:
                break
    finally:
        if lidar is not None:
            try:
                lidar.stop()
            except Exception:
                pass
            try:
                lidar.stop_motor()
            except Exception:
                pass
            try:
                lidar.disconnect()
            except Exception:
                pass

    close_degrees: List[int] = []
    for deg in range(360):
        if total[deg] == 0:
            continue
        ratio = close_hits[deg] / float(total[deg])
        if ratio >= args.min_ratio and min_dist[deg] <= args.threshold_mm:
            close_degrees.append(deg)

    ranges = collapse_ranges(close_degrees)
    print(f"[SCAN] sampled_degrees={sum(1 for x in total if x > 0)}")
    print(f"[SCAN] close_degrees={len(close_degrees)}")
    if not ranges:
        print("[RESULT] No persistent close-return ranges found.")
        return

    print("[RESULT] Suspected robot-echo angle ranges (degrees):")
    for lo, hi in ranges:
        if lo <= hi:
            members = [d for d in close_degrees if lo <= d <= hi]
        else:
            members = [d for d in close_degrees if d >= lo or d <= hi]
        mins = [min_dist[d] for d in members if math.isfinite(min_dist[d])]
        avg_min = mean(mins)
        if lo <= hi:
            width = hi - lo + 1
            print(f"  - {lo:03d}..{hi:03d} (width={width} deg, avg_min={avg_min:.1f} mm)")
        else:
            width = (359 - lo + 1) + (hi + 1)
            print(f"  - {lo:03d}..359 and 000..{hi:03d} (width={width} deg, avg_min={avg_min:.1f} mm)")

    print("\nTip: run with no nearby walls/objects to isolate robot self-echo.")


if __name__ == "__main__":
    main()
