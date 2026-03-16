#!/usr/bin/env python3
"""
SAR Autonomous Homing Flight Path Generator
Fixed-wing Search & Rescue — PX4 Cube Orange target

Watches the most recent fusion_*.jsonl file for Kraken DOA + telemetry data,
triangulates the highest-confidence target location, and continuously writes
a PX4-compatible JSON waypoint mission file.

Usage:
    python sar_homing.py [--watch-dir DIR] [--output FILE] [--max-bank DEG]
                         [--min-confidence FLOAT] [--min-usable INT]
                         [--loiter-radius FT] [--approach-alt FT]
"""

# TODO: need to modify flight path so that it accounts for direction of plane travel; take last 2 telemetry to calc instead if just most recent record

import argparse
import glob
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Optional

# ──────────────────────────────────────────────
# Constants / defaults
# ──────────────────────────────────────────────
DEFAULT_WATCH_DIR      = "."
DEFAULT_OUTPUT_FILE    = "sar_mission.json"
DEFAULT_MAX_BANK_DEG   = 25.0        # safest comfortable bank angle
DEFAULT_MIN_CONFIDENCE = 0.40        # minimum per-record confidence
DEFAULT_MIN_USABLE     = 3           # min usable records needed to publish
DEFAULT_LOITER_RADIUS_FT = 300.0     # orbit radius above target
DEFAULT_APPROACH_ALT_FT  = 200.0     # AGL for final approach / loiter
POLL_INTERVAL_S        = 0.5         # file-watch cadence (seconds)
STALE_RECORD_MS        = 30_000      # ignore records older than this
MAX_DOA_BUFFER         = 200         # rolling window of usable DOA records

# PX4 MAVLink mission item command IDs (MAV_CMD)
MAV_CMD_NAV_WAYPOINT       = 16
MAV_CMD_NAV_LOITER_UNLIM   = 17
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_DO_CHANGE_SPEED    = 178

EARTH_RADIUS_M = 6_371_000.0

# ──────────────────────────────────────────────
# Geometry helpers
# ──────────────────────────────────────────────

def deg2rad(d: float) -> float:
    return d * math.pi / 180.0

def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres."""
    rlat1, rlon1, rlat2, rlon2 = map(deg2rad, (lat1, lon1, lat2, lon2))
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))

def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from point 1 → point 2 (0–360°)."""
    rlat1, rlon1, rlat2, rlon2 = map(deg2rad, (lat1, lon1, lat2, lon2))
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon)
    return (rad2deg(math.atan2(x, y)) + 360.0) % 360.0

def offset_latlon(lat: float, lon: float, bearing: float, dist_m: float):
    """Return (lat, lon) offset by dist_m along bearing (degrees)."""
    d = dist_m / EARTH_RADIUS_M
    b = deg2rad(bearing)
    rlat1 = deg2rad(lat)
    rlon1 = deg2rad(lon)
    rlat2 = math.asin(
        math.sin(rlat1) * math.cos(d) + math.cos(rlat1) * math.sin(d) * math.cos(b)
    )
    rlon2 = rlon1 + math.atan2(
        math.sin(b) * math.sin(d) * math.cos(rlat1),
        math.cos(d) - math.sin(rlat1) * math.sin(rlat2),
    )
    return rad2deg(rlat2), rad2deg(rlon2)

def doa_to_abs_bearing(doa_deg: float, yaw_deg: float) -> float:
    """Convert relative DOA to absolute geographic bearing."""
    return (yaw_deg + doa_deg) % 360.0

def intersect_two_rays(
    lat1: float, lon1: float, hdg1: float,
    lat2: float, lon2: float, hdg2: float,
) -> Optional[tuple]:
    """
    Approximate planar intersection of two bearing rays.
    Returns (lat, lon) intersection or None if rays are nearly parallel.
    Uses local East-North projection centred on midpoint.
    """
    clat = (lat1 + lat2) / 2.0
    cos_lat = math.cos(deg2rad(clat))

    # Convert to metres in local EN frame
    def to_en(lat, lon):
        e = (lon - clat) * deg2rad(1) * EARTH_RADIUS_M * cos_lat  # crude but fine locally
        n = (lat - clat) * deg2rad(1) * EARTH_RADIUS_M
        return e, n

    # Reference point midpoint = (0, 0)
    e1 = (lon1 - (lon1 + lon2) / 2) * deg2rad(1) * EARTH_RADIUS_M * cos_lat
    n1 = (lat1 - (lat1 + lat2) / 2) * deg2rad(1) * EARTH_RADIUS_M
    e2 = (lon2 - (lon1 + lon2) / 2) * deg2rad(1) * EARTH_RADIUS_M * cos_lat
    n2 = (lat2 - (lat1 + lat2) / 2) * deg2rad(1) * EARTH_RADIUS_M

    # Direction vectors
    r1 = deg2rad(hdg1)
    r2 = deg2rad(hdg2)
    de1, dn1 = math.sin(r1), math.cos(r1)
    de2, dn2 = math.sin(r2), math.cos(r2)

    # Solve: P1 + t*d1 = P2 + s*d2
    denom = de1 * dn2 - dn1 * de2
    if abs(denom) < 1e-9:
        return None  # parallel

    dx = e2 - e1
    dy = n2 - n1
    t = (dx * dn2 - dy * de2) / denom

    ie = e1 + t * de1
    in_ = n1 + t * dn1

    mid_lat = (lat1 + lat2) / 2.0
    mid_lon = (lon1 + lon2) / 2.0
    out_lat = mid_lat + rad2deg(in_ / EARTH_RADIUS_M)
    out_lon = mid_lon + rad2deg(ie / (EARTH_RADIUS_M * cos_lat))
    return out_lat, out_lon


# ──────────────────────────────────────────────
# Target estimator
# ──────────────────────────────────────────────

class TargetEstimator:
    """
    Maintains a rolling buffer of usable DOA records and produces a
    weighted-average target position estimate.
    """

    def __init__(self, min_confidence: float, min_usable: int):
        self.min_confidence = min_confidence
        self.min_usable = min_usable
        self.buffer: deque = deque(maxlen=MAX_DOA_BUFFER)

    def ingest(self, record: dict) -> None:
        if not record.get("usable_for_triangulation"):
            return
        if record.get("confidence_0_1", 0) < self.min_confidence:
            return
        self.buffer.append(record)

    def estimate(self) -> Optional[dict]:
        """
        Returns best estimate dict with keys: lat, lon, confidence, n_records
        or None if insufficient data.

        Staleness is relative to the newest record in the buffer, not wall-clock
        time, so the script works on both static and live/growing files.
        """
        if not self.buffer:
            return None

        newest_t = max(r["t_rx_ms"] for r in self.buffer)
        fresh = [r for r in self.buffer
                 if (newest_t - r["t_rx_ms"]) <= STALE_RECORD_MS]

        if len(fresh) < self.min_usable:
            return None

        # Pairwise intersections weighted by product of confidences
        intersections = []  # (lat, lon, weight)
        for i in range(len(fresh)):
            for j in range(i + 1, len(fresh)):
                r1, r2 = fresh[i], fresh[j]
                hdg1 = doa_to_abs_bearing(r1["doa_deg"], r1["yaw_deg"])
                hdg2 = doa_to_abs_bearing(r2["doa_deg"], r2["yaw_deg"])
                pt = intersect_two_rays(
                    r1["lat_deg"], r1["lon_deg"], hdg1,
                    r2["lat_deg"], r2["lon_deg"], hdg2,
                )
                if pt is None:
                    continue
                w = r1["confidence_0_1"] * r2["confidence_0_1"]
                intersections.append((pt[0], pt[1], w))

        if not intersections:
            return None

        # Weighted centroid
        total_w = sum(w for _, _, w in intersections)
        lat = sum(la * w for la, _, w in intersections) / total_w
        lon = sum(lo * w for _, lo, w in intersections) / total_w
        avg_conf = total_w / len(intersections)

        # Spatial consistency check — reject outliers > 2× median distance
        dists = [haversine_m(lat, lon, la, lo) for la, lo, _ in intersections]
        if dists:
            median_d = sorted(dists)[len(dists) // 2]
            threshold = max(median_d * 2.0, 50.0)  # at least 50 m window
            inliers = [
                (la, lo, w) for (la, lo, w), d in zip(intersections, dists)
                if d <= threshold
            ]
            if inliers:
                total_w = sum(w for _, _, w in inliers)
                lat = sum(la * w for la, _, w in inliers) / total_w
                lon = sum(lo * w for _, lo, w in inliers) / total_w
                avg_conf = total_w / len(inliers)

        return {
            "lat": round(lat, 7),
            "lon": round(lon, 7),
            "confidence": round(avg_conf, 4),
            "n_records": len(fresh),
        }


# ──────────────────────────────────────────────
# PX4 mission builder
# ──────────────────────────────────────────────

def ft_to_m(ft: float) -> float:
    return ft * 0.3048

def build_px4_mission(
    plane_lat: float,
    plane_lon: float,
    plane_alt_ft: float,
    plane_yaw: float,
    target_lat: float,
    target_lon: float,
    approach_alt_ft: float,
    loiter_radius_ft: float,
    max_bank_deg: float,
    ground_speed_ft_s: float,
) -> dict:
    """
    Generate a minimal PX4 QGroundControl-compatible waypoint mission.

    Strategy:
      WP0 – current position (home, seq 0)
      WP1 – (optional) lead-in waypoint to align the approach heading
      WP2 – loiter above target
    """
    dist_m = haversine_m(plane_lat, plane_lon, target_lat, target_lon)
    hdg_to_target = bearing_deg(plane_lat, plane_lon, target_lat, target_lon)
    approach_alt_m = ft_to_m(approach_alt_ft)
    loiter_r_m     = ft_to_m(loiter_radius_ft)

    # Compute turn radius from bank angle: r = v² / (g * tan(φ))
    g = 9.81
    speed_ms = ft_to_m(ground_speed_ft_s)
    phi = deg2rad(max_bank_deg)
    turn_radius_m = (speed_ms ** 2) / (g * math.tan(phi)) if phi > 0 else 200.0

    items = []
    seq = 0

    def wp(seq, lat, lon, alt_m, cmd=MAV_CMD_NAV_WAYPOINT, param1=0, param2=0, param3=0, param4=0, autocontinue=1):
        return {
            "autoContinue": bool(autocontinue),
            "command": cmd,
            "doJumpId": seq + 1,
            "frame": 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            "params": [param1, param2, param3, param4, lat, lon, alt_m],
            "type": "SimpleItem",
        }

    # WP0 — home / current position (required by PX4, alt=0 means ground)
    items.append(wp(seq, plane_lat, plane_lon, 0,
                    cmd=MAV_CMD_NAV_WAYPOINT, param1=0, param2=0, param3=0, param4=math.nan))
    seq += 1

    # If far enough away, insert a lead-in waypoint 1.5× turn radius before target
    if dist_m > turn_radius_m * 3:
        lead_dist = min(turn_radius_m * 1.5, dist_m * 0.5)
        lead_lat, lead_lon = offset_latlon(
            target_lat, target_lon,
            (hdg_to_target + 180) % 360,  # approach from behind
            lead_dist,
        )
        items.append(wp(seq, lead_lat, lead_lon, approach_alt_m,
                        cmd=MAV_CMD_NAV_WAYPOINT,
                        param1=0,       # hold time
                        param2=turn_radius_m,  # acceptance radius
                        param3=0, param4=math.nan))
        seq += 1

    # Final loiter above target
    items.append(wp(seq, target_lat, target_lon, approach_alt_m,
                    cmd=MAV_CMD_NAV_LOITER_UNLIM,
                    param1=0, param2=0,
                    param3=loiter_r_m,  # positive = clockwise
                    param4=math.nan))
    seq += 1

    mission = {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "SAR-AutoHoming",
        "mission": {
            "cruiseSpeed": round(speed_ms, 2),
            "firmwareType": 12,   # PX4
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "items": items,
            "plannedHomePosition": [plane_lat, plane_lon, 0],
            "vehicleType": 1,     # Fixed-wing
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
        "_meta": {
            "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "target_lat": target_lat,
            "target_lon": target_lon,
            "dist_to_target_m": round(dist_m, 1),
            "hdg_to_target_deg": round(hdg_to_target, 1),
            "approach_alt_ft": approach_alt_ft,
            "loiter_radius_ft": loiter_radius_ft,
            "max_bank_deg": max_bank_deg,
            "turn_radius_m": round(turn_radius_m, 1),
        },
    }
    return mission


# ──────────────────────────────────────────────
# File watcher helpers
# ──────────────────────────────────────────────

def latest_jsonl(watch_dir: str) -> Optional[Path]:
    """Return the most recently named fusion_*.jsonl file."""
    pattern = os.path.join(watch_dir, "fusion_*.jsonl")
    files = glob.glob(pattern)
    if not files:
        return None
    # Timestamp is embedded in filename — lexicographic sort suffices
    return Path(sorted(files)[-1])


def tail_new_lines(path: Path, last_pos: int) -> tuple[list[str], int]:
    """Read any new lines appended since last_pos. Returns (lines, new_pos)."""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            fh.seek(last_pos)
            new_text = fh.read()
            new_pos = fh.tell()
        lines = [l.strip() for l in new_text.splitlines() if l.strip()]
        return lines, new_pos
    except OSError:
        return [], last_pos


# ──────────────────────────────────────────────
# Main loop
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="SAR Autonomous Homing — PX4 mission generator")
    parser.add_argument("--watch-dir",       default=DEFAULT_WATCH_DIR,       help="Directory containing fusion_*.jsonl files")
    parser.add_argument("--output",          default=DEFAULT_OUTPUT_FILE,     help="Output PX4 mission JSON file")
    parser.add_argument("--max-bank",        type=float, default=DEFAULT_MAX_BANK_DEG,    help="Max bank angle (degrees)")
    parser.add_argument("--min-confidence",  type=float, default=DEFAULT_MIN_CONFIDENCE,  help="Min per-record DOA confidence (0–1)")
    parser.add_argument("--min-usable",      type=int,   default=DEFAULT_MIN_USABLE,       help="Min usable records before publishing estimate")
    parser.add_argument("--loiter-radius",   type=float, default=DEFAULT_LOITER_RADIUS_FT, help="Loiter radius above target (ft)")
    parser.add_argument("--approach-alt",    type=float, default=DEFAULT_APPROACH_ALT_FT,  help="Approach / loiter altitude AGL (ft)")
    args = parser.parse_args()

    # If --output is a directory, place the default filename inside it
    output_path = Path(args.output)
    if output_path.is_dir():
        output_path = output_path / DEFAULT_OUTPUT_FILE
    args.output = str(output_path)
    # Ensure the output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)

    estimator = TargetEstimator(
        min_confidence=args.min_confidence,
        min_usable=args.min_usable,
    )

    current_file: Optional[Path] = None
    file_pos: int = 0
    last_estimate: Optional[dict] = None
    last_telemetry: Optional[dict] = None
    last_mission_hash: Optional[str] = None

    print(f"[SAR] Watching: {os.path.abspath(args.watch_dir)}")
    print(f"[SAR] Output  : {os.path.abspath(args.output)}")
    print(f"[SAR] Config  : max_bank={args.max_bank}°  min_conf={args.min_confidence}"
          f"  min_usable={args.min_usable}  loiter={args.loiter_radius}ft  approach={args.approach_alt}ft")
    print("[SAR] Running — Ctrl+C to stop\n")

    try:
        while True:
            # ── 1. Find current data file ──────────────────────────────
            new_file = latest_jsonl(args.watch_dir)
            if new_file != current_file:
                if new_file:
                    print(f"[SAR] Switched to file: {new_file.name}")
                current_file = new_file
                file_pos = 0
                estimator.buffer.clear()

            if current_file is None:
                time.sleep(POLL_INTERVAL_S)
                continue

            # ── 2. Read new lines ──────────────────────────────────────
            new_lines, file_pos = tail_new_lines(current_file, file_pos)

            ingested = 0
            for line in new_lines:
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue

                # Track latest telemetry (all records carry it)
                last_telemetry = rec

                # Feed usable records into estimator
                estimator.ingest(rec)
                if rec.get("usable_for_triangulation"):
                    ingested += 1

            if ingested:
                print(f"[SAR] +{ingested} usable records  (buffer={len(estimator.buffer)})")

            # ── 3. Estimate target ─────────────────────────────────────
            # Runs every tick — static files produce output after first read,
            # live files update as new records arrive.
            estimate = estimator.estimate()

            if estimate and estimate != last_estimate:
                last_estimate = estimate
                print(
                    f"[SAR] TARGET  lat={estimate['lat']:.6f}  lon={estimate['lon']:.6f}"
                    f"  conf={estimate['confidence']:.4f}  n={estimate['n_records']}"
                )

            # ── 4. Build & write mission ───────────────────────────────
            if estimate and last_telemetry:
                t = last_telemetry
                mission = build_px4_mission(
                    plane_lat       = t["lat_deg"],
                    plane_lon       = t["lon_deg"],
                    plane_alt_ft    = t["altitude_rel_ft"],
                    plane_yaw       = t["yaw_deg"],
                    target_lat      = estimate["lat"],
                    target_lon      = estimate["lon"],
                    approach_alt_ft = args.approach_alt,
                    loiter_radius_ft= args.loiter_radius,
                    max_bank_deg    = args.max_bank,
                    ground_speed_ft_s = t.get("ground_speed_ft_s", 60.0),
                )

                # Only rewrite if content changed (avoid unnecessary I/O)
                mission_str = json.dumps(mission, indent=2, allow_nan=True)
                mission_hash = str(hash(mission_str))
                if mission_hash != last_mission_hash:
                    out_p = Path(args.output)
                    tmp_path = str(out_p.parent / (out_p.stem + ".tmp" + out_p.suffix))
                    with open(tmp_path, "w", encoding="utf-8") as fh:
                        fh.write(mission_str)
                    os.replace(tmp_path, args.output)  # atomic write
                    last_mission_hash = mission_hash
                    meta = mission["_meta"]
                    print(
                        f"[SAR] MISSION  dist={meta['dist_to_target_m']}m"
                        f"  hdg={meta['hdg_to_target_deg']}°"
                        f"  turn_r={meta['turn_radius_m']}m  → {args.output}"
                    )

            time.sleep(POLL_INTERVAL_S)

    except KeyboardInterrupt:
        print("\n[SAR] Stopped.")
        sys.exit(0)


if __name__ == "__main__":
    main()

# import json

# # Right now, this only processes the kraken log data
# # outputs the epoch with the max confidence as well as the lat and long
# # TODO: implement the fusion data

# def log_generator(file_path):
#     with open(file_path, 'r') as f:
#         for line in f:
#             yield json.loads(line)

# # log file path; modify ts to py's file path location
# file_path = '/content/doa_log.jsonl'

# # track max confidence
# max_record = None

# try:
#     for record in log_generator(file_path):
#         if max_record is None or record.get('confidence', 0) > max_record.get('confidence', 0):
#             max_record = record

#     if max_record:
#         epoch = max_record.get('epoch')
#         conf = max_record.get('confidence')
#         lat = max_record.get('latitude')
#         lon = max_record.get('longitude')
#         print(f"At epoch {epoch}, Highest confidence: {conf}, latitude: {lat}, longitude: {lon}")
#     else:
#         print("No records found in the log file.")
# except FileNotFoundError:
#     print(f"Error: The file {file_path} was not found.")
# except Exception as e:
#     print(f"An error occurred: {e}")