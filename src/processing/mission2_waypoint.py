import sys
import json
import math
import time
import subprocess
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.mission_state_utils import load_state, update_state
from state.nav_state_utils import load_nav_state as load_state_nav, update_nav_state as update_state_nav

###############################################################################
# MISSION 2 CONFIGURATION
###############################################################################
LOITER_RADIUS_FT    = 250.0  # Loiter circle radius written into active_plan.loiter_radius_ft (ft)
BORDER_CLEARANCE_M  = 30.0   # Minimum distance circle edge must keep from search boundary (m)
LOITER_SAMPLE_PTS   = 72     # Number of points sampled around circle circumference for checks
UPDATE_INTERVAL_S   = 0.5    # Polling interval when waiting for target_location.valid (s)

EARTH_RADIUS_M = 6_371_000.0


###############################################################################
# Geodetic helpers (same as mission1)
###############################################################################

def haversine(lat1, lon1, lat2, lon2):
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = rlat2 - rlat1, rlon2 - rlon1
    a = math.sin(dlat/2)**2 + math.cos(rlat1)*math.cos(rlat2)*math.sin(dlon/2)**2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(max(0.0, min(1.0, a))))


def destination(lat, lon, brng_deg, dist_m):
    br    = math.radians(brng_deg)
    rlat  = math.radians(lat)
    rlon  = math.radians(lon)
    dr    = dist_m / EARTH_RADIUS_M
    nlat  = math.asin(math.sin(rlat)*math.cos(dr) + math.cos(rlat)*math.sin(dr)*math.cos(br))
    nlon  = rlon + math.atan2(math.sin(br)*math.sin(dr)*math.cos(rlat),
                              math.cos(dr) - math.sin(rlat)*math.sin(nlat))
    return math.degrees(nlat), math.degrees(nlon)


def to_local(lat, lon, o_lat, o_lon):
    x = haversine(o_lat, o_lon, o_lat, lon) * (1 if lon >= o_lon else -1)
    y = haversine(o_lat, o_lon, lat,   o_lon) * (1 if lat >= o_lat else -1)
    return x, y


def from_local(x, y, o_lat, o_lon):
    lat, _ = destination(o_lat, o_lon,  0, y)
    _, lon  = destination(o_lat, o_lon, 90, x)
    return lat, lon


def centroid(pts):
    n = len(pts)
    return sum(p[0] for p in pts)/n, sum(p[1] for p in pts)/n


def sort_clockwise(pts):
    cx, cy = centroid(pts)
    return sorted(pts, key=lambda p: -math.atan2(p[1]-cy, p[0]-cx))


###############################################################################
# 2-D geometry helpers (local metre frame)
###############################################################################

def pt_to_seg_dist(px, py, ax, ay, bx, by):
    """Shortest distance from point P to segment AB."""
    dx, dy = bx - ax, by - ay
    sq = dx*dx + dy*dy
    if sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / sq))
    return math.hypot(px - (ax + t*dx), py - (ay + t*dy))


def min_dist_point_to_poly(px, py, poly):
    """Minimum distance from a point to any edge of a closed polygon."""
    n = len(poly)
    return min(
        pt_to_seg_dist(px, py,
                       poly[i][0],        poly[i][1],
                       poly[(i+1)%n][0],  poly[(i+1)%n][1])
        for i in range(n)
    )


def point_in_poly(px, py, poly):
    """Ray-casting point-in-polygon test."""
    n      = len(poly)
    inside = False
    j      = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > py) != (yj > py)) and (px < (xj-xi)*(py-yi)/(yj-yi+1e-12)+xi):
            inside = not inside
        j = i
    return inside


def circle_clearance(cx, cy, radius_m, poly, n_samples=72):
    """
    Minimum clearance between a circle and all polygon edges.
    Samples n_samples points evenly around the circumference and returns
    the minimum distance from any sample point to any polygon edge.
    Positive → all samples inside with that margin; negative → breach.
    """
    min_d = float("inf")
    for i in range(n_samples):
        angle = 2 * math.pi * i / n_samples
        sx = cx + radius_m * math.cos(angle)
        sy = cy + radius_m * math.sin(angle)
        min_d = min(min_d, min_dist_point_to_poly(sx, sy, poly))
    return min_d


###############################################################################
# Circle containment / nudging
###############################################################################

def ft_to_m(ft):
    return ft * 0.3048


def clamp_circle_to_poly(cx, cy, radius_m, poly, clearance_m):
    """
    If the circle (cx,cy,radius_m) already satisfies clearance_m from every
    poly edge, return it unchanged.

    Otherwise nudge the centre toward the polygon centroid along the ray
    centroid→(cx,cy) until clearance is satisfied, using binary search.
    The result minimises displacement from the original centre while
    guaranteeing the constraint.

    Returns
    -------
    (new_cx, new_cy, was_moved)
    """
    required = radius_m + clearance_m

    # Fast path — already compliant (all circumference samples clear the boundary)
    if circle_clearance(cx, cy, radius_m, poly, LOITER_SAMPLE_PTS) >= clearance_m:
        return cx, cy, False

    pcx, pcy = centroid(poly)

    # Direction: from original centre toward polygon centroid
    dx = pcx - cx
    dy = pcy - cy
    dist_to_centroid = math.hypot(dx, dy)

    if dist_to_centroid < 1e-6:
        # Already at centroid — cannot move further; return centroid
        return pcx, pcy, True

    # Binary-search along the segment  original_centre → poly_centroid
    # for the smallest t that satisfies the clearance constraint.
    # t=0 → original (fails), t=1 → centroid (safe by design if radius < inradius)
    lo, hi = 0.0, 1.0

    for _ in range(64):
        mid  = (lo + hi) * 0.5
        qx   = cx + mid * dx
        qy   = cy + mid * dy
        if circle_clearance(qx, qy, radius_m, poly, LOITER_SAMPLE_PTS) >= clearance_m:
            hi = mid   # good — try pulling back toward original
        else:
            lo = mid   # still bad — push more toward centroid

    new_cx = cx + hi * dx
    new_cy = cy + hi * dy
    return new_cx, new_cy, True


###############################################################################
# Telemetry reader (same tail-seek approach as mission1)
###############################################################################

def read_latest_telemetry(fusion_log_path):
    path = Path(fusion_log_path)
    if not path.exists():
        print(f"[WARN] Fusion log not found: {path}")
        return None
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            if size == 0:
                return None
            buf, pos = b"", size - 1
            while pos >= 0:
                f.seek(pos)
                ch = f.read(1)
                if ch == b"\n" and buf.strip():
                    break
                buf = ch + buf
                pos -= 1
            line = buf.decode("utf-8", errors="replace").strip()
            if line:
                return json.loads(line)
    except Exception as e:
        print(f"[WARN] Could not read fusion log: {e}")
    return None


###############################################################################
# Main
###############################################################################

def run_mission_2():
    print("[MISSION 2] Starting …")

    # ------------------------------------------------------------------
    # Load state files
    # ------------------------------------------------------------------
    mission_state = load_state()
    nav_state     = load_state_nav()

    fusion_log_path = mission_state.get("fusion_log", "")

    # ------------------------------------------------------------------
    # Startup: write plan metadata
    # ------------------------------------------------------------------
    nav = nav_state.get("navigation", {})
    nav["mission_phase"] = 2
    update_state_nav("navigation", nav)

    active_plan = nav_state.get("active_plan", {})
    active_plan["plan_id"]   = 2
    active_plan["plan_type"] = "Big Loiter Pattern"
    active_plan["status"]    = "searching"
    update_state_nav("active_plan", active_plan)

    update_state_nav("next_plan", 3)

    print("[STATE] mission_phase=2, plan_id=2, plan_type='Big Loiter Pattern', "
          "status='searching', next_plan=3")

    # ------------------------------------------------------------------
    # Read search area
    # ------------------------------------------------------------------
    nav_state  = load_state_nav()
    raw_search = nav_state.get("search_area", [])
    if len(raw_search) != 4:
        raise ValueError(f"search_area must have 4 entries, got {len(raw_search)}")

    search_coords = []
    for entry in raw_search:
        if not isinstance(entry, (list, tuple)) or len(entry) < 2:
            raise ValueError(f"Invalid search_area entry: {entry}")
        search_coords.append((float(entry[0]), float(entry[1])))

    # Origin for local-metre frame
    o_lat = sum(p[0] for p in search_coords) / 4
    o_lon = sum(p[1] for p in search_coords) / 4

    local_pts = [to_local(lat, lon, o_lat, o_lon) for lat, lon in search_coords]
    search_xy = sort_clockwise(local_pts)   # closed convex quad in local metres

    # ------------------------------------------------------------------
    # Read loiter target
    # ------------------------------------------------------------------
    loiter_target = nav_state.get("mra_refined_loiter_target", {})
    if not loiter_target.get("valid", False):
        print("[WARN] mra_refined_loiter_target.valid is False — "
              "proceeding with whatever lat/lon is present.")

    target_lat = loiter_target.get("lat")
    target_lon = loiter_target.get("lon")
    if target_lat is None or target_lon is None:
        raise ValueError("mra_refined_loiter_target lat/lon are None — cannot build loiter.")

    # ------------------------------------------------------------------
    # Read loiter radius (feet — kept as-is, no unit conversion)
    # ------------------------------------------------------------------
    active_plan = nav_state.get("active_plan", {})
    active_plan["loiter_radius_ft"] = LOITER_RADIUS_FT
    update_state_nav("active_plan", active_plan)
    print(f"[STATE] active_plan.loiter_radius_ft → {LOITER_RADIUS_FT} ft")
    loiter_radius_ft = LOITER_RADIUS_FT

    # Radius in metres for geometric calculations
    loiter_radius_m = ft_to_m(float(loiter_radius_ft))

    # ------------------------------------------------------------------
    # Read altitude (feet — kept as-is)
    # ------------------------------------------------------------------
    alt_ft = nav_state.get("alt_ft")
    if alt_ft is None:
        raise ValueError("alt_ft is None in navigation_state.json.")

    # ------------------------------------------------------------------
    # Convert loiter centre to local frame and clamp to search area
    # ------------------------------------------------------------------
    orig_cx, orig_cy = to_local(target_lat, target_lon, o_lat, o_lon)

    new_cx, new_cy, was_moved = clamp_circle_to_poly(
        orig_cx, orig_cy, loiter_radius_m, search_xy, BORDER_CLEARANCE_M
    )

    if was_moved:
        new_lat, new_lon = from_local(new_cx, new_cy, o_lat, o_lon)
        move_m = math.hypot(new_cx - orig_cx, new_cy - orig_cy)
        print(f"[LOITER] Circle nudged {move_m:.1f} m to stay within boundary clearance.")
        print(f"[LOITER] Original centre: ({target_lat:.7f}, {target_lon:.7f})")
        print(f"[LOITER] Adjusted centre: ({new_lat:.7f}, {new_lon:.7f})")
    else:
        new_lat, new_lon = target_lat, target_lon
        print(f"[LOITER] Circle already within boundary — centre unchanged "
              f"({new_lat:.7f}, {new_lon:.7f})")

    print(f"[LOITER] Radius: {loiter_radius_ft} ft  |  alt: {alt_ft} ft")

    # ------------------------------------------------------------------
    # Write waypoint (single centre point) to active_plan.waypoints
    # ------------------------------------------------------------------
    active_plan = load_state_nav().get("active_plan", {})
    active_plan["waypoints"] = [
        {"lat": new_lat, "lon": new_lon, "alt_ft": alt_ft}
    ]
    update_state_nav("active_plan", active_plan)
    print("[STATE] active_plan.waypoints written (loiter centre).")

    # ------------------------------------------------------------------
    # Guidance loop — poll until target_location.valid becomes True
    # ------------------------------------------------------------------
    print("[MISSION 2] Loitering … waiting for target_location.valid …")

    telem = read_latest_telemetry(fusion_log_path)
    if telem:
        print(f"[TELEM] Plane at ({telem.get('lat_deg'):.6f}, "
              f"{telem.get('lon_deg'):.6f}), yaw={telem.get('yaw_deg', 0):.1f}°")

    try:
        while True:
            nav_state      = load_state_nav()
            target_location = nav_state.get("target_location", {})

            if target_location.get("valid", False):
                print("[MISSION 2] target_location.valid = True — exiting loiter.")
                break

            time.sleep(UPDATE_INTERVAL_S)

    except KeyboardInterrupt:
        print("\n[MISSION 2] Stopped by user.")

    # ------------------------------------------------------------------
    # Mission end: mark status complete, hand off to mission 3
    # ------------------------------------------------------------------
    nav_state   = load_state_nav()
    active_plan = nav_state.get("active_plan", {})
    active_plan["status"] = "complete"
    update_state_nav("active_plan", active_plan)
    print("[STATE] active_plan.status → 'complete'")

    print("[MISSION 2] Done. Launching mission3_waypoint.py …")
    mission3_path = Path(__file__).resolve().parent / "mission3_waypoint.py"
    if mission3_path.is_file():
        subprocess.Popen([sys.executable, str(mission3_path)])
        print(f"[HANDOFF] Launched {mission3_path}")
    else:
        print(f"[WARN] mission3_waypoint.py not found at {mission3_path} — skipping handoff.")


if __name__ == "__main__":
    run_mission_2()