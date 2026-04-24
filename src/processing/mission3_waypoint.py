"""
mission3_waypoint.py — Real-time flight path waypoint system for fixed-wing UAV.
Mission 3: Small Loiter Pattern
"""

import json
import math
import os
import time
import zlib
import struct

# ─────────────────────────── CONFIGURABLE PARAMETERS ────────────────────────
LOITER_RADIUS_FT    = 20      # Loiter circle radius written into active_plan.loiter_radius_ft (ft)
BORDER_CLEARANCE_FT = 50.0    # Circle must be at least this far from search border (ft)
LOITER_SAMPLE_PTS   = 30      # Number of points sampled around circle circumference for checks
UPDATE_INTERVAL_S   = 0.1     # Guidance loop polling interval (s)
GENERATE_IMAGE      = True    # Toggle PNG generation
# ─────────────────────────────────────────────────────────────────────────────

# ── File paths ────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_STATE_DIR         = os.path.join(_HERE, "..", "state")
NAVIGATION_STATE   = os.path.join(_STATE_DIR, "navigation_state.json")
MISSION_STATE      = os.path.join(_STATE_DIR, "mission_state.json")
PNG_OUTPUT         = os.path.join(_HERE, "mission3_map.png")

# ── Geo helpers ───────────────────────────────────────────────────────────────
FEET_PER_METER  = 3.28084
EARTH_RADIUS_FT = 20_902_231.0   # Earth radius in feet


def _ft_to_deg_lat(ft: float) -> float:
    """Convert feet to degrees of latitude (constant everywhere)."""
    return ft / EARTH_RADIUS_FT * (180.0 / math.pi)


def _ft_to_deg_lon(ft: float, lat_deg: float) -> float:
    """Convert feet to degrees of longitude at a given latitude."""
    r = EARTH_RADIUS_FT * math.cos(math.radians(lat_deg))
    return ft / r * (180.0 / math.pi)


def _haversine_ft(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in feet between two lat/lon points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * EARTH_RADIUS_FT * math.asin(math.sqrt(a))


def _circle_sample_pts(clat, clon, radius_ft, n=LOITER_SAMPLE_PTS):
    """Return *n* (lat, lon) points evenly spaced around the circle circumference."""
    pts = []
    for i in range(n):
        bearing = 2 * math.pi * i / n
        dlat = _ft_to_deg_lat(radius_ft * math.cos(bearing))
        dlon = _ft_to_deg_lon(radius_ft * math.sin(bearing), clat)
        pts.append((clat + dlat, clon + dlon))
    return pts


# ── Polygon helpers ───────────────────────────────────────────────────────────

def _point_in_polygon(lat, lon, poly) -> bool:
    """Ray-casting test; poly is a list of (lat, lon) tuples (closed or open)."""
    n   = len(poly)
    ins = False
    j   = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > lon) != (yj > lon)) and (lat < (xj - xi) * (lon - yi) / (yj - yi + 1e-15) + xi):
            ins = not ins
        j = i
    return ins


def _segment_to_point_dist_ft(px, py, ax, ay, bx, by) -> float:
    """Minimum distance (in flat-earth feet) from point P to segment AB."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return _haversine_ft(px, py, ax, ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    nx, ny = ax + t * dx, ay + t * dy
    return _haversine_ft(px, py, nx, ny)


def _min_dist_circle_to_boundary_ft(clat, clon, radius_ft, poly) -> float:
    """
    Minimum distance from any point on the circle circumference to any polygon
    edge.  Returns a *signed* value: positive = inside, but we only call this
    when we already know the center is inside, so it is always positive.
    """
    sample_pts = _circle_sample_pts(clat, clon, radius_ft)
    n = len(poly)
    min_d = math.inf
    for (slat, slon) in sample_pts:
        for i in range(n):
            j = (i + 1) % n
            d = _segment_to_point_dist_ft(slat, slon,
                                          poly[i][0], poly[i][1],
                                          poly[j][0], poly[j][1])
            if d < min_d:
                min_d = d
    return min_d


def _closest_valid_center(orig_lat, orig_lon, radius_ft, poly,
                           clearance_ft=BORDER_CLEARANCE_FT) -> tuple:
    """
    If the loiter circle (centered at orig_lat/lon) violates the border
    clearance, nudge the center toward the polygon centroid as little as
    possible until the constraint is met.

    Strategy: binary-search along the vector from original center → polygon
    centroid.
    """
    # Compute polygon centroid
    clat = sum(p[0] for p in poly) / len(poly)
    clon = sum(p[1] for p in poly) / len(poly)

    # If the centroid itself doesn't satisfy, we do our best (shouldn't happen
    # with a well-formed search area larger than the circle).
    def _ok(lat, lon):
        if not _point_in_polygon(lat, lon, poly):
            return False
        # All circumference points must be inside with clearance
        for slat, slon in _circle_sample_pts(lat, lon, radius_ft):
            if not _point_in_polygon(slat, slon, poly):
                return False
        return _min_dist_circle_to_boundary_ft(lat, lon, radius_ft, poly) >= clearance_ft

    if _ok(orig_lat, orig_lon):
        return orig_lat, orig_lon

    # Binary search between orig and centroid
    lo, hi = 0.0, 1.0
    for _ in range(40):        # ~40 iterations → nanometer accuracy
        mid = (lo + hi) / 2
        lat = orig_lat + mid * (clat - orig_lat)
        lon = orig_lon + mid * (clon - orig_lon)
        if _ok(lat, lon):
            hi = mid
        else:
            lo = mid

    lat = orig_lat + hi * (clat - orig_lat)
    lon = orig_lon + hi * (clon - orig_lon)
    return lat, lon


# ── JSON I/O ──────────────────────────────────────────────────────────────────

def _load(path: str) -> dict:
    with open(path, "r") as f:
        return json.load(f)


def _save(path: str, data: dict):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


# ── PNG generation (stdlib only: zlib + struct) ───────────────────────────────

def _png_bytes(width: int, height: int, buf: bytearray) -> bytes:
    """Encode an RGB bytearray pixel buffer as a PNG byte string."""

    def _chunk(tag: bytes, data: bytes) -> bytes:
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    # Add filter byte (0 = None) at the start of every row
    raw_rows = bytearray()
    row_bytes = width * 3
    for y in range(height):
        raw_rows.append(0)
        raw_rows += buf[y * row_bytes: (y + 1) * row_bytes]

    ihdr  = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    idat  = zlib.compress(bytes(raw_rows), 6)

    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", idat)
        + _chunk(b"IEND", b"")
    )


def _set_pixel(buf: bytearray, W: int, x: int, y: int, rgb: tuple):
    if 0 <= x < W and 0 <= y < W:
        idx = (y * W + x) * 3
        buf[idx], buf[idx + 1], buf[idx + 2] = rgb


def _draw_circle_outline(buf, W, cx, cy, r, rgb, dashed=False, dash=6):
    pts = 360 * max(1, r // 4)
    for i in range(pts):
        if dashed and (i // dash) % 2:
            continue
        a = 2 * math.pi * i / pts
        x = int(cx + r * math.cos(a))
        y = int(cy + r * math.sin(a))
        _set_pixel(buf, W, x, y, rgb)


def _draw_dot(buf, W, cx, cy, r, rgb):
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                _set_pixel(buf, W, cx + dx, cy + dy, rgb)


def _draw_line(buf, W, x0, y0, x1, y1, rgb):
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        _set_pixel(buf, W, x0, y0, rgb)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x0 += sx
        if e2 < dx:
            err += dx; y0 += sy


def _draw_polygon(buf, W, screen_pts, rgb):
    n = len(screen_pts)
    for i in range(n):
        x0, y0 = screen_pts[i]
        x1, y1 = screen_pts[(i + 1) % n]
        _draw_line(buf, W, x0, y0, x1, y1, rgb)


def _draw_arrow(buf, W, sx, sy, ex, ey, rgb, head=10):
    _draw_line(buf, W, sx, sy, ex, ey, rgb)
    angle = math.atan2(ey - sy, ex - sx)
    for side in (+0.4, -0.4):
        ax = int(ex - head * math.cos(angle - side))
        ay = int(ey - head * math.sin(angle - side))
        _draw_line(buf, W, ex, ey, ax, ay, rgb)


def generate_png(poly, orig_clat, orig_clon, final_clat, final_clon,
                 radius_ft, plane_lat, plane_lon, plane_hdg,
                 target_lat, target_lon):
    """Render the mission map and save to PNG_OUTPUT."""

    W = 800
    buf = bytearray(W * W * 3)   # black background

    # Determine bounding box from polygon
    all_lats = [p[0] for p in poly]
    all_lons = [p[1] for p in poly]
    margin = 0.0002
    min_lat = min(all_lats) - margin
    max_lat = max(all_lats) + margin
    min_lon = min(all_lons) - margin
    max_lon = max(all_lons) + margin

    def to_screen(lat, lon):
        x = int((lon - min_lon) / (max_lon - min_lon) * (W - 1))
        y = int((max_lat - lat) / (max_lat - min_lat) * (W - 1))
        return x, y

    def r_to_px(r_ft):
        span_lat_ft = _haversine_ft(min_lat, min_lon, max_lat, min_lon)
        return int(r_ft / span_lat_ft * W)

    # Draw yellow search area border
    spoly = [to_screen(lat, lon) for lat, lon in poly]
    _draw_polygon(buf, W, spoly, (255, 220, 0))

    # Draw dashed grey raw circle
    ocx, ocy = to_screen(orig_clat, orig_clon)
    opr = r_to_px(radius_ft)
    _draw_circle_outline(buf, W, ocx, ocy, opr, (140, 140, 140), dashed=True)

    # Draw solid white final/shifted circle
    fcx, fcy = to_screen(final_clat, final_clon)
    fpr = r_to_px(radius_ft)
    _draw_circle_outline(buf, W, fcx, fcy, fpr, (255, 255, 255))

    # Cyan dot for target location
    tx, ty = to_screen(target_lat, target_lon)
    _draw_dot(buf, W, tx, ty, 5, (0, 220, 220))

    # White dot for final circle center
    _draw_dot(buf, W, fcx, fcy, 4, (255, 255, 255))

    # Green dot + arrow for plane
    px, py = to_screen(plane_lat, plane_lon)
    _draw_dot(buf, W, px, py, 5, (0, 220, 0))
    arrow_len = 25
    hdg_rad = math.radians(plane_hdg)
    ex = int(px + arrow_len * math.sin(hdg_rad))
    ey = int(py - arrow_len * math.cos(hdg_rad))
    _draw_arrow(buf, W, px, py, ex, ey, (0, 220, 0))

    with open(PNG_OUTPUT, "wb") as f:
        f.write(_png_bytes(W, W, buf))
    print(f"[mission3] Map saved → {PNG_OUTPUT}")


# ── Mission startup ───────────────────────────────────────────────────────────

def _startup(nav: dict, mis: dict):
    """Apply mission-3 state to both JSON blobs (in-place) and save."""

    # mission_state.json active plan fields
    ap = mis.setdefault("active_plan", {})
    ap["mission_phase"] = 3
    ap["plan_id"]       = 3
    ap["plan_type"]     = "Small Loiter Pattern"
    ap["status"]        = "Loitering"
    ap["next_plan"]     = "Manual Takeover"

    # Read telemetry from fusion_log
    fusion = mis.get("fusion_log", {})
    plane_lat = fusion.get("lat", 0.0)
    plane_lon = fusion.get("lon", 0.0)
    plane_hdg = fusion.get("heading_deg", 0.0)

    # Read search area from navigation_state
    sa_raw = nav.get("search_area", [])
    poly = [(pt["lat"], pt["lon"]) for pt in sa_raw]   # exactly 6 coords

    # Read target location
    tgt = nav.get("target_location", {})
    target_lat = tgt.get("lat", 0.0)
    target_lon = tgt.get("lon", 0.0)

    # Altitude from navigation_state active_plan
    alt_ft = nav.get("active_plan", {}).get("alt_ft", 100)

    # Compute validated circle center
    orig_lat, orig_lon = target_lat, target_lon
    final_lat, final_lon = _closest_valid_center(
        orig_lat, orig_lon, LOITER_RADIUS_FT, poly, BORDER_CLEARANCE_FT
    )

    # Update navigation_state active_plan
    nav_ap = nav.setdefault("active_plan", {})
    nav_ap["plan_id"]         = 3
    nav_ap["plan_type"]       = "Small Loiter Pattern"
    nav_ap["status"]          = "Loitering"
    nav_ap["loiter_radius_ft"] = LOITER_RADIUS_FT
    nav_ap["alt_ft"]          = alt_ft
    nav_ap["next_plan"]       = "Manual Takeover"
    nav_ap["waypoints"]       = [{"lat": final_lat, "lon": final_lon, "alt_ft": alt_ft}]

    _save(MISSION_STATE,    mis)
    _save(NAVIGATION_STATE, nav)

    print(f"[mission3] Startup complete. Circle center: ({final_lat:.6f}, {final_lon:.6f})")
    if (final_lat, final_lon) != (orig_lat, orig_lon):
        shift_ft = _haversine_ft(orig_lat, orig_lon, final_lat, final_lon)
        print(f"[mission3] Circle shifted {shift_ft:.1f} ft to satisfy clearance.")

    # Optional PNG
    if GENERATE_IMAGE:
        generate_png(poly, orig_lat, orig_lon, final_lat, final_lon,
                     LOITER_RADIUS_FT, plane_lat, plane_lon, plane_hdg,
                     target_lat, target_lon)

    return mis, nav


# ── Mission teardown ──────────────────────────────────────────────────────────

def _teardown(nav: dict, mis: dict):
    """Clear loiter-related fields on autonomy deactivation."""

    # navigation_state.json → mra_refined_loiter_target
    rlt = nav.setdefault("mra_refined_loiter_target", {})
    rlt["valid"]      = False
    rlt["lat"]        = None
    rlt["lon"]        = None
    rlt["confidence"] = None
    rlt["timestamp"]  = None
    rlt["fix_id"]     = None

    # navigation_state.json → target_location
    tl = nav.setdefault("target_location", {})
    tl["valid"]      = False
    tl["source"]     = None
    tl["lat"]        = None
    tl["lon"]        = None
    tl["confidence"] = None
    tl["timestamp"]  = None
    tl["id"]         = None

    # navigation_state.json → active_plan
    ap = nav.setdefault("active_plan", {})
    ap["plan_id"]          = None
    ap["plan_type"]        = None
    ap["status"]           = None
    ap["loiter_radius_ft"] = None
    ap["alt_ft"]           = None
    ap["waypoints"]        = None
    ap["next_plan"]        = None

    # navigation_state.json → top-level next_plan
    nav["next_plan"] = None

    _save(NAVIGATION_STATE, nav)
    print("[mission3] Teardown complete. All loiter fields cleared.")


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    print("[mission3] Loading state files …")
    nav = _load(NAVIGATION_STATE)
    mis = _load(MISSION_STATE)

    mis, nav = _startup(nav, mis)

    print("[mission3] Entering guidance loop (polling autonomy_active) …")
    while True:
        time.sleep(UPDATE_INTERVAL_S)
        try:
            mis = _load(MISSION_STATE)
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[mission3] WARN: could not read mission_state ({exc}); retrying …")
            continue

        if not mis.get("autonomy_active", True):
            print("[mission3] autonomy_active → False. Initiating teardown …")
            nav = _load(NAVIGATION_STATE)
            _teardown(nav, mis)
            break

    print("[mission3] Exited.")


if __name__ == "__main__":
    main()