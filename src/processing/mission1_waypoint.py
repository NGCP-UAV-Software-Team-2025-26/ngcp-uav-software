import sys
import json
import math
import time
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.state_utils import load_state, update_state, STATE_FILE

# MISSION 1 CONFIGURATION — modify these values to tune behaviour
MAX_TURN_ANGLE_DEG  = 35.0   # Maximum acceptable turn angle at each corner (deg)
WAYPOINT_RADIUS_M   = 20.0   # Capture / acceptance radius around each waypoint (m)
BORDER_SHRINK_M     = 200.0   # Distance the diamond keeps from the search-area border (m)
L1_DISTANCE_M       = 30.0   # L1 guidance lookahead distance (m)
UPDATE_INTERVAL_S   = 0.1    # Main-loop polling interval (seconds)
GENERATE_IMAGE      = True   # Set False to skip PNG generation entirely
# ----------------------------------------------------------------------------------------

EARTH_RADIUS_M = 6_371_000.0


# Geodetic helpers 

def haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres."""
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = (math.sin(dlat / 2) ** 2
         + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing in degrees [0, 360) from point-1 to point-2."""
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = (math.cos(rlat1) * math.sin(rlat2)
         - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def destination(lat, lon, brng_deg, dist_m):
    """(lat, lon) after travelling dist_m along brng_deg from (lat, lon)."""
    br = math.radians(brng_deg)
    rlat, rlon = math.radians(lat), math.radians(lon)
    d_r = dist_m / EARTH_RADIUS_M
    nlat = math.asin(
        math.sin(rlat) * math.cos(d_r)
        + math.cos(rlat) * math.sin(d_r) * math.cos(br)
    )
    nlon = rlon + math.atan2(
        math.sin(br) * math.sin(d_r) * math.cos(rlat),
        math.cos(d_r) - math.sin(rlat) * math.sin(nlat),
    )
    return math.degrees(nlat), math.degrees(nlon)


def latlon_to_local(lat, lon, o_lat, o_lon):
    """(lat, lon) → local (east_m, north_m) relative to origin."""
    x = haversine(o_lat, o_lon, o_lat, lon) * (1 if lon >= o_lon else -1)
    y = haversine(o_lat, o_lon, lat,   o_lon) * (1 if lat >= o_lat else -1)
    return x, y


def local_to_latlon(x, y, o_lat, o_lon):
    """Local (east_m, north_m) → (lat, lon)."""
    lat, _ = destination(o_lat, o_lon, 0,  y)
    _, lon = destination(o_lat, o_lon, 90, x)
    return lat, lon


def poly_centroid(pts):
    n = len(pts)
    return sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n


def shrink_toward(cx, cy, px, py, dist_m):
    """Move (px,py) towards centroid (cx,cy) by dist_m."""
    dx, dy = px - cx, py - cy
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return px, py
    factor = max(0.0, (length - dist_m) / length)
    return cx + dx * factor, cy + dy * factor


def midpoint(a, b):
    return (a[0] + b[0]) / 2, (a[1] + b[1]) / 2


def normalize_angle(a):
    """Normalise to (-180, 180]."""
    while a > 180:
        a -= 360
    while a <= -180:
        a += 360
    return a


# --------------------------------------------
# Diamond + rounded-corner waypoint generation
# --------------------------------------------

def build_diamond_waypoints(search_coords):
    """
    Returns
    -------
    waypoints       : list of (lat, lon)  — one per rounded corner, clockwise
    search_local    : list of (x, y) m   — search area corners in local frame
    diamond_local   : list of (x, y) m   — raw (shrunk) diamond vertices
    waypoints_local : list of (x, y) m   — waypoints in local frame
    origin_lat/lon  : float              — local coordinate origin
    """
    # Local-frame origin = centroid of the 4 search corners
    o_lat = sum(p[0] for p in search_coords) / 4
    o_lon = sum(p[1] for p in search_coords) / 4

    # Convert search area to local coords ; sort clockwise
    local_corners = [latlon_to_local(lat, lon, o_lat, o_lon)
                     for lat, lon in search_coords]
    cx, cy = poly_centroid(local_corners)
    local_corners.sort(key=lambda p: -math.atan2(p[1] - cy, p[0] - cx))

    # Edge mid-points -> raw diamond
    n = len(local_corners)
    diamond = [midpoint(local_corners[i], local_corners[(i + 1) % n])
               for i in range(n)]

    # Shrink each diamond vertex away from the search-area boundary
    dc_x, dc_y = poly_centroid(diamond)
    diamond = [shrink_toward(dc_x, dc_y, px, py, BORDER_SHRINK_M)
               for px, py in diamond]

    # Build rounded-corner waypoints
    # For corners tighter than MAX_TURN_ANGLE_DEG we place the waypoint
    # slightly before the vertex on the incoming leg so the plane eases through
    waypoints_local = []
    nd = len(diamond)
    for i in range(nd):
        prev_pt = diamond[(i - 1) % nd]
        curr_pt = diamond[i]
        next_pt = diamond[(i + 1) % nd]

        in_dx  = curr_pt[0] - prev_pt[0]
        in_dy  = curr_pt[1] - prev_pt[1]
        out_dx = next_pt[0] - curr_pt[0]
        out_dy = next_pt[1] - curr_pt[1]

        in_ang  = math.degrees(math.atan2(in_dy,  in_dx))
        out_ang = math.degrees(math.atan2(out_dy, out_dx))
        turn    = normalize_angle(out_ang - in_ang)

        if abs(turn) <= MAX_TURN_ANGLE_DEG:
            waypoints_local.append(curr_pt)
        else:
            # Set the waypoint back from the vertex so approach heading fits
            half_turn_rad = math.radians(abs(turn) / 2)
            set_back = L1_DISTANCE_M * math.tan(half_turn_rad)
            in_len = math.hypot(in_dx, in_dy)
            if in_len < 1e-9:
                waypoints_local.append(curr_pt)
                continue
            ratio = max(0.0, min(1.0, set_back / in_len))
            wp_x = curr_pt[0] - in_dx * ratio
            wp_y = curr_pt[1] - in_dy * ratio
            waypoints_local.append((wp_x, wp_y))

    waypoints = [local_to_latlon(px, py, o_lat, o_lon)
                 for px, py in waypoints_local]

    return waypoints, local_corners, diamond, waypoints_local, o_lat, o_lon


# -------------------------------------------------
# Fusion-log reader ; read last entry of JSONL file
# -------------------------------------------------

def read_latest_telemetry(fusion_log_path):
    """Return the most-recent telemetry dict from the fusion JSONL log."""
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


# --------------------------
# Closest waypoint selection
# --------------------------

def closest_waypoint_index(plane_lat, plane_lon, waypoints):
    best_i, best_d = 0, float("inf")
    for i, (wlat, wlon) in enumerate(waypoints):
        d = haversine(plane_lat, plane_lon, wlat, wlon)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


# ------------
# L1 Guidance ######################################## ( might not be necessary, will probably remove)
# ------------

def l1_guidance(plane_lat, plane_lon, plane_yaw, wp_lat, wp_lon):
    """
    Simplified L1 guidance.

    Returns
    -------
    guidance_heading : float   corrected heading command (degrees)
    desired_bearing  : float   direct bearing to waypoint (degrees)
    dist_m           : float   distance to waypoint (metres)
    """
    dist_m          = haversine(plane_lat, plane_lon, wp_lat, wp_lon)
    desired_bearing = bearing_deg(plane_lat, plane_lon, wp_lat, wp_lon)
    cross_track_err = normalize_angle(desired_bearing - plane_yaw)
    eta             = math.radians(cross_track_err)
    correction      = math.degrees(
        math.asin(max(-1.0, min(1.0,
            2.0 * math.sin(eta) * dist_m / max(dist_m, L1_DISTANCE_M))))
    )
    return (desired_bearing + correction) % 360, desired_bearing, dist_m


# --------------------
# PNG image generation
# --------------------

def _write_png(filename, width, height, pixels, zlib, struct):
    """Write a 24-bit RGB PNG using only zlib and struct."""

    def chunk(tag, data):
        payload = tag + data
        return (struct.pack(">I", len(data))
                + payload
                + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF))
    
    print("right before row iteration")

    raw = b""
    for row in range(height):
        raw += b"\x00"  # no filter
        for col in range(width):
            r, g, b = pixels[row * width + col]
            raw += bytes([r, g, b])

    print("after iteration, before strut pack")

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png  = (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw, 6))
            + chunk(b"IEND", b""))
    with open(filename, "wb") as f:
        f.write(png)


def generate_map_image(search_local, waypoints_local,
                       plane_lat, plane_lon, start_wp_idx,
                       origin_lat, origin_lon,
                       filename="mission1_map.png"):
    """
    Render a top-down PNG map showing:
      • Search-area quadrilateral   (yellow border)
      • Diamond path with direction arrows  (cyan)
      • Waypoint rings              (yellow)
      • Start-waypoint highlight    (orange)
      • Plane position dot          (green)
      • Arrow: plane → start WP    (green)
    """
    import zlib, struct
    W, H, PAD = 800, 800, 60

    plane_x, plane_y = latlon_to_local(plane_lat, plane_lon, origin_lat, origin_lon)

    all_pts = list(search_local) + list(waypoints_local) + [(plane_x, plane_y)]
    min_x = min(p[0] for p in all_pts)
    max_x = max(p[0] for p in all_pts)
    min_y = min(p[1] for p in all_pts)
    max_y = max(p[1] for p in all_pts)
    span_x = max_x - min_x or 1.0
    span_y = max_y - min_y or 1.0

    def to_px(lx, ly):
        px = int(PAD + (lx - min_x) / span_x * (W - 2 * PAD))
        py = int(PAD + (1.0 - (ly - min_y) / span_y) * (H - 2 * PAD))
        return px, py

    pixels = [(18, 18, 30)] * (W * H)

    def set_px(px, py, c):
        if 0 <= px < W and 0 <= py < H:
            pixels[py * W + px] = c

    def draw_line(x0, y0, x1, y1, c, t=1):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            for tx in range(-(t // 2), t // 2 + 1):
                for ty in range(-(t // 2), t // 2 + 1):
                    set_px(x0 + tx, y0 + ty, c)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; x0 += sx
            if e2 < dx:
                err += dx; y0 += sy

    def draw_circle(cx, cy, r, c, fill=False):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if fill:
                    if dx * dx + dy * dy <= r * r:
                        set_px(cx + dx, cy + dy, c)
                else:
                    if abs(math.hypot(dx, dy) - r) < 1.2:
                        set_px(cx + dx, cy + dy, c)

    def draw_arrow(x0, y0, x1, y1, c, head=10):
        draw_line(x0, y0, x1, y1, c, 2)
        ang = math.atan2(y1 - y0, x1 - x0)
        for side in (0.45, -0.45):
            ax = int(x1 - head * math.cos(ang - side))
            ay = int(y1 - head * math.sin(ang - side))
            draw_line(x1, y1, ax, ay, c, 2)

    # Search area border
    sc = [to_px(p[0], p[1]) for p in search_local]
    for i in range(len(sc)):
        draw_line(*sc[i], *sc[(i + 1) % len(sc)], (200, 200, 60), 1)

    # Diamond path with arrows
    wp_px = [to_px(p[0], p[1]) for p in waypoints_local]
    nd = len(wp_px)
    for i in range(nd):
        draw_arrow(*wp_px[i], *wp_px[(i + 1) % nd], (0, 200, 220), 12)

    # Waypoint rings
    for i, (px, py) in enumerate(wp_px):
        draw_circle(px, py, 8, (255, 220, 0))
        draw_circle(px, py, 3, (255, 220, 0), fill=True)

    # Start-waypoint highlight
    sx, sy = wp_px[start_wp_idx]
    draw_circle(sx, sy, 13, (255, 100, 40))
    draw_circle(sx, sy, 14, (255, 100, 40))

    # Plane
    pp = to_px(plane_x, plane_y)
    draw_circle(pp[0], pp[1], 7, (50, 255, 100), fill=True)

    # Arrow: plane → start waypoint
    draw_arrow(pp[0], pp[1], sx, sy, (50, 255, 100), 14)


    _write_png(filename, W, H, pixels, zlib, struct)
    print(f"[IMAGE] Saved → {filename}")


# ---------------------------------------------------------------------------
# Main mission loop

def run_mission_1():
    print("[MISSION 1] Starting waypoint system …")

    # Load state
    state = load_state()
    nav   = state.get("navigation", {})

    raw_search = nav.get("search_area", [])
    if len(raw_search) != 4:
        raise ValueError(
            f"search_area must contain exactly 4 coordinate pairs, got {len(raw_search)}"
        )

    search_coords = []
    for entry in raw_search:
        if not isinstance(entry, (list, tuple)) or len(entry) < 2:
            raise ValueError(f"Invalid search_area entry: {entry}")
        search_coords.append((float(entry[0]), float(entry[1])))

    fusion_log_path = state.get("fusion_log", "")

    # Set mission phase
    nav["mission_phase"] = 1
    update_state("navigation", nav)
    print("[STATE] mission_phase → 1")

    # Build waypoints
    (waypoints, search_local, diamond_local,
     waypoints_local, o_lat, o_lon) = build_diamond_waypoints(search_coords)

    print(f"[WAYPOINTS] {len(waypoints)} waypoints generated:")
    for i, (lat, lon) in enumerate(waypoints):
        print(f"  WP{i}  lat={lat:.7f}  lon={lon:.7f}")

    # Initial telemetry
    telem = read_latest_telemetry(fusion_log_path)
    if telem is None:
        raise RuntimeError(
            "No telemetry available — cannot determine plane position. "
            "Check fusion_log path in mission_state.json."
        )

    plane_lat = telem["lat_deg"]
    plane_lon = telem["lon_deg"]
    plane_yaw = telem.get("yaw_deg", 0.0)

    # Nearest waypoint
    current_wp_idx = closest_waypoint_index(plane_lat, plane_lon, waypoints)
    print(f"[NAV] Plane at ({plane_lat:.6f}, {plane_lon:.6f})  "
          f"→ starting on WP{current_wp_idx}")

    # Generate map image
    if GENERATE_IMAGE:
        try:
            img_path = Path(__file__).resolve().parent / "mission1_map.png"
            generate_map_image(
                search_local, waypoints_local,
                plane_lat, plane_lon, current_wp_idx,
                o_lat, o_lon,
                filename=str(img_path),
            )
        except Exception as img_err:
            import traceback
            print(f"[ERROR] Image generation failed: {img_err}")
            traceback.print_exc()

    # Write initial waypoints to state
    nav = load_state().get("navigation", {})
    nav["current_waypoint"]  = list(waypoints[current_wp_idx])
    nav["guidance_waypoint"] = list(waypoints[current_wp_idx])
    update_state("navigation", nav)

    print("[MISSION 1] Entering guidance loop  (Ctrl-C to stop) …")

    # Guidance loop
    try:
        while True:
            telem = read_latest_telemetry(fusion_log_path)
            if telem is None:
                time.sleep(UPDATE_INTERVAL_S)
                continue

            plane_lat = telem["lat_deg"]
            plane_lon = telem["lon_deg"]
            plane_yaw = telem.get("yaw_deg", 0.0)

            wp_lat, wp_lon = waypoints[current_wp_idx]
            dist_to_wp     = haversine(plane_lat, plane_lon, wp_lat, wp_lon)

            # Waypoint capture
            if dist_to_wp < WAYPOINT_RADIUS_M:
                current_wp_idx = (current_wp_idx + 1) % len(waypoints)
                wp_lat, wp_lon = waypoints[current_wp_idx]
                print(f"[NAV] WP captured → advancing to WP{current_wp_idx}  "
                      f"({wp_lat:.7f}, {wp_lon:.7f})")

                nav = load_state().get("navigation", {})
                nav["current_waypoint"]  = [wp_lat, wp_lon]
                nav["guidance_waypoint"] = [wp_lat, wp_lon]
                update_state("navigation", nav)

            # L1 guidance
            guidance_hdg, desired_brng, dist = l1_guidance(
                plane_lat, plane_lon, plane_yaw, wp_lat, wp_lon
            )

            # L1 lookahead point on the track
            guide_lat, guide_lon = destination(
                plane_lat, plane_lon,
                desired_brng,
                min(L1_DISTANCE_M, dist),
            )

            nav = load_state().get("navigation", {})
            nav["guidance_waypoint"] = [guide_lat, guide_lon]
            update_state("navigation", nav)

            time.sleep(UPDATE_INTERVAL_S)

    except KeyboardInterrupt:
        print("\n[MISSION 1] Guidance loop stopped.")

    print("[MISSION 1] Done.")


##########################
if __name__ == "__main__":
    run_mission_1()