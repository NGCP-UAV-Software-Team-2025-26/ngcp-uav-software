import sys
import json
import math
import time
#import subprocess
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.mission_state_utils import load_state, update_state
from state.nav_state_utils import load_nav_state, update_nav_state

#######################################################################################################################################
### JUST AS A NOTE, THIS WAYPOINT SYSTEM WILL ONLY WORK IF THE SEARCH AREA IS A **CONVEX** QUADRILATERAL. IT WILL NOT WORK FOR CONCAVE. 
#######################################################################################################################################

# MISSION 1 CONFIGURATION
MAX_TURN_ANGLE_DEG = 35.0   # Max heading change at any waypoint (deg)
WAYPOINT_RADIUS_M  = 20.0   # Capture radius: advance to next WP within this distance (m)
BORDER_CLEARANCE_M = 50.0   # Diamond corners must be at least this far from search border (m)    ###################### Adjust this as necessary
L1_DISTANCE_M      = 30.0   # L1 guidance lookahead distance (m)
UPDATE_INTERVAL_S  = 0.1    # Guidance loop polling interval (s)
GENERATE_IMAGE     = True   # Toggle PNG map generation (True / False)
#MISSION_TIMEOUT_S  = 240.0  # Maximum mission runtime in seconds (0 = no limit)

EARTH_RADIUS_M = 6_371_000.0


# Geodetic helpers

def haversine(lat1, lon1, lat2, lon2):
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = rlat2 - rlat1, rlon2 - rlon1
    a = math.sin(dlat/2)**2 + math.cos(rlat1)*math.cos(rlat2)*math.sin(dlon/2)**2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(max(0.0, min(1.0, a))))


def bearing_to(lat1, lon1, lat2, lon2):
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = math.cos(rlat1)*math.sin(rlat2) - math.sin(rlat1)*math.cos(rlat2)*math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def destination(lat, lon, brng_deg, dist_m):
    br = math.radians(brng_deg)
    rlat, rlon = math.radians(lat), math.radians(lon)
    dr = dist_m / EARTH_RADIUS_M
    nlat = math.asin(math.sin(rlat)*math.cos(dr) + math.cos(rlat)*math.sin(dr)*math.cos(br))
    nlon = rlon + math.atan2(math.sin(br)*math.sin(dr)*math.cos(rlat),
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


def normalize_angle(a):
    while a >  180: a -= 360
    while a <= -180: a += 360
    return a


# -------------------------------------------------------
# 2-D geometry helpers (all units: metres in local frame)
# -------------------------------------------------------

def pt_to_seg_dist(px, py, ax, ay, bx, by):
    """Shortest distance from point P to segment AB."""
    dx, dy = bx - ax, by - ay
    sq = dx*dx + dy*dy
    if sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / sq))
    return math.hypot(px - (ax + t*dx), py - (ay + t*dy))


def min_dist_to_poly_edges(px, py, poly):
    """Minimum distance from point to any edge of a closed polygon."""
    n = len(poly)
    return min(
        pt_to_seg_dist(px, py,
                       poly[i][0], poly[i][1],
                       poly[(i+1)%n][0], poly[(i+1)%n][1])
        for i in range(n)
    )


def sort_clockwise(pts):
    cx, cy = centroid(pts)
    return sorted(pts, key=lambda p: -math.atan2(p[1]-cy, p[0]-cx))


def shrink_vertex_to_clearance(cx, cy, vx, vy, poly, clearance_m):
    """
    The raw diamond vertex (vx,vy) is a midpoint of a search-area edge and
    therefore lies ON the boundary (dist ≈ 0).  We need to move it inward
    along the ray  centroid→vertex  until it sits exactly CLEARANCE_M inside
    the polygon boundary.

    Strategy
    --------
    The centroid is deep inside the polygon so dist(centroid, boundary) >> 0.
    We binary-search along the segment  centroid→vertex  for the point whose
    distance to the nearest edge equals exactly clearance_m.

    lo  = parameter where dist >= clearance  (start: 0 = centroid, safe)
    hi  = parameter where dist <  clearance  (start: 1 = vertex,   on border)

    We maximise t (go as far outward as possible) while keeping dist >= clearance.
    """
    dx, dy = vx - cx, vy - cy

    # Parameterise the ray: P(t) = centroid + t*(vertex - centroid), t in [0,1]
    # t=0 → centroid (safe), t=1 → vertex (on border, unsafe)
    lo, hi = 0.0, 1.0

    for _ in range(64):          # 64 iterations → sub-micrometre precision
        mid = (lo + hi) * 0.5
        qx, qy = cx + mid*dx, cy + mid*dy
        if min_dist_to_poly_edges(qx, qy, poly) >= clearance_m:
            lo = mid             # safe here — try further outward
        else:
            hi = mid             # too close to edge — pull inward

    # lo is the furthest safe position
    return cx + lo*dx, cy + lo*dy


# --------------------------------------------------------------
# Corner rounding: entry / apex / exit per shrunk diamond vertex
# --------------------------------------------------------------

def make_corner_waypoints(prev_v, curr_v, next_v):
    """
    Given three consecutive shrunk-diamond vertices, compute the three
    waypoints that round the corner at curr_v:

        entry  — on the incoming leg, set back from curr_v
        apex   — midpoint between entry and exit (crown of the arc)
        exit   — on the outgoing leg, set back from curr_v

    The set-back is chosen so the heading change from  prev→entry  to
    entry→apex  does not exceed MAX_TURN_ANGLE_DEG.  If the full turn is
    already within the limit, all three collapse to curr_v (no rounding).

    The set-back distance scales with WAYPOINT_RADIUS_M so that shrinking
    the diamond automatically shrinks the rounding geometry.
    """
    in_dx  = curr_v[0] - prev_v[0]
    in_dy  = curr_v[1] - prev_v[1]
    out_dx = next_v[0] - curr_v[0]
    out_dy = next_v[1] - curr_v[1]

    in_len  = math.hypot(in_dx,  in_dy)
    out_len = math.hypot(out_dx, out_dy)

    if in_len < 1e-9 or out_len < 1e-9:
        return curr_v, curr_v, curr_v

    in_ang  = math.degrees(math.atan2(in_dy,  in_dx))
    out_ang = math.degrees(math.atan2(out_dy, out_dx))
    turn    = normalize_angle(out_ang - in_ang)

    if abs(turn) <= MAX_TURN_ANGLE_DEG:
        # Turn is gentle enough — use the vertex itself as the single waypoint
        return curr_v, curr_v, curr_v

    # Set-back distance: use WAYPOINT_RADIUS_M as the rounding radius.
    # For a turn of |turn| degrees the set-back along each leg is:
    #   set_back = radius / tan(|turn|/2)
    half_turn_rad = math.radians(abs(turn) / 2.0)
    set_back = WAYPOINT_RADIUS_M / math.tan(half_turn_rad) if half_turn_rad > 1e-6 else 0.0

    # Clamp so we never overshoot the midpoint of the incoming/outgoing leg
    set_back = min(set_back, in_len * 0.45, out_len * 0.45)

    # Entry: walk set_back metres back along the incoming leg from curr_v
    entry = (curr_v[0] - (in_dx / in_len) * set_back,
             curr_v[1] - (in_dy / in_len) * set_back)

    # Exit: walk set_back metres forward along the outgoing leg from curr_v
    exit_ = (curr_v[0] + (out_dx / out_len) * set_back,
             curr_v[1] + (out_dy / out_len) * set_back)

    # Apex: geometric midpoint of entry and exit
    apex  = ((entry[0] + exit_[0]) / 2.0,
             (entry[1] + exit_[1]) / 2.0)

    return entry, apex, exit_


# ---------------------
# Full diamond pipeline
# ---------------------

def build_diamond(search_coords):
    """
    Step 1  Convert 4 search-area corners to local-metre frame.
    Step 2  Sort corners clockwise.
    Step 3  Midpoint of each edge → raw diamond (4 vertices ON the border).
    Step 4  Shrink each raw vertex inward along centroid→vertex ray until
            its distance to every search-area edge >= BORDER_CLEARANCE_M.
    Step 5  Round each shrunk vertex with entry/apex/exit waypoints so the
            heading change never exceeds MAX_TURN_ANGLE_DEG.

    Returns
    -------
    waypoints_ll   [(lat,lon)]          ordered flight sequence (clockwise)
    search_xy      [(x,y)]             search area corners, local m, clockwise
    raw_diam_xy    [(x,y)]             raw diamond (step 3) — on the border
    shrunk_diam_xy [(x,y)]             shrunk diamond (step 4) — inside border
    wp_xy          [(x,y)]             waypoints, local m (same order as waypoints_ll)
    corner_triples [((x,y),(x,y),(x,y))] (entry, apex, exit) per corner
    o_lat, o_lon   float               local frame origin
    """
    o_lat = sum(p[0] for p in search_coords) / 4
    o_lon = sum(p[1] for p in search_coords) / 4

    local_pts = [to_local(lat, lon, o_lat, o_lon) for lat, lon in search_coords]
    search_xy = sort_clockwise(local_pts)

    # Step 3 — edge midpoints, these sit exactly on the search-area boundary
    n = len(search_xy)
    raw_diam_xy = [
        ((search_xy[i][0] + search_xy[(i+1)%n][0]) / 2.0,
         (search_xy[i][1] + search_xy[(i+1)%n][1]) / 2.0)
        for i in range(n)
    ]

    # Step 4 — shrink each vertex inward to BORDER_CLEARANCE_M from the boundary
    dc_x, dc_y = centroid(raw_diam_xy)
    shrunk_diam_xy = [
        shrink_vertex_to_clearance(dc_x, dc_y, vx, vy, search_xy, BORDER_CLEARANCE_M)
        for vx, vy in raw_diam_xy
    ]

    # Step 5 — rounded corners
    nd = len(shrunk_diam_xy)
    corner_triples = [
        make_corner_waypoints(
            shrunk_diam_xy[(i-1) % nd],
            shrunk_diam_xy[i],
            shrunk_diam_xy[(i+1) % nd],
        )
        for i in range(nd)
    ]

    # Flatten into flight sequence, collapsing identical triples to one point
    wp_xy = []
    for entry, apex, exit_ in corner_triples:
        if entry == apex == exit_:
            wp_xy.append(apex)
        else:
            wp_xy.append(entry)
            wp_xy.append(apex)
            wp_xy.append(exit_)

    waypoints_ll = [from_local(x, y, o_lat, o_lon) for x, y in wp_xy]

    return (waypoints_ll, search_xy, raw_diam_xy, shrunk_diam_xy,
            wp_xy, corner_triples, o_lat, o_lon)


# -------------------------------------------------
# Fusion-log reader — reads last line of JSONL file
# -------------------------------------------------

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


# --------------------------
# Closest waypoint selection
# --------------------------

def closest_wp_index(plane_lat, plane_lon, waypoints_ll):
    best_i, best_d = 0, float("inf")
    for i, (wlat, wlon) in enumerate(waypoints_ll):
        d = haversine(plane_lat, plane_lon, wlat, wlon)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


# -----------
# L1 Guidance ######################################## ( might not be necessary, will probably remove)
# -----------

def l1_guidance(plane_lat, plane_lon, plane_yaw, wp_lat, wp_lon):
    dist_m   = haversine(plane_lat, plane_lon, wp_lat, wp_lon)
    des_brng = bearing_to(plane_lat, plane_lon, wp_lat, wp_lon)
    eta      = math.radians(normalize_angle(des_brng - plane_yaw))
    corr     = math.degrees(math.asin(
        max(-1.0, min(1.0, 2.0 * math.sin(eta) * dist_m / max(dist_m, L1_DISTANCE_M)))
    ))
    return (des_brng + corr) % 360, des_brng, dist_m


# --------------------
# PNG image generation
# --------------------

def _write_png(filename, width, height, pixels):
    import zlib, struct

    def chunk(tag, data):
        payload = tag + data
        return (struct.pack(">I", len(data))
                + payload
                + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF))

    stride = 1 + width * 3
    raw = bytearray(height * stride)
    for row in range(height):
        rs = row * stride
        raw[rs] = 0                          # PNG filter: None
        for col in range(width):
            r, g, b = pixels[row * width + col]
            off = rs + 1 + col * 3
            raw[off] = r; raw[off+1] = g; raw[off+2] = b

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    data = (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
            + chunk(b"IEND", b""))
    with open(filename, "wb") as f:
        f.write(data)


def generate_map_image(search_xy, raw_diam_xy, shrunk_diam_xy, wp_xy,
                       corner_triples, plane_lat, plane_lon, start_wp_idx,
                       o_lat, o_lon, filename="mission1_map.png"):
    """
    Renders:
      • Search-area quadrilateral border        (dim yellow)
      • Raw diamond (edge midpoints)            (dim grey, dashed-style)
      • Shrunk diamond                          (white)
      • Rounded flight path arrows WP→WP       (cyan)
      • Entry/apex/exit dots per corner         (yellow)
      • Active start waypoint highlight         (orange ring)
      • Plane position                          (green filled circle)
      • Arrow: plane → first waypoint           (green)
    """
    W, H, PAD = 800, 800, 70

    plane_x, plane_y = to_local(plane_lat, plane_lon, o_lat, o_lon)

    all_xy = list(search_xy) + list(shrunk_diam_xy) + list(wp_xy) + [(plane_x, plane_y)]
    min_x = min(p[0] for p in all_xy)
    max_x = max(p[0] for p in all_xy)
    min_y = min(p[1] for p in all_xy)
    max_y = max(p[1] for p in all_xy)
    span_x = max_x - min_x or 1.0
    span_y = max_y - min_y or 1.0
    # Keep aspect ratio square — use the larger span for both axes
    span   = max(span_x, span_y)
    cx_w   = (min_x + max_x) / 2
    cy_w   = (min_y + max_y) / 2

    draw_w = W - 2*PAD
    draw_h = H - 2*PAD

    def to_px(lx, ly):
        px = int(PAD + (lx - (cx_w - span/2)) / span * draw_w)
        py = int(PAD + (1.0 - (ly - (cy_w - span/2)) / span) * draw_h)
        return px, py

    pixels = [(15, 17, 26)] * (W * H)

    def set_px(px, py, c):
        if 0 <= px < W and 0 <= py < H:
            pixels[py * W + px] = c

    def draw_line(x0, y0, x1, y1, c, t=1):
        dx, dy = abs(x1-x0), abs(y1-y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        half = t // 2
        while True:
            for tx in range(-half, half+1):
                for ty in range(-half, half+1):
                    set_px(x0+tx, y0+ty, c)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2*err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 <  dx: err += dx; y0 += sy

    def draw_circle(cx, cy, r, c, fill=False):
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                d = math.hypot(dx, dy)
                if fill:
                    if d <= r: set_px(cx+dx, cy+dy, c)
                else:
                    if abs(d - r) < 1.2: set_px(cx+dx, cy+dy, c)

    def draw_arrow(x0, y0, x1, y1, c, head=12, t=2):
        draw_line(x0, y0, x1, y1, c, t)
        ang = math.atan2(y1-y0, x1-x0)
        for side in (0.42, -0.42):
            ax = int(x1 - head * math.cos(ang - side))
            ay = int(y1 - head * math.sin(ang - side))
            draw_line(x1, y1, ax, ay, c, t)

    def draw_dashed_poly(pts, c, dash=8, gap=6):
        n = len(pts)
        for i in range(n):
            ax, ay = to_px(*pts[i])
            bx, by = to_px(*pts[(i+1)%n])
            dx, dy = bx-ax, by-ay
            length = math.hypot(dx, dy)
            if length < 1e-9:
                continue
            ux, uy = dx/length, dy/length
            pos = 0.0
            drawing = True
            while pos < length:
                seg = dash if drawing else gap
                end = min(pos + seg, length)
                if drawing:
                    x0 = int(ax + pos*ux); y0 = int(ay + pos*uy)
                    x1 = int(ax + end*ux); y1 = int(ay + end*uy)
                    draw_line(x0, y0, x1, y1, c, 1)
                pos = end
                drawing = not drawing

    # --- draw layers (back to front) ---

    # Search-area border
    n = len(search_xy)
    for i in range(n):
        ax, ay = to_px(*search_xy[i])
        bx, by = to_px(*search_xy[(i+1)%n])
        draw_line(ax, ay, bx, by, (160, 150, 50), 1)

    # Raw diamond (dashed grey) — shows where midpoints fall before shrinking
    draw_dashed_poly(raw_diam_xy, (80, 80, 90))

    # Shrunk diamond outline (solid white)
    nd = len(shrunk_diam_xy)
    for i in range(nd):
        ax, ay = to_px(*shrunk_diam_xy[i])
        bx, by = to_px(*shrunk_diam_xy[(i+1)%nd])
        draw_line(ax, ay, bx, by, (200, 200, 210), 1)

    # Flight path arrows between consecutive waypoints (cyan)
    nw = len(wp_xy)
    for i in range(nw):
        ax, ay = to_px(*wp_xy[i])
        bx, by = to_px(*wp_xy[(i+1)%nw])
        draw_arrow(ax, ay, bx, by, (0, 190, 215), 10, 2)

    # Entry / apex / exit dots per corner
    for entry, apex, exit_ in corner_triples:
        if entry == apex == exit_:
            px, py = to_px(*apex)
            draw_circle(px, py, 5, (255, 210, 0), fill=True)
        else:
            for pt, col in ((entry, (255, 160, 0)), (apex, (255, 210, 0)), (exit_, (255, 160, 0))):
                px, py = to_px(*pt)
                draw_circle(px, py, 4, col, fill=True)

    # Start-waypoint orange highlight ring
    sx, sy = to_px(*wp_xy[start_wp_idx])
    draw_circle(sx, sy, 13, (255, 90, 30))
    draw_circle(sx, sy, 14, (255, 90, 30))

    # Plane (green filled dot)
    ppx, ppy = to_px(plane_x, plane_y)
    draw_circle(ppx, ppy, 7, (40, 230, 90), fill=True)

    # Arrow: plane → start waypoint
    draw_arrow(ppx, ppy, sx, sy, (40, 230, 90), 12, 2)

    _write_png(filename, W, H, pixels)
    print(f"[IMAGE] Saved → {filename}")


# ---------------------------------------------------------------------------
# Main mission loop

def run_mission_1():
    print("[MISSION 1] Starting …")

    mission_state = load_state()
    nav_state     = load_nav_state()

    raw_search = nav_state.get("search_area", [])
    if len(raw_search) != 4:
        raise ValueError(f"search_area must have 4 entries, got {len(raw_search)}")

    search_coords = []
    for entry in raw_search:
        if not isinstance(entry, (list, tuple)) or len(entry) < 2:
            raise ValueError(f"Invalid search_area entry: {entry}")
        search_coords.append((float(entry[0]), float(entry[1])))

    fusion_log_path = mission_state.get("fusion_log", "")

    # ------------------------------------------------------------------
    # Startup: write plan metadata & mission phase before anything else
    # ------------------------------------------------------------------
    nav_state = load_nav_state()          # fresh read

    nav = nav_state.get("navigation", {})
    nav["mission_phase"] = 1
    update_nav_state("navigation", nav)
    print("[STATE] mission_phase → 1")

    active_plan = nav_state.get("active_plan", {})
    active_plan["plan_id"]   = 1
    active_plan["plan_type"] = "Diamond Pattern"
    active_plan["status"]    = "searching"
    update_nav_state("active_plan", active_plan)

    update_nav_state("next_plan", 2)

    print("[STATE] active_plan → plan_id=1, plan_type='Diamond Pattern', status='searching'")
    print("[STATE] next_plan   → 2")

    # Build diamond and waypoints
    (waypoints_ll, search_xy, raw_diam_xy, shrunk_diam_xy,
     wp_xy, corner_triples, o_lat, o_lon) = build_diamond(search_coords)
    
    # ------------------------------------------------------------------
    # Serialise the 12 waypoints into active_plan.waypoints
    # alt_m is read from active_plan.alt_m (already in the JSON)
    # ------------------------------------------------------------------
    nav_state   = load_nav_state()
    active_plan = nav_state.get("active_plan", {})
    alt_m       = active_plan.get("alt_m")          # None is fine — kept as-is

    active_plan["waypoints"] = [
        {"lat": lat, "lon": lon, "alt_m": alt_m}
        for lat, lon in waypoints_ll
    ]
    update_nav_state("active_plan", active_plan)
    print(f"[STATE] active_plan.waypoints → {len(waypoints_ll)} entries written")

    print(f"[WAYPOINTS] {len(waypoints_ll)} waypoints generated:")
    for i, (lat, lon) in enumerate(waypoints_ll):
        print(f"  WP{i}  lat={lat:.7f}  lon={lon:.7f}")

    # Get initial telemetry
    telem = read_latest_telemetry(fusion_log_path)
    if telem is None:
        raise RuntimeError(
            "No telemetry available. Check fusion_log path in mission_state.json."
        )

    plane_lat = telem["lat_deg"]
    plane_lon = telem["lon_deg"]
    plane_yaw = telem.get("yaw_deg", 0.0)

    current_wp_idx = closest_wp_index(plane_lat, plane_lon, waypoints_ll)
    print(f"[NAV] Plane at ({plane_lat:.6f}, {plane_lon:.6f}) "
          f"→ starting on WP{current_wp_idx}")

    # Generate image
    if GENERATE_IMAGE:
        try:
            img_path = Path(__file__).resolve().parent / "mission1_map.png"
            generate_map_image(
                search_xy, raw_diam_xy, shrunk_diam_xy, wp_xy,
                corner_triples, plane_lat, plane_lon, current_wp_idx,
                o_lat, o_lon, filename=str(img_path),
            )
        except Exception:
            import traceback
            print("[ERROR] Image generation failed:")
            traceback.print_exc()

    # Write initial state
    nav = load_nav_state().get("navigation", {})
    nav["current_waypoint"]  = list(waypoints_ll[current_wp_idx])
    nav["guidance_waypoint"] = list(waypoints_ll[current_wp_idx])
    update_nav_state("navigation", nav)

    print("[MISSION 1] Entering guidance loop (Ctrl-C to stop) …")

    #mission_start = time.monotonic()

    try:
        while True:
            #---- timeout check ----------------------------------------
            #if MISSION_TIMEOUT_S > 0:
            #    elapsed = time.monotonic() - mission_start
            #    if elapsed >= MISSION_TIMEOUT_S:
            #        print(f"[MISSION 1] Timeout reached ({MISSION_TIMEOUT_S:.0f}s). Ending mission.")
            #        break
            #--------------------------------------------------------------
            refined = load_nav_state().get("mra_refined_loiter_target", {})
            if refined["valid"] == True:
                print("Loiter coordinates received. Ending mission 1. Proceeding to Mission 2.")
                break

            telem = read_latest_telemetry(fusion_log_path)
            if telem is None:
                time.sleep(UPDATE_INTERVAL_S)
                continue

            plane_lat = telem["lat_deg"]
            plane_lon = telem["lon_deg"]
            plane_yaw = telem.get("yaw_deg", 0.0)

            wp_lat, wp_lon = waypoints_ll[current_wp_idx]
            dist_to_wp = haversine(plane_lat, plane_lon, wp_lat, wp_lon)

            # Waypoint capture
            if dist_to_wp < WAYPOINT_RADIUS_M:
                current_wp_idx = (current_wp_idx + 1) % len(waypoints_ll)
                wp_lat, wp_lon = waypoints_ll[current_wp_idx]
                print(f"[NAV] Captured → WP{current_wp_idx} "
                      f"({wp_lat:.7f}, {wp_lon:.7f})")
                nav = load_nav_state().get("navigation", {})
                nav["current_waypoint"]  = [wp_lat, wp_lon]
                nav["guidance_waypoint"] = [wp_lat, wp_lon]
                update_nav_state("navigation", nav)

            # L1 guidance — compute lookahead point and write to state
            _, des_brng, dist = l1_guidance(
                plane_lat, plane_lon, plane_yaw, wp_lat, wp_lon
            )
            guide_lat, guide_lon = destination(
                plane_lat, plane_lon, des_brng, min(L1_DISTANCE_M, dist)
            )
            nav = load_nav_state().get("navigation", {})
            nav["guidance_waypoint"] = [guide_lat, guide_lon]
            update_nav_state("navigation", nav)

            time.sleep(UPDATE_INTERVAL_S)

    except KeyboardInterrupt:
        print("\n[MISSION 1] Stopped.")

    # ------------------------------------------------------------------
    # Mission end: mark status complete, then hand off to mission 2
    # ------------------------------------------------------------------
    nav_state   = load_nav_state()
    active_plan = nav_state.get("active_plan", {})
    active_plan["status"] = "complete"
    update_nav_state("active_plan", active_plan)
    print("[STATE] active_plan.status → 'complete'")

    print("[MISSION 1] Done. Launching mission2_waypoint.py …")
    mission2_path = Path(__file__).resolve().parent / "mission2_waypoint.py"
    if mission2_path.exists():
        subprocess.Popen([sys.executable, str(mission2_path)])
        print(f"[HANDOFF] Launched {mission2_path}")
    else:
        print(f"[WARN] mission2_waypoint.py not found at {mission2_path} — skipping handoff.")


if __name__ == "__main__":
    run_mission_1()