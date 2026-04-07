"""
DOA triangulation processor for Kraken + telemetry data

Core idea
---------
Each high-confidence fusion record gives us the plane's GPS position AND a compass bearing toward the RF signal source (doa_deg). 
That is a ray, not a point.  This processor casts a ray from every high-confidence record and finds where the rays intersect. 
When enough intersection points cluster tightly together, the centroid of that cluster is published as the target fix.

Algorithm per cycle
-------------------
1.  Read only *new* lines appended to the fusion JSONL log.
2.  Keep records whose confidence >= _confidence_threshold ("memory").
3.  Recompute all pairwise ray intersections for the current memory.
4.  Search intersections for a spatial cluster of size >= _cluster_count_threshold
    all within _vicinity_threshold_m of each other.
5.  If a cluster is found:
      a. Confidence-weighted centroid of the cluster intersection points.
      b. Blend in the previous estimate (if any) as an extra weighted point.
      c. Publish to state["target_fix"], increment fix_id.
      d. Adapt the three live thresholds based on drift from the prior estimate.
      e. Clear memory and intersection cache; retain the new estimate.
6.  Sleep to maintain STATE_POLL_HZ.

"""

import sys
import json
import time
import math
import logging
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.state_utils import load_state, update_state, STATE_FILE
STATE_POLL_HZ = 2.0

#  BASE (INITIAL) THRESHOLDS  –  live values start here and adapt from here
###########################################################################
CONFIDENCE_THRESHOLD_BASE    = 0.50   # float [0,1]  min confidence_0_1 to enter memory
CLUSTER_COUNT_THRESHOLD_BASE = 10      # int          min intersections in vicinity to trigger
VICINITY_THRESHOLD_M_BASE    = 500.0  # float (m)    neighbourhood radius for intersection cluster

#  ADAPTIVE THRESHOLD CONTROLS
#############################################################
# Distance that defines "close" between consecutive estimates
CONVERGENCE_DISTANCE_M  = 200.0   

#  When consecutive estimates are CLOSE (converging) -> tighten the net
VICINITY_DECREASE_M     = 50.0    # shrink vicinity by this much
CLUSTER_COUNT_DECREASE  = 1       # require fewer intersections in cluster

#  When consecutive estimates are FAR (diverging) -> broaden search, demand quality
VICINITY_INCREASE_M     = 100.0   # expand vicinity by this much
CONFIDENCE_INCREASE     = 0.05    # raise minimum confidence bar
CLUSTER_COUNT_INCREASE  = 1       # require more intersections in cluster

#  Hard clamp ranges so thresholds cannot run away
MIN_CONFIDENCE          = 0.5
MAX_CONFIDENCE          = 0.95
MIN_VICINITY_M          = 30.0
MAX_VICINITY_M          = 500.0
MIN_CLUSTER_COUNT       = 5
MAX_CLUSTER_COUNT       = 20

#  Sanity filter: discard intersections farther than this from the midpoint of
#  the two contributing aircraft positions (catches near-parallel ray blowups)
MAX_INTERSECTION_RANGE_M = 1_000.0  # 50 km ; increase for long-range RF
#############################################################

EARTH_RADIUS_M = 6_371_000.0

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("fusion_processor")


# adaptive threshold state
_confidence_threshold:    float = CONFIDENCE_THRESHOLD_BASE
_cluster_count_threshold: int   = CLUSTER_COUNT_THRESHOLD_BASE
_vicinity_threshold_m:    float = VICINITY_THRESHOLD_M_BASE

# persistent processor state
_memory: list[dict]            = []   # high-confidence fusion records
_current_estimate: dict | None = None
_fix_id: int                   = 0
_fusion_log_offset: int        = 0    # byte cursor into the JSONL file


#geometry

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    #Great-circle distance in metres between two (lat, long) points
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class _LocalCS:
    # Flat-earth (local tangent plane) coordinate system in meters
    # Accurate to ~0.1 % within ~50 km of the reference point
    __slots__ = ("ref_lat", "ref_lon", "_m_per_deg_lat", "_m_per_deg_lon")

    def __init__(self, ref_lat: float, ref_lon: float) -> None:
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self._m_per_deg_lat = EARTH_RADIUS_M * math.pi / 180.0
        self._m_per_deg_lon = self._m_per_deg_lat * math.cos(math.radians(ref_lat))

    def to_xy(self, lat: float, lon: float) -> tuple[float, float]:
        return (
            (lon - self.ref_lon) * self._m_per_deg_lon,
            (lat - self.ref_lat) * self._m_per_deg_lat,
        )

    def to_latlon(self, x: float, y: float) -> tuple[float, float]:
        return (
            self.ref_lat + y / self._m_per_deg_lat,
            self.ref_lon + x / self._m_per_deg_lon,
        )


def _ray_intersection(
    cs: _LocalCS,
    rec_a: dict,
    rec_b: dict,
) -> tuple[float, float] | None:
    # Find the forward intersection of two compass-bearing rays in local Cartesian
    # ray is defined by an aircraft position (lat_deg, lon_deg) and a bearing (doa_deg) in compass convention (0 = North, 90 = East, CW positive)
    # Returns (lat, long) of the intersection, or None if:
    #- rays are parallel (|det| < ε)
    #- intersection is *behind* either ray  (t < 0 or s < 0)
    #- intersection is implausibly far from both aircraft positions
    
    x0, y0 = cs.to_xy(rec_a["lat_deg"], rec_a["lon_deg"])
    x1, y1 = cs.to_xy(rec_b["lat_deg"], rec_b["lon_deg"])

    # Direction vectors in (East=x, North=y) from compass bearing
    b0, b1 = math.radians(rec_a["doa_deg"]), math.radians(rec_b["doa_deg"])
    dx0, dy0 = math.sin(b0), math.cos(b0)
    dx1, dy1 = math.sin(b1), math.cos(b1)

    # Solve:  [dx0  -dx1] [t]   [x1-x0]
    #         [dy0  -dy1] [s] = [y1-y0]
    det = -dx0 * dy1 + dx1 * dy0
    if abs(det) < 1e-10:
        return None  # parallel / anti-parallel rays

    dpx, dpy = x1 - x0, y1 - y0
    t = (-dpx * dy1 + dx1 * dpy) / det
    s = (-dpx * dy0 + dx0 * dpy) / det

    if t < 0.0 or s < 0.0:
        return None  # intersection is behind one of the aircraft

    ix, iy = x0 + t * dx0, y0 + t * dy0

    # Sanity: reject intersections absurdly far from both source positions
    mid_x, mid_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    if math.hypot(ix - mid_x, iy - mid_y) > MAX_INTERSECTION_RANGE_M:
        return None

    return cs.to_latlon(ix, iy)


#FUSION LOG I/O

def read_new_fusion_records(path: str | Path) -> list[dict]:
    # Read only lines appended to the JSONL log since the previous call
    # Advances `_fusion_log_offset` byte cursor
    # Skip malformed lines
    
    global _fusion_log_offset
    fusion_path = Path(path)
    if not fusion_path.exists():
        log.warning("Fusion log not found: %s", fusion_path)
        return []

    records: list[dict] = []
    with fusion_path.open("r", encoding="utf-8") as fh:
        fh.seek(_fusion_log_offset)
        for line in fh:
            stripped = line.strip()
            if not stripped:
                continue
            try:
                records.append(json.loads(stripped))
            except json.JSONDecodeError:
                pass
        _fusion_log_offset = fh.tell()

    if records:
        log.debug("Ingested %d new record(s), log offset → %d B", len(records), _fusion_log_offset)
    return records


def filter_high_confidence(records: list[dict]) -> list[dict]:
    """Return only records meeting the current live confidence threshold."""
    return [r for r in records if r.get("confidence_0_1", 0.0) >= _confidence_threshold]


# compute intersections

def compute_all_intersections(memory: list[dict]) -> list[dict]:
    # Compute all valid pairwise ray intersections for the current memory.

    # Returns a list of intersection dicts:
    # {
    #    "lat"    : float,          # intersection latitude
    #    "lon"    : float,          # intersection longitude
    #    "weight" : float,          # mean confidence of the two source rays
    #    "i"      : int,            # index of first contributing record in memory
    #    "j"      : int,            # index of second contributing record in memory
    # }

    # A fresh local coordinate system is built from the memory centroid so the flat-earth approximation is centered on the data
    
    n = len(memory)
    if n < 2:
        return []

    ref_lat = sum(r["lat_deg"] for r in memory) / n
    ref_lon = sum(r["lon_deg"] for r in memory) / n
    cs = _LocalCS(ref_lat, ref_lon)

    results: list[dict] = []
    for i in range(n):
        for j in range(i + 1, n):
            pt = _ray_intersection(cs, memory[i], memory[j])
            if pt is None:
                continue
            ilat, ilon = pt
            weight = (memory[i]["confidence_0_1"] + memory[j]["confidence_0_1"]) / 2.0
            results.append({"lat": ilat, "lon": ilon, "weight": weight, "i": i, "j": j})

    log.debug(
        "%d record(s) in memory → %d valid intersection(s)", n, len(results)
    )
    return results


# Clustering

def find_intersection_cluster(intersections: list[dict]) -> list[dict] | None:
    # Scan *intersections* for the densest neighbourhood that satisfies:
    #     |neighbours| >= _cluster_count_threshold
    #     all within _vicinity_threshold_m of the anchor.

    # Returns the list of cluster members (sorted by weight desc), or None
    # The anchor yielding the most neighbours is preferred; ties broken by weight
    
    best_cluster: list[dict] | None = None
    best_count = 0

    for anchor in intersections:
        neighbours = [
            pt for pt in intersections
            if haversine_m(anchor["lat"], anchor["lon"], pt["lat"], pt["lon"])
               <= _vicinity_threshold_m
        ]
        if len(neighbours) >= _cluster_count_threshold and len(neighbours) > best_count:
            best_count = len(neighbours)
            best_cluster = sorted(neighbours, key=lambda p: p["weight"], reverse=True)

    if best_cluster is None:
        log.debug(
            "No cluster yet  (need %d within %.0f m, got %d intersection(s))",
            _cluster_count_threshold, _vicinity_threshold_m, len(intersections),
        )
    return best_cluster



# estimate computation

def compute_estimate(
    cluster: list[dict],
    memory: list[dict],
    previous: dict | None,
    next_fix_id: int,
) -> dict:
    # Build the new target_fix estimate

    # Centroid
    # --------
    # Weighted by each intersection's weight (mean confidence of its two rays).
    # The previous estimate (if any) is injected as an additional contributor weighted by its own confidence, so each fix refines the last.

    # Estimate confidence
    # -------------------
    # Mean confidence of the unique memory records whose rays produced at least one intersection in the cluster. 
    # This reflects the quality of the signal directions, not just the intersection geometry.

    # Params
    # ------
    # cluster      : intersection cluster returned by find_intersection_cluster
    # memory       : full current memory (for per-ray confidence look-up)
    # previous     : last published target_fix, or None
    # next_fix_id  : fix_id to stamp on the result
    
    # Build contributor list: (lat, lon, weight)
    contributors: list[tuple[float, float, float]] = [
        (pt["lat"], pt["lon"], pt["weight"]) for pt in cluster
    ]

    if previous is not None:
        contributors.append((previous["lat"], previous["lon"], previous["confidence"]))

    total_w = sum(c[2] for c in contributors) or 1e-12
    w_lat   = sum(c[0] * c[2] for c in contributors) / total_w
    w_lon   = sum(c[1] * c[2] for c in contributors) / total_w

    # Identify unique memory indices that contributed to the cluster
    contributing_memory_indices: set[int] = set()
    for pt in cluster:
        contributing_memory_indices.add(pt["i"])
        contributing_memory_indices.add(pt["j"])

    # Mean confidence of those rays
    ray_confidences = [
        memory[idx]["confidence_0_1"]
        for idx in contributing_memory_indices
        if idx < len(memory)
    ]
    estimate_confidence = (sum(ray_confidences) / len(ray_confidences)) if ray_confidences else 0.0

    return {
        "lat":        w_lat,
        "lon":        w_lon,
        "alt":        None,  # DOA triangulation is 2-D; altitude not derived
        "confidence": estimate_confidence,
        "fix_id":     next_fix_id,
        "timestamp":  time.time(),
    }



# ADAPTIVE THRESHOLD UPDATE

def adapt_thresholds(prev: dict, new: dict) -> None:
    # Adjust the thresholds based on the drift between consecutive estimates.

    # Converging  (dist <= CONVERGENCE_DISTANCE_M)
    ##############################################
    #   vicinity       v  VICINITY_DECREASE_M
    #   cluster count  v  CLUSTER_COUNT_DECREASE
    #   confidence     unchanged

    # Diverging   (dist > CONVERGENCE_DISTANCE_M)
    ##############################################
    #   vicinity       ^  VICINITY_INCREASE_M
    #   confidence     ^  CONFIDENCE_INCREASE
    #   cluster count  ^  CLUSTER_COUNT_INCREASE
    
    global _confidence_threshold, _cluster_count_threshold, _vicinity_threshold_m

    dist = haversine_m(prev["lat"], prev["lon"], new["lat"], new["lon"])

    if dist <= CONVERGENCE_DISTANCE_M:
        _vicinity_threshold_m    = max(MIN_VICINITY_M,
                                       _vicinity_threshold_m - VICINITY_DECREASE_M)
        _cluster_count_threshold = max(MIN_CLUSTER_COUNT,
                                       _cluster_count_threshold - CLUSTER_COUNT_DECREASE)
        log.info(
            "Converging (drift=%.0f m) → vicinity=%.0f m, cluster≥%d, conf≥%.2f",
            dist, _vicinity_threshold_m, _cluster_count_threshold, _confidence_threshold,
        )
    else:
        _vicinity_threshold_m    = min(MAX_VICINITY_M,
                                       _vicinity_threshold_m + VICINITY_INCREASE_M)
        _confidence_threshold    = min(MAX_CONFIDENCE,
                                       _confidence_threshold + CONFIDENCE_INCREASE)
        _cluster_count_threshold = min(MAX_CLUSTER_COUNT,
                                       _cluster_count_threshold + CLUSTER_COUNT_INCREASE)
        log.info(
            "Diverging  (drift=%.0f m) → vicinity=%.0f m, cluster≥%d, conf≥%.2f",
            dist, _vicinity_threshold_m, _cluster_count_threshold, _confidence_threshold,
        )


# main process cycle

def process_once() -> None:
    
    # One full ingest -> triangulate -> (optional) publish cycle

    # State transitions
    # -----------------
    # Every call : read new lines -> filter -> extend memory
    # If cluster : compute estimate -> publish -> adapt thresholds -> clear memory
    
    global _memory, _current_estimate, _fix_id

    #get fusion log
    state = load_state()
    fusion_log_path: str | None = state.get("fusion_log")
    if not fusion_log_path:
        log.warning("State has no 'fusion_log' entry; skipping cycle.")
        return

    #Ingest new lines, append high-confidence records to memory
    new_records = read_new_fusion_records(fusion_log_path)
    high_conf   = filter_high_confidence(new_records)

    if high_conf:
        _memory.extend(high_conf)
        log.info(
            "+%d high-conf record(s) (conf≥%.2f) → memory size: %d",
            len(high_conf), _confidence_threshold, len(_memory),
        )

    # Need at least 2 records to form any intersection
    if len(_memory) < 2:
        return

    # Recompute all pairwise ray intersections from current memory
    intersections = compute_all_intersections(_memory)
    if not intersections:
        log.debug("No valid intersections yet.")
        return

    #Search for a qualifying spatial cluster of intersections
    cluster = find_intersection_cluster(intersections)
    if cluster is None:
        return

    #Cluster found -> compute new estimate
    _fix_id += 1
    estimate = compute_estimate(cluster, _memory, _current_estimate, _fix_id)

    log.info(
        "Cluster: %d intersection(s)  →  fix #%d | "
        "lat=%.6f  lon=%.6f  confidence=%.4f",
        len(cluster), _fix_id,
        estimate["lat"], estimate["lon"], estimate["confidence"],
    )

    # Publish to mission state
    update_state("target_fix", estimate)

    # Adapt thresholds since have have a previous and new estimate
    if _current_estimate is not None:
        adapt_thresholds(_current_estimate, estimate)

    # advance internal state
    _current_estimate = estimate
    _memory.clear()
    log.info("Memory cleared. Estimate retained for next-round blending.")




# continuous loop

def run_loop(poll_hz: float = STATE_POLL_HZ) -> None:
    # Drive process_once() at *poll_hz* Hz until KeyboardInterrupt.
    # Exceptions inside process_once() are caught and logged so the loop stays alive across transient errors (file locks, partial writes, etc)    

    interval = 1.0 / poll_hz
    log.info(
        "Triangulation processor starting  "
        "(%.1f Hz | conf≥%.2f | cluster≥%d | vicinity≤%.0f m)",
        poll_hz, _confidence_threshold, _cluster_count_threshold, _vicinity_threshold_m,
    )

    while True:
        t0 = time.monotonic()
        try:
            process_once()
        except Exception as exc:
            log.exception("Unhandled error in process_once: %s", exc)
        time.sleep(max(0.0, interval - (time.monotonic() - t0)))



if __name__ == "__main__":
    try:
        run_loop()
    except KeyboardInterrupt:
        log.info("Triangulation processor stopped by user.")