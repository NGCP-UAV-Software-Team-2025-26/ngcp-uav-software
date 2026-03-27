"""
kraken_and_telemetry_processor.py

Progressive triangulation processor for Kraken DOA + telemetry fusion data.

Algorithm
---------
1. Read only *new* lines appended to the fusion JSONL log since the last cycle.
2. Filter incoming records by CONFIDENCE_THRESHOLD; add survivors to `_memory`.
3. After each ingestion pass, scan `_memory` for a cluster:
    - A cluster is any anchor record that has >= CLUSTER_COUNT_THRESHOLD neighbours (including itself) all within VICINITY_THRESHOLD_M metres
4.  When a cluster is found:
      a. Compute a confidence-weighted centroid of the cluster members
      b. If a previous estimate exists, blend it in as an additional weighted data point so each fix refines the last
      c. Write the result to state["target_fix"] and increment the fix_id
      d. Clear `_memory` entirely; retain the estimate for future blending
5.  Repeat at STATE_POLL_HZ

Thresholds:
    CONFIDENCE_THRESHOLD: minimum confidence_0_1 to enter memory
    CLUSTER_COUNT_THRESHOLD: how many high-conf coords must sit in vicinity
    VICINITY_THRESHOLD_M: radius (metres) that defines "close"
"""

# might need to add a change where the vicinity threshold automatically decreases as fix_id increases (500m "fix 1-10">300m "fix 10-50">100m "fix 50-90">30 "fix 90+")
# Alternative, these automatic changes prob depend on whether or not estimate is close to previous estimate by proximity; if so then close up threshold, if not, then keep current threshold
# same can go for confidence threshold

import sys
import json
import time
import math
import logging
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.state_utils import load_state, update_state, STATE_FILE
STATE_POLL_HZ = 2.0

# Thresholds; modify these as needed 
CONFIDENCE_THRESHOLD    = 0.50   # float [0, 1]  (records below this are ignored)
CLUSTER_COUNT_THRESHOLD = 5      # int           (min coords in vicinity to trigger)
VICINITY_THRESHOLD_M    = 500.0  # float (m)     (neighbourhood radius for clustering)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("fusion_processor")


# processor state ; persists across process_once() calls
_memory: list[dict]         = []    # high-confidence records not yet consumed
_current_estimate: dict | None = None  # last published target_fix
_fix_id: int                = 0     # increments with every published fix
_fusion_log_offset: int     = 0     # byte offset – only read lines written after this


# geometry helpers

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    #Great-circle distance in metres between two (lat, long) points
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# fusion log helpers

def read_new_fusion_records(path: str | Path) -> list[dict]:
    # Open the fusion JSONL log, seek to the last-known byte offset, and return only the records written since the previous call. 
    # Malformed lines are skipped. `_fusion_log_offset` is advanced on success.
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
                log.debug("Skipping malformed line at offset %d", _fusion_log_offset)
        _fusion_log_offset = fh.tell()

    if records:
        log.debug("Ingested %d new record(s); log offset now %d B", len(records), _fusion_log_offset)
    return records


def filter_high_confidence(records: list[dict]) -> list[dict]:
    # Keep only records whose confidence_0_1 meets CONFIDENCE_THRESHOLD
    return [r for r in records if r.get("confidence_0_1", 0.0) >= CONFIDENCE_THRESHOLD]


# clustering

def find_cluster(memory: list[dict]) -> list[dict] | None:
    # Scan *memory* for the first anchor whose neighbourhood contains at least CLUSTER_COUNT_THRESHOLD records (including itself) within VICINITY_THRESHOLD_M.
    # Returns the list of cluster members, or None if no cluster is found yet.
    # The returned cluster is sorted by descending confidence so the highest-confidence record is always first.
    for anchor in memory:
        a_lat = anchor["lat_deg"]
        a_lon = anchor["lon_deg"]
        neighbours = [
            r for r in memory
            if haversine_m(a_lat, a_lon, r["lat_deg"], r["lon_deg"]) <= VICINITY_THRESHOLD_M
        ]
        if len(neighbours) >= CLUSTER_COUNT_THRESHOLD:
            neighbours.sort(key=lambda r: r.get("confidence_0_1", 0.0), reverse=True)
            return neighbours

    return None


# estimate computation 

def compute_weighted_estimate(
    cluster: list[dict],
    previous: dict | None,
    next_fix_id: int,
) -> dict:
    
    # Compute a confidence-weighted centroid from *cluster* members, optionally blending in the *previous* estimate as an additional weighted data point.

    # Weighted centroid
    #     lat  = sum(lat_i * conf_i) / sum(conf_i)
    #     lon  = sum(lon_i * conf_i) / sum(conf_i)
    #     alt  = sum(alt_i * conf_i) / sum(conf_i)   (members with alt only)
    #     confidence = sum(conf_i) / n               (mean of all contributing weights)

    # Params
    # cluster: list of raw fusion records forming the spatial cluster
    # previous: the last published target_fix dict (lat/lon/confidence keys), or None on the very first fix
    # next_fix_id: the fix_id to stamp on the new estimate
    
    # Build a unified list of (lat, lon, alt, confidence) tuples
    contributors: list[tuple[float, float, float | None, float]] = []

    for r in cluster:
        contributors.append((
            r["lat_deg"],
            r["lon_deg"],
            r.get("altitude_rel_ft"),
            r.get("confidence_0_1", 0.0),
        ))

    # Blend previous estimate as an additional data point
    if previous is not None:
        contributors.append((
            previous["lat"],
            previous["lon"],
            previous.get("alt"),
            previous["confidence"],
        ))

    total_weight = sum(c[3] for c in contributors)
    if total_weight == 0:
        total_weight = 1e-9  # guard against zero division

    w_lat = sum(c[0] * c[3] for c in contributors) / total_weight
    w_lon = sum(c[1] * c[3] for c in contributors) / total_weight

    # Weighted altitude (only from contributors that have one)
    alt_contributors = [(c[2], c[3]) for c in contributors if c[2] is not None]
    if alt_contributors:
        alt_weight = sum(a[1] for a in alt_contributors)
        w_alt = sum(a[0] * a[1] for a in alt_contributors) / alt_weight if alt_weight else None
    else:
        w_alt = None

    w_confidence = total_weight / len(contributors)  # mean contribution weight

    return {
        "lat":        w_lat,
        "lon":        w_lon,
        "alt":        w_alt,
        "confidence": w_confidence,
        "fix_id":     next_fix_id,
        "timestamp":  time.time(),
    }


# single processing cycle

def process_once() -> None:
    # Execute one full ingest  -> cluster-check -> (optional) publish cycle

    # State transitions
    # Always: read new lines -> filter -> append to _memory
    # If cluster found: compute estimate -> update_state -> clear _memory -> retain estimate
    
    global _memory, _current_estimate, _fix_id

    # get fusion path from mission state
    state = load_state()
    fusion_log_path: str | None = state.get("fusion_log")
    if not fusion_log_path:
        log.warning("State has no 'fusion_log' entry; skipping cycle.")
        return

    #Ingest only new records appended since last cycle
    new_records = read_new_fusion_records(fusion_log_path)
    high_conf   = filter_high_confidence(new_records)

    if high_conf:
        _memory.extend(high_conf)
        log.info(
            "Memory: +%d high-conf record(s)  (threshold=%.2f)  total in memory: %d",
            len(high_conf),
            CONFIDENCE_THRESHOLD,
            len(_memory),
        )

    # check whether memory now contains a qualifying spatial cluster
    cluster = find_cluster(_memory)
    if cluster is None:
        log.debug(
            "No cluster yet  (need %d within %.0f m, have %d in memory)",
            CLUSTER_COUNT_THRESHOLD,
            VICINITY_THRESHOLD_M,
            len(_memory),
        )
        return

    #Cluster found, compute new estimate
    _fix_id += 1
    estimate = compute_weighted_estimate(cluster, _current_estimate, _fix_id)

    log.info(
        "Cluster found  (%d members) → fix #%d  "
        "lat=%.6f  lon=%.6f  confidence=%.4f",
        len(cluster),
        _fix_id,
        estimate["lat"],
        estimate["lon"],
        estimate["confidence"],
    )

    # Publish to mission state
    update_state("target_fix", estimate)

    # Advance internal state: clear memory, retain estimate for next blend
    _current_estimate = estimate
    _memory.clear()
    log.info("Memory cleared.  Estimate retained for next-round blending.")


# continuous loop

def run_loop(poll_hz: float = STATE_POLL_HZ) -> None:
    # Drive process_once() at *poll_hz* Hz until KeyboardInterrupt.
    # Exceptions inside process_once() are caught and logged so the loop stays alive across transient errors (file locks, partial writes, etc)
    
    interval = 1.0 / poll_hz
    log.info(
        "Fusion processor starting  (%.1f Hz | "
        "conf≥%.2f | cluster≥%d | vicinity≤%.0f m)",
        poll_hz,
        CONFIDENCE_THRESHOLD,
        CLUSTER_COUNT_THRESHOLD,
        VICINITY_THRESHOLD_M,
    )

    while True:
        cycle_start = time.monotonic()
        try:
            process_once()
        except Exception as exc:
            log.exception("Unhandled error in process_once: %s", exc)

        elapsed   = time.monotonic() - cycle_start
        sleep_for = max(0.0, interval - elapsed)
        time.sleep(sleep_for)



if __name__ == "__main__":
    try:
        run_loop()
    except KeyboardInterrupt:
        log.info("Fusion processor stopped by user.")