"""
kraken_and_telemetry_processor.py

Processes fused Kraken DOA + telemetry data and updates the mission state
with the best target fix derived from the fusion log.

Pipeline:
    1. Poll mission state for the current fusion_log path.
    2. Read all records from the JSONL fusion log.
    3. Select the record with the highest confidence score.
    4. Write the result into state["target_fix"].

Run continuously at STATE_POLL_HZ, or call process_once() directly.
"""

import sys
import json
import time
import logging
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.state_utils import load_state, update_state, STATE_FILE
STATE_POLL_HZ = 2.0


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("fusion_processor")


#fusion log helpers

def read_fusion_log(path: str | Path) -> list[dict]:
    """
    Read every line from a JSONL fusion log and return a list of records.
    Silently skips malformed lines.
    """
    records: list[dict] = []
    fusion_path = Path(path)

    if not fusion_path.exists():
        log.warning("Fusion log not found: %s", fusion_path)
        return records

    with fusion_path.open("r", encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                log.debug("Skipping malformed line %d: %s", lineno, exc)

    log.info("Read %d records from %s", len(records), fusion_path.name)
    return records


def select_best_record(records: list[dict]) -> dict | None:
    """
    Return the record with the highest confidence_0_1 score.
    Returns None when the list is empty.
    """
    if not records:
        return None
    return max(records, key=lambda r: r.get("confidence_0_1", 0.0))


#state update

def build_target_fix(record: dict) -> dict:
    """
    Map a fusion record to the target_fix schema expected by mission_state.json.

    target_fix fields
    -----------------
    lat         : float | None
    lon         : float | None
    alt         : float | None   (altitude_rel_ft from fusion record)
    confidence  : float | None   (confidence_0_1)
    fix_id      : int   | None   (kraken_seq used as a stable identifier)
    timestamp   : float | None   (t_rx_ms converted to seconds)
    """
    return {
        "lat":        record.get("lat_deg"),
        "lon":        record.get("lon_deg"),
        "alt":        record.get("altitude_rel_ft"),
        "confidence": record.get("confidence_0_1"),
        "fix_id":     record.get("kraken_seq"),
        "timestamp":  (record["t_rx_ms"] / 1_000.0) if "t_rx_ms" in record else None,
    }


def push_target_fix(fix: dict) -> None:
    """Write target_fix into the mission state file."""
    update_state("target_fix", fix)
    log.info(
        "target_fix updated  fix_id=%s  confidence=%.4f  lat=%.6f  lon=%.6f",
        fix.get("fix_id"),
        fix.get("confidence") or 0.0,
        fix.get("lat") or 0.0,
        fix.get("lon") or 0.0,
    )


#single processing cycle

def process_once() -> None:
    """
    Execute one full read-select-update cycle.

    Steps
    -----
    1. Load current mission state.
    2. Resolve the fusion_log path from state.
    3. Read + parse the JSONL file.
    4. Pick the highest-confidence record.
    5. Update target_fix in state.
    """
    state = load_state()

    fusion_log_path: str | None = state.get("fusion_log")
    if not fusion_log_path:
        log.warning("State has no 'fusion_log' entry; skipping cycle.")
        return

    records = read_fusion_log(fusion_log_path)
    if not records:
        log.warning("No records loaded from fusion log; skipping cycle.")
        return

    best = select_best_record(records)
    if best is None:
        log.warning("select_best_record returned None; skipping cycle.")
        return

    fix = build_target_fix(best)
    push_target_fix(fix)


#loop

def run_loop(poll_hz: float = STATE_POLL_HZ) -> None:
    """
    Poll process_once() at *poll_hz* Hz until interrupted.

    Parameters
    ----------
    poll_hz : float
        Cycles per second.  Defaults to the module-level STATE_POLL_HZ.
    """
    interval = 1.0 / poll_hz
    log.info("Fusion processor starting  (%.1f Hz, interval=%.3fs)", poll_hz, interval)

    while True:
        cycle_start = time.monotonic()
        try:
            process_once()
        except Exception as exc:                       # keep loop alive on errors
            log.exception("Unhandled error in process_once: %s", exc)

        elapsed = time.monotonic() - cycle_start
        sleep_for = max(0.0, interval - elapsed)
        time.sleep(sleep_for)



if __name__ == "__main__":
    try:
        run_loop()
    except KeyboardInterrupt:
        log.info("Fusion processor stopped by user.")