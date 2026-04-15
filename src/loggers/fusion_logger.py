import argparse
import bisect
import json
import time
from pathlib import Path

import sys
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import load_state, update_state

DEFAULT_MAX_TIME_DIFF_MS     = 250 #Difference in telemetry t_rx_ms and kraken t_rx_ms (Can't be over 250 cuz that means its too far apart)
DEFAULT_MIN_CONFIDENCE       = 0.3
DEFAULT_MAX_ROLL_DEG         = 60.0
DEFAULT_MIN_GROUND_SPEED_FT_S = 0.0

SCRIPT_NAME = "fusion_logger.py"

BASE_DIR   = Path(__file__).resolve().parents[2]


FUSION_DIR = BASE_DIR / "logs" / "fusion"
FUSION_DIR.mkdir(parents=True, exist_ok=True)

# How often (seconds) the idle loop polls mission_state for a new telemetry file.
IDLE_POLL_INTERVAL_S   = 1.0
# How often (seconds) an active fusion session re-reads both logs and rewrites the fusion output. 
ACTIVE_POLL_INTERVAL_S = 0.5


def load_jsonl(path: Path) -> list[dict]:
    records = []
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as e:
                print(f"{path.name}:{lineno} — skipping bad JSON: {e}")
    return records

def build_timestamp_index(records: list[dict]) -> list[int]:
    return [r["t_rx_ms"] for r in records]


def find_nearest(timestamps: list[int], target: int) -> int:
    n = len(timestamps)
    if n == 0:
        raise ValueError("timestamps list is empty")

    pos = bisect.bisect_left(timestamps, target)

    if pos == 0:
        return 0
    if pos == n:
        return n - 1

    
    before = pos - 1
    after  = pos
    if abs(timestamps[before] - target) <= abs(timestamps[after] - target):
        return before
    return after

#For triangulation to see if usable in confidence

def is_usable(
    confidence_0_1: float | None,
    roll_deg: float | None,
    ground_speed_ft_s: float | None,
    dt_ms: int,
    *,
    max_time_diff_ms: int,
    min_confidence: float,
    max_roll_deg: float,
    min_ground_speed_ft_s: float,
) -> bool:
    if abs(dt_ms) > max_time_diff_ms:
        print(f"FAIL dt_ms: {dt_ms}")
        return False
    if confidence_0_1 is None or confidence_0_1 < min_confidence:
        print(f"FAIL confidence: {confidence_0_1}")
        return False
    if roll_deg is None or abs(roll_deg) > max_roll_deg:
        print(f"FAIL roll_deg: {roll_deg}")
        return False
    if ground_speed_ft_s is None or ground_speed_ft_s < min_ground_speed_ft_s:
        print(f"FAIL grsound_speed_ft_s: {ground_speed_ft_s}")
        return False
    return True

def fuse(
    kraken_records: list[dict],
    tel_records: list[dict],
    *,
    max_time_diff_ms: int,
    min_confidence: float,
    max_roll_deg: float,
    min_ground_speed_ft_s: float,
) -> list[dict]:
    
    # For each Kraken record, find the nearest telemetry record by t_rx_ms.
    # Don't use if abs(dt_ms) > max_time_diff_ms.
    # Returns a list of fused records.
    
    tel_timestamps = build_timestamp_index(tel_records)
    fused = []

    for k in kraken_records:
        k_ts = k["t_rx_ms"]
        idx  = find_nearest(tel_timestamps, k_ts)
        t    = tel_records[idx]

        dt_ms = t["t_rx_ms"] - k_ts

              

       
        if abs(dt_ms) > max_time_diff_ms:
            continue

        confidence    = k.get("confidence_0_1")
        roll          = t.get("roll_deg")
        ground_speed  = t.get("ground_speed_ft_s")

        record = {
            
            "run_id":           k.get("run_id"),
            "kraken_seq":       k.get("seq"),
            "telemetry_seq":    t.get("seq"),
            "t_rx_ms":          k_ts, #Kraken time is the fusion anchor
            "dt_ms":            dt_ms,#Telemetry.t_rx_ms - kraken.t_rx_ms

            
            "doa_deg":          k.get("doa_deg"),
            "confidence_0_1":   confidence,

           
            "lat_deg":          t.get("lat_deg"),
            "lon_deg":          t.get("lon_deg"),
            "altitude_rel_ft":  t.get("altitude_rel_ft"),
            "yaw_deg":          t.get("yaw_deg"),
            "pitch_deg":        t.get("pitch_deg"),
            "roll_deg":         roll,
            "ground_speed_ft_s": ground_speed,
            "vel_north_m_s":    t.get("vel_north_m_s"),
            "vel_east_m_s":     t.get("vel_east_m_s"),
            "vel_down_m_s":     t.get("vel_down_m_s"),

            
            "usable_for_triangulation": is_usable(
                confidence, roll, ground_speed, dt_ms,
                max_time_diff_ms=max_time_diff_ms,
                min_confidence=min_confidence,
                max_roll_deg=max_roll_deg,
                min_ground_speed_ft_s=min_ground_speed_ft_s,
            ),
            # "usable_for_triangulation": True,
        }
        fused.append(record)

    return fused


def main() -> None:
    parser = argparse.ArgumentParser(description="Fused Kraken DOA + telemetry logs.")
    parser.add_argument("--max_time_diff_ms",      type=int,   default=DEFAULT_MAX_TIME_DIFF_MS)
    parser.add_argument("--min_confidence",        type=float, default=DEFAULT_MIN_CONFIDENCE)
    parser.add_argument("--max_roll_deg",          type=float, default=DEFAULT_MAX_ROLL_DEG)
    parser.add_argument("--min_ground_speed_ft_s", type=float, default=DEFAULT_MIN_GROUND_SPEED_FT_S)
    args = parser.parse_args()

    active_session: tuple[str, str] | None = None
    out_file: Path | None = None
    run_id: str | None = None
    t_fusion_start_ms: int | None = None

    
    print("[fusion_logger] Waiting for kraken_log and telemetry_log in mission_state ...")
    


    # Main continuous loop 
    while True:
        # Re-read mission_state on every iteration so we always see the latest paths written by other loggers.
        state = load_state()

        kraken_path_str = state.get("kraken_log")
        tel_path_str    = state.get("telemetry_log")


        # Kraken log need valid path in state before fusing
        if not kraken_path_str:
            print("[fusion_logger] Waiting: kraken_log not in state yet …")
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue

        #Telemetry needs valid path
        if not tel_path_str:
            print("[fusion_logger] Waiting: telemetry_log not in state yet ...")
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue

        session = (kraken_path_str, tel_path_str)

        kraken_file = Path(kraken_path_str)
        tel_file    = Path(tel_path_str)   # tel_path_str is not None here

        # NEW-SESSION DETECTION, fires once when telemetry_log first changes (startup→new), and again any time it changes a second time mid-run (new session within the same process lifetime)
        # active_session tracks the (kraken_log, telemetry_log) pair being fused
        if session != active_session:
            # Extract run_id from the kraken filename, same as the original code.
            run_id = kraken_file.stem.replace("doa_", "")

            out_file = FUSION_DIR / f"fusion_{run_id}.jsonl"

            # Record start time of fusion session for meta file
            t_fusion_start_ms = int(time.time() * 1000)

            active_session = session

            print(f"\n[fusion_logger] New fusion session detected.")
            print(f"  run_id      : {run_id}")
            print(f"  Kraken      : {kraken_file.name}")
            print(f"  Telemetry   : {tel_file.name}")
            print(f"  Output      : {out_file.name}")

            update_state("fusion_log", str(out_file))

        #Makes sure it exists first
        if not kraken_file.exists():
            print(f"[fusion_logger] Waiting: Kraken file does not exist yet: {kraken_file}")
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue

        if not tel_file.exists():
            print(f"[fusion_logger] Waiting: Telemetry file does not exist yet: {tel_file}")
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue
        # re-read both files completely on every cycle
        # JSONL files are append-only, loading them fresh each time is the simplest way to pick up newly appended records
        kraken_records = load_jsonl(kraken_file)
        tel_records    = load_jsonl(tel_file)

        # Sort telemetry by timestamp in case records arrive out of order
        tel_records.sort(key=lambda r: r["t_rx_ms"])

        if not kraken_records or not tel_records:
            # One of the files is present but still empty
            print("[fusion_logger] Waiting: logs exist but no data yet")
            time.sleep(ACTIVE_POLL_INTERVAL_S)
            continue

        fused = fuse(
            kraken_records,
            tel_records,
            max_time_diff_ms=args.max_time_diff_ms,
            min_confidence=args.min_confidence,
            max_roll_deg=args.max_roll_deg,
            min_ground_speed_ft_s=args.min_ground_speed_ft_s,
        )

        n_usable    = sum(1 for r in fused if r["usable_for_triangulation"])
        n_discarded = len(kraken_records) - len(fused)

        # print(f"  Kraken samples         : {len(kraken_records)}")
        # print(f"  Discarded (dt too large): {n_discarded}")
        # print(f"  Fused records          : {len(fused)}")
        # print(f"  Usable for triangulation: {n_usable} / {len(fused)}")

        if out_file is None or run_id is None:
            time.sleep(IDLE_POLL_INTERVAL_S)
            continue

        # WRITE, overwrite the fusion file completely on each cycle
        with open(out_file, "w", encoding="utf-8") as f:
            for record in fused:
                f.write(json.dumps(record) + "\n")

        
        print(
            f"[fusion_logger] {time.strftime('%H:%M:%S')} — "
            f"kraken={len(kraken_records)} | tel={len(tel_records)} | "
            f"fused={len(fused)} | usable={n_usable} | discarded(dt)={n_discarded}"
        )

        #Meta to keep track of what parameters for fusion
        # updated each cycle
        meta = {
            "script":                  SCRIPT_NAME,
            "run_id":                  run_id,
            "kraken_file":             str(kraken_file),
            "telemetry_files":         [str(tel_file)],
            "fusion_file":             str(out_file),
            "fusion_time_ms":          t_fusion_start_ms,
            "kraken_records_in":       len(kraken_records),
            "telemetry_records_in":    len(tel_records),
            "fused_records_out":       len(fused),
            "discarded_dt_exceeded":   n_discarded,
            "usable_for_triangulation": n_usable,

            "max_time_diff_ms":        args.max_time_diff_ms,
            "min_confidence":          args.min_confidence,
            "max_roll_deg":            args.max_roll_deg,
            "min_ground_speed_ft_s":   args.min_ground_speed_ft_s,
        }
        meta_file = FUSION_DIR / f"fusion_{run_id}_meta.json"
        meta_file.write_text(json.dumps(meta, indent=2))

        # Sleep before the next fusion cycle
        time.sleep(ACTIVE_POLL_INTERVAL_S)


if __name__ == "__main__":
    main()