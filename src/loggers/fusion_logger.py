import argparse
import bisect
import json
import time
from pathlib import Path

from state.state_utils import load_state, update_state

DEFAULT_MAX_TIME_DIFF_MS     = 250 #Difference in telemetry t_rx_ms and kraken t_rx_ms (Can't be over 250 cuz that means its too far apart)
DEFAULT_MIN_CONFIDENCE       = 0.5
DEFAULT_MAX_ROLL_DEG         = 30.0
DEFAULT_MIN_GROUND_SPEED_FT_S = 3.0

SCRIPT_NAME = "fusion_logger.py"

BASE_DIR   = Path(__file__).resolve().parents[2]


FUSION_DIR = BASE_DIR / "logs" / "fusion"
FUSION_DIR.mkdir(parents=True, exist_ok=True)


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
        return False
    if confidence_0_1 is None or confidence_0_1 < min_confidence:
        return False
    if roll_deg is None or abs(roll_deg) > max_roll_deg:
        return False
    if ground_speed_ft_s is None or ground_speed_ft_s < min_ground_speed_ft_s:
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
    t_fusion_start_ms = int(time.time() * 1000)

    state        = load_state()
    if not state.get("kraken_log"):
        print("[fusion_logger] ERROR: kraken_log not found in state file. Run kraken_logger first.")
        return
    if not state.get("telemetry_log"):
        print("[fusion_logger] ERROR: telemetry_log not found in state file. Run telemetry_logger first.")
        return
    kraken_file  = Path(state["kraken_log"])
    tel_files    = [Path(state["telemetry_log"])]
    run_id       = kraken_file.stem.replace("doa_", "")

    print(f"\n[fusion_logger] run_id = {run_id}")
    print(f"  Kraken  : {kraken_file.name}")
    print(f"  Telemetry : {tel_files[0].name}")
    
    print("\n[fusion_logger] Loading records ...")
    kraken_records = load_jsonl(kraken_file)
    print(f"  Kraken records loaded  : {len(kraken_records)}")

    #Merge all telemetry sessions and sort by t_rx_ms
    tel_records: list[dict] = []
    for tf in tel_files:
        tel_records.extend(load_jsonl(tf))
    tel_records.sort(key=lambda r: r["t_rx_ms"])
    print(f"  Telemetry records loaded: {len(tel_records)}")

    if not kraken_records:
        print("[fusion_logger] ERROR: Kraken log is empty. Aborting.")
        return
    if not tel_records:
        print("[fusion_logger] ERROR: Telemetry log is empty. Aborting.")
        return

    
    print("\n[fusion_logger] Fusing ...")
    fused = fuse(
        kraken_records,
        tel_records,
        max_time_diff_ms=args.max_time_diff_ms,
        min_confidence=args.min_confidence,
        max_roll_deg=args.max_roll_deg,
        min_ground_speed_ft_s=args.min_ground_speed_ft_s,
    )

    n_usable   = sum(1 for r in fused if r["usable_for_triangulation"])
    n_discarded = len(kraken_records) - len(fused)

    print(f"  Kraken samples         : {len(kraken_records)}")
    print(f"  Discarded (dt too large): {n_discarded}")
    print(f"  Fused records          : {len(fused)}")
    print(f"  Usable for triangulation: {n_usable} / {len(fused)}")

  
    out_file = FUSION_DIR / f"fusion_{run_id}.jsonl"
    with open(out_file, "w", encoding="utf-8") as f:
        for record in fused:
            f.write(json.dumps(record) + "\n")
    print(f"\n[fusion_logger] Wrote {len(fused)} records → {out_file}")
    update_state("fusion_log", str(out_file))

    #Meta to keep track of what parameters for fusion
    meta = {
        "script":                  SCRIPT_NAME,
        "run_id":                  run_id,
        "kraken_file":             str(kraken_file),
        "telemetry_files":         [str(tf) for tf in tel_files],
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
    print(f"Meta written in {meta_file}\n")


if __name__ == "__main__":
    main()
