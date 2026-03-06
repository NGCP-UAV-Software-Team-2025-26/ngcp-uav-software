#!/usr/bin/env python3
import time
import json
import requests
from pathlib import Path

# Kraken DOA endpoint (LOCALHOST on Pi)
DOA_URL = "http://127.0.0.1:8081/DOA_value.html"
# Polling rate (seconds)
UPDATE_RATE = 0.1  # 10 Hz or match Kraken update rate
SCRIPT_NAME = "kraken_logger.py"


BASE_DIR = Path(__file__).resolve().parents[2]
LOG_DIR = BASE_DIR / "logs" / "kraken"
LOG_DIR.mkdir(parents=True, exist_ok=True)


START_STRUCT = time.localtime()
RUN_ID   = time.strftime("%Y%m%d_%H%M%S", START_STRUCT)
# Output JSON log file (JSON Lines format)
OUT_FILE = LOG_DIR / f"doa_{RUN_ID}.jsonl"
#META_FILE = LOG_DIR / f"doa_{RUN_ID}_meta.json"

# def write_meta() -> None:
#     meta = {
#         "script": SCRIPT_NAME,
#         "run_id":       RUN_ID,
#         "poll_rate_hz": 1.0 / UPDATE_RATE,
#         "doa_endpoint": DOA_URL,
#         "log_file":     str(OUT_FILE),
#         "t_start_ms":   int(time.time() * 1000),
#     }

#     META_FILE.write_text(json.dumps(meta, indent=2))
#     print(f"Meta written in {META_FILE}")


seq:            int   = 0
last_kraken_counter: float = -1 

def log_once(f):
    global seq, last_kraken_counter
    
    try:
        t_rx_ms = int(time.time() * 1000) #Pi receipt time

        r = requests.get(DOA_URL, timeout=1)
        line = r.text.strip()
        fields = line.split(',')
        
        kraken_counter = float(fields[0])

        if kraken_counter == last_kraken_counter:
            return
        
        # Build JSON object with selected fields
        entry = {
            "t_rx_ms": t_rx_ms,
            "run_id": RUN_ID,
            "seq": seq,
            "kraken_counter": kraken_counter,

            "doa_deg": float(fields[1]),               # unit-circle DoA (0°=East CCW)
            "confidence_0_1": float(fields[2]),

            "lat_deg": float(fields[9]),
            "lon_deg": float(fields[10]),
            "gps_heading_deg": float(fields[11]),
            #"compass_heading": float(fields[12]),
        }

        f.write(json.dumps(entry) + "\n")
        f.flush()

        last_kraken_counter = kraken_counter
        seq += 1

    except Exception as e:
        print("Error:", e)
    

def main():
    # write_meta()
    print(f"Logging in {OUT_FILE}  (run_id={RUN_ID})")

    with open(OUT_FILE, "a", encoding="utf-8") as f:
        while True:
            loop_start = time.time()
            log_once(f)
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, UPDATE_RATE - elapsed))


if __name__ == "__main__":
    main()
