#!/usr/bin/env python3
import time
import json
import requests
from pathlib import Path

# Kraken DOA endpoint (LOCALHOST on Pi)
DOA_URL = "http://127.0.0.1:8081/DOA_value.html"
# Polling rate (seconds)
UPDATE_RATE = 0.1  # 10 Hz or match Kraken update rate

BASE_DIR = Path(__file__).resolve().parents[2]
LOG_DIR = BASE_DIR / "logs" / "kraken"
LOG_DIR.mkdir(parents=True, exist_ok=True)

# Output JSON log file (JSON Lines format)
OUT_FILE = LOG_DIR / f"doa_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"




count=-1
def log_once():
    global count
    
    try:
        t_rx = time.time() #Pi receipt time

        r = requests.get(DOA_URL, timeout=1)
        line = r.text.strip()
        fields = line.split(',')
        # Build JSON object with selected fields
        entry = {
            "t_rx": t_rx,
            "epoch": float(fields[0]),
            "doa": float(fields[1]),               # unit-circle DoA (0°=East CCW)
            "confidence": float(fields[2]),
            "lat": float(fields[9]),
            "lon": float(fields[10]),
            "gps_heading": float(fields[11]),
            #"compass_heading": float(fields[12]),
        }

        # Append JSON object to file
        with open(OUT_FILE, "a") as f:
            if count != float(fields[0]):
                print(count)
                print(float(fields[0]))
                f.write(json.dumps(entry) + "\n")
                count=float(fields[0])

    except Exception as e:
        print("Error:", e)
    

def main():
    while True:
        log_once()
        time.sleep(UPDATE_RATE)


if __name__ == "__main__":
    main()



