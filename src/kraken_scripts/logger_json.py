#!/usr/bin/env python3
import time
import json
import requests

# Kraken DOA endpoint (LOCALHOST on Pi)
DOA_URL = "http://127.0.0.1:8081/DOA_value.html"

# Output JSON log file (JSON Lines format)
OUT_FILE = "/home/ngcp25/kraken_logs/doa_log.jsonl"

# Polling rate (seconds)
UPDATE_RATE = 0.1  # 10 Hz or match Kraken update rate


def log_once():
    try:
        r = requests.get(DOA_URL, timeout=1)
        line = r.text.strip()
        fields = line.split(',')

        # Build JSON object with selected fields
        entry = {
            "epoch": float(fields[0]),
            "doa": float(fields[1]),               # unit-circle DoA (0°=East CCW)
            "confidence": float(fields[2]),
            "lat": float(fields[9]),
            "lon": float(fields[10]),
            "gps_heading": float(fields[11]),
          #  "compass_heading": float(fields[12]),
        }

        # Append JSON object to file
        with open(OUT_FILE, "a") as f:
            f.write(json.dumps(entry) + "\n")

    except Exception as e:
        print("Error:", e)


def main():
    while True:
        log_once()
        time.sleep(UPDATE_RATE)


if __name__ == "__main__":
    main()


