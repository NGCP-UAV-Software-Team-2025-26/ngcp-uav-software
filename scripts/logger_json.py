#!/usr/bin/env python3
import time
import json
import requests
import os

# Kraken DOA endpoint (LOCALHOST on Pi)
DOA_URL = "http://127.0.0.1:8081/DOA_value.html"

# Output JSON log file (JSON Lines format)
OUT_FILE = "/home/ngcp25/kraken_logs/doa_log.jsonl"

# Ensure output directory exists
os.makedirs(os.path.dirname(OUT_FILE), exist_ok=True)

# Polling rate (seconds)
UPDATE_RATE = 0.1  # 10 Hz or match Kraken update rate

count=-1
def log_once():
    global count
    
    try:
        r = requests.get(DOA_URL, timeout=1)
        line = r.text.strip()
        if line != '':
            fields = line.split(',')
            # Build JSON object with selected fields
            entry = {
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
        else:
            print("no incoming signal yet")

    except Exception as e:
        print("Error:", e)

def krakenOnline() -> bool:
    print("Connecting to KrakenSDR server");
    try:
        response = requests.get("http://127.0.0.1:8080/", timeout=2)
        print(response.status_code)
        return response.status_code == 200
    except requests.RequestException:
        return False

def main():
    while True:
        if krakenOnline():
            break
        print("Connection attempt failed, retrying")
        time.sleep(UPDATE_RATE)
    print("KrakenSDR server online\n")
    while True:
        log_once()
        time.sleep(UPDATE_RATE)


if __name__ == "__main__":
    main()



