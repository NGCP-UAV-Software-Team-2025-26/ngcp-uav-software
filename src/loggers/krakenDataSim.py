import time
import random
import os
import json

# ==========================
# CONFIGURATION
# ==========================
OUTPUT_DIR = "C:\MYDOWNLOADS"      # Change directory the wherever its needed
OUTPUT_FILE = "doa_log.jsonl"
RATE_HZ = 10
INTERVAL = 1.0 / RATE_HZ

filepath = os.path.join(OUTPUT_DIR, OUTPUT_FILE)

# Ensure directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ==========================
# DATA GENERATION LOOP
# ==========================
with open(filepath, "a") as f:
    while True:
        epoch = float(int(time.time() * 1000))

        record = {
            "epoch": epoch,
            "doa": float(random.randint(0, 359)),
            "confidence": random.uniform(0, 99),
            "lat": float(random.randint(0, 99)),
            "lon": float(random.randint(0, 99)),
            "gps_heading": float(random.randint(0, 359))
        }

        f.write(json.dumps(record) + "\n")
        f.flush()

        time.sleep(INTERVAL)