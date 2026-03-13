import time
import random
import json
from pathlib import Path #Needs to import path

from state.state_utils import update_state #Makes it so this output updates mission-state.json
# ==========================
# CONFIGURATION
# ==========================

BASE_DIR   = Path(__file__).resolve().parents[2]
LOG_DIR    = BASE_DIR / "logs" / "kraken"
LOG_DIR.mkdir(parents=True, exist_ok=True)

RUN_ID   = time.strftime("%Y%m%d_%H%M%S")
filepath = LOG_DIR / f"simdoa_{RUN_ID}.jsonl"

update_state("kraken_log", str(filepath))
RATE_HZ = 10
INTERVAL = 1.0 / RATE_HZ


# ==========================
# DATA GENERATION LOOP
# ==========================

seq = 0

with open(filepath, "a") as f:
    while True:
        epoch = float(int(time.time() * 1000))
        record = {
            "t_rx_ms":        int(time.time() * 1000), 
            "run_id":         "SIM_TEST",               
            "seq":            seq,                       
            "kraken_counter": epoch,                    
            "doa_deg":        float(random.randint(0, 359)),  # was "doa"
            "confidence_0_1": random.uniform(0, 1),      # was "confidence", range was 0-99, should be 0-1
            "lat_deg":        34.0 + random.uniform(-0.01, 0.01),  # was "lat", more realistic
            "lon_deg":        -118.0 + random.uniform(-0.01, 0.01), # was "lon"
            "gps_heading_deg": float(random.randint(0, 359)), # was "gps_heading"
        }
        seq += 1


        f.write(json.dumps(record) + "\n")
        f.flush()

        time.sleep(INTERVAL)