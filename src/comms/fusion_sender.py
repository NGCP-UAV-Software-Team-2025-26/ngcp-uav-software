#!/usr/bin/env python3
"""
fusion_sender.py

Reads new records from the live fusion JSONL and sends each one over the
existing MAVLink/RFD900x link as chunked STATUSTEXT messages.

Each STATUSTEXT text field (50 chars) carries:
  F{seq:02d}{chunk_idx:01d}{total:01d}:{payload}
  └─ 6-char header, 44 chars of JSON payload per message
"""

import json
import os
import time
from pathlib import Path
import sys

from pymavlink import mavutil

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.state_utils import load_state

RFD_PORT         = os.environ.get("RFD_PORT", "udp:127.0.0.1:14601")
RFD_BAUD         = int(os.environ.get("RFD_BAUD", 57600))
POLL_INTERVAL_S  = 0.1
CHUNK_PAYLOAD    = 44   # bytes of JSON per STATUSTEXT (50 - 6 header chars)


def send_record(mav, record: dict, seq: int) -> None:
    raw    = json.dumps(record, separators=(',', ':')).encode()
    chunks = [raw[i:i+CHUNK_PAYLOAD] for i in range(0, len(raw), CHUNK_PAYLOAD)]
    total  = len(chunks)

    for idx, chunk in enumerate(chunks):
        text = f"F{seq%100:02d}{idx:01d}{total:01d}:{chunk.decode()}"
        mav.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_DEBUG,
            text.encode().ljust(50, b'\x00')
        )


def main():
    print(f"[fusion_sender] Connecting → {RFD_PORT}")
    mav = mavutil.mavlink_connection(RFD_PORT, baud=RFD_BAUD, source_system=200)

    active_path:   Path | None = None
    last_seq_sent: int         = -1

    while True:
        state           = load_state()
        fusion_path_str = state.get("fusion_log")

        if not fusion_path_str:
            time.sleep(1.0)
            continue

        fusion_path = Path(fusion_path_str)

        if fusion_path != active_path:
            print(f"[fusion_sender] New session: {fusion_path.name}")
            active_path   = fusion_path
            last_seq_sent = -1

        if not fusion_path.exists():
            time.sleep(0.5)
            continue

        try:
            with open(fusion_path, encoding='utf-8') as f:
                records = [json.loads(l) for l in f if l.strip()]
        except OSError:
            time.sleep(POLL_INTERVAL_S)
            continue

        new_records = sorted(
            [r for r in records if r.get("kraken_seq", -1) > last_seq_sent],
            key=lambda r: r.get("kraken_seq", 0)
        )

        for record in new_records:
            send_record(mav, record, record["kraken_seq"])
            last_seq_sent = record["kraken_seq"]
            print(f"[fusion_sender] sent seq={last_seq_sent} "
                  f"usable={record.get('usable_for_triangulation')}")

        time.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
    main()