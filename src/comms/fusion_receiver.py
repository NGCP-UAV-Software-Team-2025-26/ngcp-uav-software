#!/usr/bin/env python3
"""
fusion_receiver.py — reads framed fusion records from the RFD900x serial port
and re-emits each as a UDP JSON datagram for the kraken-triangulator.

Also optionally writes a local GCS-side fusion log.
"""
import json
import os
import socket
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

RFD_PORT           = os.environ.get("RFD_PORT",           "udp:127.0.0.1:14602")
TRIANGULATOR_HOST  = os.environ.get("TRIANGULATOR_HOST",  "127.0.0.1")
TRIANGULATOR_PORT  = int(os.environ.get("TRIANGULATOR_PORT", "14550"))

LOG_DIR = Path("logs/fusion_gcs")
LOG_DIR.mkdir(parents=True, exist_ok=True)


def main():
    print(f"[fusion_bridge] Listening on {RFD_PORT}")
    mav  = mavutil.mavlink_connection(RFD_PORT, baud=57600, input=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[fusion_bridge] Forwarding → {TRIANGULATOR_HOST}:{TRIANGULATOR_PORT}")

    log_path = LOG_DIR / f"fusion_gcs_{datetime.utcnow():%Y%m%d_%H%M%S}.jsonl"
    print(f"[fusion_bridge] GCS log → {log_path}")

    # { seq_int: { chunk_idx: payload_str, '_total': int } }
    buf: defaultdict = defaultdict(dict)
    count = 0

    with open(log_path, 'w', encoding='utf-8') as log_f:
        while True:
            msg = mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
            if msg is None:
                continue

            text = msg.text.rstrip('\x00').strip()

            # Only process our fusion messages
            if not text.startswith('F'):
                continue
            try:
                header, payload = text[1:5], text[6:]  # skip 'F', split at ':'
                # Parse: seq(2) chunk_idx(1) total(1)
                seq_byte   = int(text[1:3])
                chunk_idx  = int(text[3])
                total      = int(text[4])
                payload    = text[6:]                   # everything after 'F##NM:'
            except (ValueError, IndexError):
                continue

            buf[seq_byte][chunk_idx] = payload
            buf[seq_byte]['_total']  = total

            if len(buf[seq_byte]) - 1 == total:  # -1 for '_total' key
                raw = ''.join(buf[seq_byte][i] for i in range(total))
                del buf[seq_byte]

                # Prune any stale incomplete buffers
                stale = [k for k in list(buf) if isinstance(k, int)
                         and abs(k - seq_byte) > 10]
                for k in stale:
                    del buf[k]

                try:
                    record = json.loads(raw)
                except json.JSONDecodeError as e:
                    print(f"[bridge] Bad JSON: {e}")
                    continue

                record["t_gcs_rx_ms"] = int(time.time() * 1000)

                out = json.dumps(record, separators=(',', ':')).encode()
                sock.sendto(out, (TRIANGULATOR_HOST, TRIANGULATOR_PORT))

                log_f.write(json.dumps(record) + '\n')
                log_f.flush()

                count += 1
                print(f"[bridge] {time.strftime('%H:%M:%S')} "
                      f"seq={record.get('kraken_seq')} "
                      f"usable={record.get('usable_for_triangulation')} "
                      f"total={count}")


if __name__ == "__main__":
    main()