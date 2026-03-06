#!/usr/bin/env python3
import json
import time
from pathlib import Path

from pymavlink import mavutil


LISTEN_URI = "udpin:0.0.0.0:14601"

TXT_START_LOG = "NGCP:START_LOG"
TXT_STOP_LOG = "NGCP:STOP_LOG"

BASE_DIR = Path(__file__).resolve().parents[2]
STATE_DIR = BASE_DIR / "state"
STATE_DIR.mkdir(parents=True, exist_ok=True)

STATE_FILE = STATE_DIR / "mission_state.json"

def load_state() -> dict:
    if STATE_FILE.exists():
        try:
            return json.loads(STATE_FILE.read_text())
        except Exception:
            pass
    return {
        "logging_enabled": False,
        "last_command": None,
        "timestamp": None,
        "last_sender_sysid": None,
        "last_sender_compid": None,
    }
#Saves system state and writes it to the JSON
def save_state(state: dict) -> None:
    STATE_FILE.write_text(json.dumps(state, indent=2))

#Modift and save
def update_state(logging_enabled: bool, cmd_name: str, src_sys: int, src_comp: int) -> None:
    state = load_state()
    state["logging_enabled"] = logging_enabled
    state["last_command"] = cmd_name
    state["timestamp"] = time.time()
    state["last_sender_sysid"] = src_sys
    state["last_sender_compid"] = src_comp
    save_state(state)

def _decode_statustext(msg) -> str:
    t = msg.text
    if isinstance(t, (bytes, bytearray)):
        return t.decode("utf-8", errors="ignore").strip("\x00").strip()
    return str(t).strip("\x00").strip()

def main():
    #Logs
    print()
    print(f"Listening for MAVLink on {LISTEN_URI} ...")
    print(f"State file: {STATE_FILE}")
    print()
    print(f"Commands: START_LOG={TXT_START_LOG}, STOP_LOG={TXT_STOP_LOG}")
    print("Waiting for STATUSTEXT\n")

    #Makes sure file exists
    if not STATE_FILE.exists():
        save_state(load_state())
        print(f"[STATE] Initialized {STATE_FILE}\n")

    m = mavutil.mavlink_connection(LISTEN_URI)

    print("Waiting for heartbeat to confirm MAVLink link")
    m.wait_heartbeat(timeout=30)
    print(f"Heartbeat received. Listening for COMMAND_LONG messages...\n")

    # Temporarily replace recv_match line with this:
    
    while True:
        msg = m.recv_match(type="COMMAND_LONG", blocking=True, timeout=5)

        if msg is None:
            continue
        
        cmd = int(msg.command)
        src_sys = msg.get_srcSystem()
        src_comp = msg.get_srcComponent()
        text = _decode_statustext(msg)

        #Print Got command_long 
        print(f"COMMAND_LONG cmd={cmd} from sys={src_sys} comp={src_comp}")

        print(f"STATUSTEXT '{text}' from sys={src_sys} comp={src_comp}")

        if text == TXT_START_LOG:
            update_state(True, "START_LOG", src_sys, src_comp)
            print("START_LOG applied (state updated)\n")

        elif text == TXT_STOP_LOG:
            update_state(False, "STOP_LOG", src_sys, src_comp)
            print("STOP_LOG applied (state updated)\n")

        else:
            print(f"Unrecognised command id={cmd}, ignoring.\n")


if __name__ == "__main__":
    main()