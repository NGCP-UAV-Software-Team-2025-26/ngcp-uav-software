#!/usr/bin/env python3
from pymavlink import mavutil
import time

#PX4_UDP = "udpout:127.0.0.1:14550"

RFD_PORT = "/dev/ttyUSB0"   # adjust if needed
RFD_BAUD = 57600            # must match your radio/TELEM1

# CMD_MAP = {
#     "start_log": 31000,
#     "stop_log":  31001,
# }

# def send_cmd(m, cmd_id: int):
#     m.mav.command_long_send(
#         1, 1, cmd_id, 0,
#         1, 0, 0, 0, 0, 0, 0
#     )
TEXT_MAP = {
    "start_log": "NGCP:START_LOG",
    "stop_log":  "NGCP:STOP_LOG",
}

def send_text(m, text: str):
    # MAVLink STATUSTEXT has max ~50 chars depending on dialect/version
    m.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_NOTICE,
        text.encode("utf-8")
    )

def main():
    print(f"Connecting to RFD on {RFD_PORT} @ {RFD_BAUD}...")
    m = mavutil.mavlink_connection(RFD_PORT, baud=RFD_BAUD)

    # Wait for PX4 heartbeat so we know the link is real
    hb = m.wait_heartbeat(timeout=10)
    if not hb:
        print("No heartbeat from vehicle. Check radio link/baud.")
        return
    print("Heartbeat received. Ground console ready.")
    print("Available commands:", ", ".join(TEXT_MAP.keys()))
    print("Type 'stop' to exit.\n")

    while True:
        s = input("> ").strip().lower()

        if s in ("stop", "exit"):
            print("Exiting.")
            break

        if s == "help":
            print("Commands:", ", ".join(TEXT_MAP.keys()))
            continue

        if s not in TEXT_MAP:
            print("Unknown command. Type 'help'.")
            continue

        text = TEXT_MAP[s]
        send_text(m, text)
        print(f"Sent STATUSTEXT: {text}")
        
if __name__ == "__main__":
    main()