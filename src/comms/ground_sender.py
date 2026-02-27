#!/usr/bin/env python3
from pymavlink import mavutil
import time

PX4_UDP = "udpout:127.0.0.1:14550"

CMD_MAP = {
    "start_log": 31000,
    "stop_log":  31001,
}

def send_cmd(m, cmd_id: int):
    m.mav.command_long_send(
        1, 1, cmd_id, 0,
        1, 0, 0, 0, 0, 0, 0
    )


def main():
    print("Connecting to PX4...")
    m = mavutil.mavlink_connection(PX4_UDP)
    time.sleep(0.5)

    print("Ground Console Connected.")
    print("Available commands:", ", ".join(CMD_MAP.keys()))
    print("Type 'stop' to exit.\n")

    while True:
        s = input("> ").strip().lower()

        if s in ("stop", "exit"):
            print("Exiting.")
            break

        if s == "help":
            print("Commands:", ", ".join(CMD_MAP.keys()))
            continue

        if s not in CMD_MAP:
            print("Unknown command. Type 'help'.")
            continue

        cmd_id = CMD_MAP[s]
        send_cmd(m, cmd_id)
        print(f"Sent {s.upper()} ({cmd_id})")

if __name__ == "__main__":
    main()