#!/usr/bin/env python3
from pymavlink import mavutil

def main():
    print("Listening for COMMAND_LONG on 14601...")
    m = mavutil.mavlink_connection('udpin:127.0.0.1:14604')

    while True:
        # Filter strictly for COMMAND_LONG
        msg = m.recv_match(type='COMMAND_LONG', blocking=True)
        if msg:
            print(f"\n[COMMAND RECEIVED]")
            print(f"Command ID: {msg.command}")
            print(f"Target Sys: {msg.target_system}, Target Comp: {msg.target_component}")
            print(f"Params: [{msg.param1}, {msg.param2}, {msg.param3}, ...]")


if __name__ == "__main__":
    main()

