#!/usr/bin/env python3
import json
import time
from pathlib import Path


from pymavlink import mavutil
from state.state_utils import update_state, STATE_FILE

LISTEN_URI = "udpin:0.0.0.0:14601"


CMD_START_LOG = 31000
CMD_STOP_LOG  = 31001

def main():
   #Logs
   print()
   print(f"Listening for MAVLink on {LISTEN_URI} ...")
   print(f"State file: {STATE_FILE}")
   print()
   print(f"Commands: START_LOG={CMD_START_LOG}, STOP_LOG={CMD_STOP_LOG}")
   print("Waiting for COMMAND_LONG\n")


   #Makes sure file exists
   if not STATE_FILE.exists():
    update_state("logging_enabled", False)
    print(f"[STATE] Initialized {STATE_FILE}\n")


   m = mavutil.mavlink_connection(LISTEN_URI, source_system=1, source_component=191)

   print("Sending heartbeat to announce PI5")
   m.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0
   )

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


    #Print Got command_long
    print(f"COMMAND_LONG cmd={cmd} from sys={src_sys} comp={src_comp}")
    
    m.mav.command_ack_send(msg.command, 0, target_system=msg.get_srcSystem(), target_component=msg.get_srcComponent())
    
    #Updates
    if cmd == CMD_START_LOG:
        update_state("logging_enabled", True)
        update_state("last_command", "START_LOG")
        update_state("last_sender_sysid", src_sys)
        update_state("last_sender_compid", src_comp)
        update_state("timestamp", time.time())
        print("START_LOG applied (state updated)\n")


    elif cmd == CMD_STOP_LOG:
        update_state("logging_enabled", False)
        update_state("last_command", "STOP_LOG")
        update_state("last_sender_sysid", src_sys)
        update_state("last_sender_compid", src_comp)
        update_state("timestamp", time.time())
        print("STOP_LOG applied (state updated)\n")


    else:
        print(f"Unrecognised command id={cmd}, ignoring.\n")

if __name__ == "__main__":
   main()
