#!/usr/bin/env python3
from pymavlink import mavutil
import time



RFD_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HL2Y-if00-port0"
#RFD_PORT = "udpin:0.0.0.0:14602" # for SITL
RFD_BAUD = 57600            # must matc                                                                                 h your radio/TELEM1


CMD_START_LOG = 31000                                   
CMD_STOP_LOG  = 31001

ACK_TIMEOUT = 3

#Target system/component to target PX4 directly (1,1)
TARGET_SYSTEM    = 1
TARGET_COMPONENT = 191

CMD_MAP = {
    "start_log": CMD_START_LOG,
    "stop_log":  CMD_STOP_LOG,
}

MAV_RESULT = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}

def send_command_long(m, cmd_id: int) -> None:
    m.mav.command_long_send(
        TARGET_SYSTEM,    #target_system
        TARGET_COMPONENT, #target_component
        cmd_id,           #command
        0,                #confirmation (0 = first transmission)
        1,                #param1 — use as a simple flag if needed
        0, 0, 0, 0, 0, 0  #param2-7 unused
    )

def wait_for_ack(m, cmd_id: int, timeout: float = ACK_TIMEOUT) -> None:

    deadline = time.time() + timeout
    while time.time() < deadline:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack is None:
            continue
        if int(ack.command) != cmd_id:
            continue
        result_code = int(ack.result)
        result_name = MAV_RESULT.get(result_code, f"UNKNOWN({result_code})")
        src_sys  = ack.get_srcSystem()
        src_comp = ack.get_srcComponent()
        print(f"[ACK] cmd={cmd_id} result={result_name} ({result_code}) "
            f"from sys={src_sys} comp={src_comp}")
        return

    print(f"[ACK] No ACK received for cmd={cmd_id} within {timeout}s "
        f"(radio link issue, or listener does not send ACKs)")
    
def main():
    # print(f"Connecting to RFD on {RFD_PORT} @ {RFD_BAUD}...")
    # m = mavutil.mavlink_connection(RFD_PORT, baud=RFD_BAUD)

    print(f"Connecting to RFD on {RFD_PORT}")
    m = mavutil.mavlink_connection(RFD_PORT, baud=RFD_BAUD)

    print("Waiting for heartbeat (timeout 30s)")
    hb = m.wait_heartbeat(timeout=30)
    if not hb:
        print("ERROR: No heartbeat received. mavlink-router and upstream serial link.")
        return

    print(f"Heartbeat received from system={m.target_system} component={m.target_component}")

    # NEW: Update global targets based on the heartbeat
    global TARGET_SYSTEM
    TARGET_SYSTEM = m.target_system
    print("Ground console ready.\n")
    print()
    print("Available commands:", ", ".join(CMD_MAP.keys()))
    print("Type 'exit' to quit.\n")

    while True:
        try:
            s = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if s in ("exit", "quit"):
            print("Exiting.")
            break

        if s not in CMD_MAP:
            print(f"Unknown command '{s}'. Type 'help' for available commands.")
            continue

        cmd_id = CMD_MAP[s]
        send_command_long(m, cmd_id)
        print(f"Sent COMMAND_LONG: {s.upper()} (cmd_id={cmd_id})")


if __name__ == "__main__":
    main()