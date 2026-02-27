import asyncio
import time
import json
from pathlib import Path
import os

from mavsdk import System


SYSTEM_ADDRESS = "udp://<SITL_IP>:14540"
SYSTEM_ADDRESS = os.getenv("MAVSDK_SYSTEM_ADDRESS", "udp://:14540")


BASE_DIR = Path(__file__).resolve().parents[2]
LOG_DIR = BASE_DIR / "logs" / "telemetry"
LOG_DIR.mkdir(parents=True, exist_ok=True)

#Checks here to see if it should start or not
STATE_FILE = BASE_DIR / "state" / "mission_state.json"


LOG_HZ = 5.0  #logging rate (Hz)
M_TO_FT = 3.280839895
MS_TO_FTS = 3.280839895

STATE_POLL_HZ = 2.0 #How often to check state file

def load_state() -> dict:
    
    try:
        if STATE_FILE.exists():
            return json.loads(STATE_FILE.read_text())
    except Exception:
        pass
    return {"logging_enabled": False}


async def wait_connected(drone: System):
    print(f"Connecting via {SYSTEM_ADDRESS} ...")
    await drone.connect(system_address=SYSTEM_ADDRESS)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4")
            return
        
async def wait_position_ok(drone: System):
    print("Waiting for global position + home position ...")
    print()
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK + Home OK")
            print()
            return
        
async def main():
    drone = System()
    await wait_connected(drone)
    await wait_position_ok(drone)

    latest = {
        "position": None,   #lat/lon/alt
        "vel_ned": None,    #north/east/down velocities
        "att_euler": None,  #roll/pitch/yaw in degrees
        "battery": None,    #remaining percent, voltage
    }

    async def pump_position():
            async for msg in drone.telemetry.position():
                latest["position"] = msg

    async def pump_velocity():
            async for msg in drone.telemetry.velocity_ned():
                latest["vel_ned"] = msg

    async def pump_attitude():
            async for msg in drone.telemetry.attitude_euler():
                latest["att_euler"] = msg   

    async def pump_battery():
            async for msg in drone.telemetry.battery():
                latest["battery"] = msg


    tasks = [
            asyncio.create_task(pump_position()),
            asyncio.create_task(pump_velocity()),
            asyncio.create_task(pump_attitude()),
            asyncio.create_task(pump_battery()),
        ]

    period_s = 1.0 / LOG_HZ
    state_period_s = 1.0 / STATE_POLL_HZ

    logging_enabled = False
    f = None
    out_file = None
    last_state_check = 0.0
    
    

    print(f"State file: {STATE_FILE}")
    print()
    print("Telemetry logger ready (waiting for START_LOG)...")
    print()
    try:
        while True:
            loop_start = time.time()

            # Poll state file at STATE_POLL_HZ
            if loop_start - last_state_check >= state_period_s:
                last_state_check = loop_start
                state = load_state()
                should_log = state.get("logging_enabled", False)

                if should_log and not logging_enabled:
                    #Starts Logging
                    out_file = LOG_DIR / f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
                    f = open(out_file, "a", encoding="utf-8")
                    logging_enabled = True
                    print(f"START_LOG: logging to {out_file}")

                elif not should_log and logging_enabled:
                    #Stops logging
                    if f:
                        f.close()
                        f = None
                    logging_enabled = False
                    print(f"STOP_LOG: closed {out_file}")
                    out_file = None

            if logging_enabled and f is not None:
                t_rx_epoch_ms = int(time.time() * 1000)

                pos = latest["position"]
                vel = latest["vel_ned"]
                att = latest["att_euler"]
                bat = latest["battery"]

                speed_fts = None
                if vel is not None:
                    vn = vel.north_m_s
                    ve = vel.east_m_s
                    speed_fts = ((vn * vn + ve * ve) ** 0.5) * MS_TO_FTS

                record = {
                    "speed":           speed_fts,
                    "pitch":           att.pitch_deg if att is not None else None,
                    "yaw":             att.yaw_deg   if att is not None else None,
                    "roll":            att.roll_deg  if att is not None else None,
                    "altitude":        (pos.relative_altitude_m * M_TO_FT) if pos is not None else None,
                    "batteryLife":     (bat.remaining_percent * 100.0) if bat is not None and bat.remaining_percent is not None else None,
                    "lastUpdated":     t_rx_epoch_ms,
                    "currentPosition": [pos.latitude_deg if pos is not None else None,
                                        pos.longitude_deg if pos is not None else None],
                }

                f.write(json.dumps(record) + "\n")
                f.flush()

            dt = time.time() - loop_start
            await asyncio.sleep(max(0.0, period_s - dt))

    except KeyboardInterrupt:
        print("Stopping telemetry logger")
    finally:
        if f:
            f.close()
        for t in tasks:
            t.cancel()


if __name__ == "__main__":
    asyncio.run(main())