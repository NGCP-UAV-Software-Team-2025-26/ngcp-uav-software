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

OUT_FILE = LOG_DIR / f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"

LOG_HZ = 5.0  #logging rate (Hz)
M_TO_FT = 3.280839895
MS_TO_FTS = 3.280839895

async def wait_connected(drone: System):
    print(f"Connecting via {SYSTEM_ADDRESS} ...")
    await drone.connect(system_address=SYSTEM_ADDRESS)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4")
            return
        
async def wait_position_ok(drone: System):
    print("Waiting for global position + home position ...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK + Home OK")
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

    print(f"Logging to {OUT_FILE}")
    period_s = 1.0 / LOG_HZ

    try:
            with open(OUT_FILE, "a", encoding="utf-8") as f:
                while True:
                    loop_start = time.time()

                    #Pi receipt timestamps (For timing with Kraken Logs)
                    t_rx_epoch_ms = int(time.time() * 1000)
                    t_rx_mono_ns = time.monotonic_ns()

                    pos = latest["position"]
                    vel = latest["vel_ned"]
                    att = latest["att_euler"]
                    bat = latest["battery"]

                    
                    
                    speed_fts = None
                    if vel is not None:
                        vn = vel.north_m_s
                        ve = vel.east_m_s
                        speed_ms = (vn * vn + ve * ve) ** 0.5
                        speed_fts = speed_ms * MS_TO_FTS

                    #pitch/yaw/roll (deg)
                    pitch = att.pitch_deg if att is not None else None
                    yaw = att.yaw_deg if att is not None else None
                    roll = att.roll_deg if att is not None else None

                    #Altitude (ft): relative altitude
                    altitude_ft = (pos.relative_altitude_m * M_TO_FT) if pos is not None else None

                    #Battery (0-100)
                    batteryLife = None
                    if bat is not None and bat.remaining_percent is not None:
                        batteryLife = bat.remaining_percent * 100.0

                    #CurrentPosition(lat/lon)
                    lat = pos.latitude_deg if pos is not None else None
                    lon = pos.longitude_deg if pos is not None else None

                
                    record = {
                        "speed": speed_fts,
                        "pitch": pitch,
                        "yaw": yaw,
                        "roll": roll,
                        "altitude": altitude_ft,
                        "batteryLife": batteryLife,
                        "lastUpdated": t_rx_epoch_ms,
                        "currentPosition": [lat, lon],
                    }

                    f.write(json.dumps(record) + "\n")
                    f.flush()

                    #Fixed logging rate
                    dt = time.time() - loop_start
                    await asyncio.sleep(max(0.0, period_s - dt))

    except KeyboardInterrupt:
        print("Stopping telemetry logger")
    finally:
        for t in tasks:
            t.cancel()


if __name__ == "__main__":
    asyncio.run(main())