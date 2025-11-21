import asyncio
import time
import threading

#from Packet.Telemetry.Telemetry import Telemetry
shared_telemetry = {
    "lat": None,
    "lon": None,
    "heading": None,
    "airspeed": None,
    "battery": None,
    "status": None,
    "timestamp": None
}


#from Logger.Logger import Logger

def log(msg):
    print(f"[LOG] {msg}")


from mavsdk import System

# ---------------- SHARED RESOURCES ----------------
#shared_telemetry = Telemetry()
#logger = Logger()
telemetry_lock = threading.Lock()

UAV = System()

# ---------------- TELEMETRY FUNCTIONS ----------------

async def get_Telem():

    pos_gen = UAV.telemetry.position()
    euler_gen = UAV.telemetry.attitude_euler()  
    fw_gen = UAV.telemetry.fixedwing_metrics()
    batt_gen = UAV.telemetry.battery()
    status_gen = UAV.telemetry.status_text()

    while True:
        pos      = await pos_gen.__anext__()
        euler    = await euler_gen.__anext__()
        fw       = await fw_gen.__anext__()
        battery  = await batt_gen.__anext__()
        status   = await status_gen.__anext__()

        with telemetry_lock:
          shared_telemetry["lat"]       = pos.latitude_deg
          shared_telemetry["lon"]       = pos.longitude_deg
          shared_telemetry["heading"]   = euler.yaw_deg
          shared_telemetry["airspeed"]  = fw.airspeed_m_s
          shared_telemetry["battery"]   = battery.remaining_percentage
          shared_telemetry["status"]    = status.text
          shared_telemetry["timestamp"] = time.time()


        #Debug Display
        print("\n[TELEMETRY]")
        print(f"Lat/Lon: {pos.latitude_deg:.6f}, {pos.longitude_deg:.6f}")
        print(f"Heading (yaw_deg): {euler.yaw_deg:.2f}°")
        print(f"Airspeed:          {fw.airspeed_m_s:.2f} m/s")
        print(f"Battery:           {battery.remaining_percentage*100:.1f}%")
        print(f"Status:            {status.text}")

async def main():
    await UAV.connect(system_address="udp://:14540")

    print("Waiting for FC connection...")
    async for state in UAV.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    #Waits for valid GPS
    print("Waiting for GPS lock...")
    async for health in UAV.telemetry.health():
        if health.is_global_position_ok:
            print("GPS OK")
            break
        await asyncio.sleep(1)

    #Starts getting telemetry 
    asyncio.create_task(get_Telem())

    #Keeps alive
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
