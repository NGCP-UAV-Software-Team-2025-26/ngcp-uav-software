import asyncio
import sys
import time
import threading
import math
import subprocess

from Communication.XBee.XBee import XBee
from Communication.XBee.Frames.x81 import x81
from Packet.Telemetry.Telemetry import Telemetry
from Logger.Logger import Logger
from Packet.Command.EmergencyStop import EmergencyStop
from Packet.Command.CommandResponse import CommandResponse

from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw
from mavsdk.mission_raw import MissionItem

# ---------------- CONSTANTS ----------------

VEHICLE_NAME = "UAV"
GCS_MAC = "0013A200424366C7"
PORT = "/dev/ttyUSB0"

# ---------------- SHARED RESOURCES ----------------

shared_telemetry = Telemetry()
logger = Logger()
telemetry_lock = threading.Lock()

UAV_xbee = XBee(port=PORT, baudrate=115200, logger=logger)
UAV_xbee.open()

UAV = System()

# ---------------- TELEMETRY FUNCTIONS ----------------

async def get_Telem():
    global UAV
    global shared_telemetry
    async for (position, battery, status_text, AngVelBody, FixedWingMetrics) in combine_Telem(UAV):
        shared_telemetry.speed = FixedWingMetrics.airspeed_m_s
        shared_telemetry.pitch = AngVelBody.pitch_rad_s
        shared_telemetry.yaw = AngVelBody.yaw_rad_s
        shared_telemetry.roll = AngVelBody.roll_rad_s
        shared_telemetry.altitude = position.relative_altitude_m
        shared_telemetry.battery_life = battery.remaining_percentage
        shared_telemetry.current_latitude = position.latitude_deg
        shared_telemetry.current_longitude = position.longitude_deg
        shared_telemetry.message_lat = position.latitude_deg
        shared_telemetry.message_long = position.longitude_deg
        shared_telemetry.message_flag = 0
        shared_telemetry.vehicle_status = status_text.text
        shared_telemetry.last_updated = time.time()
	
        # ✅ Debug Print for Vehicle Telemetry
        print("\n[TELEMETRY RECEIVED]")
        print(f"Longitude: {position.longitude_deg:.6f}")
        print(f"Altitude (relative): {position.relative_altitude_m:.2f} m")
        print(f"Altitude (absolute): {position.absolute_altitude_m:.2f} m")
        print(f"Airspeed:  {FixedWingMetrics.airspeed_m_s:.2f} m/s")
        print(f"Attitude (Pitch/Yaw/Roll): {AngVelBody.pitch_rad_s:.2f}, {AngVelBody.yaw_rad_s:.2f}, {AngVelBody.roll_rad_s:.2f}")
        print(f"Battery:   {battery.remaining_percentage*100:.1f}%")
        print(f"Status:    {status_text.text}")

async def combine_Telem(UAV):
    pos_gen = UAV.telemetry.position()
    batt_gen = UAV.telemetry.battery()
    status_gen = UAV.telemetry.status_text()
    ang_vel_gen = UAV.telemetry.attitude_angular_velocity_body()
    fixedwing_gen = UAV.telemetry.fixedwing_metrics()

    while True:
        position = await pos_gen.__anext__()
        battery = await batt_gen.__anext__()
        status_text = await status_gen.__anext__()
        AngVelBody = await ang_vel_gen.__anext__()
        FixedWingMetrics = await fixedwing_gen.__anext__()
        yield (position, battery, status_text, AngVelBody, FixedWingMetrics)

def send_telemetry():
	global shared_telemetry
    logger.write("Starting to send telemetry...")
    while True:
        with telemetry_lock:
            telemetry_data = shared_telemetry.encode()
        UAV_xbee.transmit_data(telemetry_data, address=GCS_MAC)
        time.sleep(3)

def update_telemetry(): # REDUNDANT FUNCTION, NOT NEEDED 
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    while True:
    	with telemetry_lock:
    		loop.run_until_complete(get_Telem())
		time.sleep(1)

def listen_for_commands():
    while True:
        time.sleep(1)

# ---------------- MISSION FUNCTIONS ----------------

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def get_transmitter_coordinates():
    try:
        result = subprocess.run(["./read"], capture_output=True, text=True, check=True)
        print("Raw output from ./read:")
        print(result.stdout)
        lines = result.stdout.strip().splitlines()

        for line in lines:
            if ',' in line:
                lat_str, lon_str = line.strip().split(",")
                return float(lat_str), float(lon_str)

        print(f"No valid coordinates found in output: {result.stdout}")
        return None, None

    except Exception as e:
        print(f"Error running transmitter locator: {e}")
        return None, None

async def complex_altitude_profile_and_hold(drone, target_lat, target_lon, descent_agl_m, hold_agl_m, yaw_deg):
    pos = await drone.telemetry.position().__anext__()
    att = await drone.telemetry.attitude_euler().__anext__()

    ground_amsl = pos.absolute_altitude_m - pos.relative_altitude_m
    descent_amsl = ground_amsl + descent_agl_m
    hold_amsl = ground_amsl + hold_agl_m

    current_lat = pos.latitude_deg
    current_lon = pos.longitude_deg
    current_yaw = att.yaw_deg

    for _ in range(10):
        await drone.offboard.set_position_global(PositionGlobalYaw(
            current_lat, current_lon, pos.absolute_altitude_m, current_yaw,
            altitude_type=PositionGlobalYaw.AltitudeType.AMSL))
        await asyncio.sleep(0.1)

    await drone.offboard.start()

    while True:
        await drone.offboard.set_position_global(PositionGlobalYaw(
            current_lat, current_lon, descent_amsl, current_yaw,
            altitude_type=PositionGlobalYaw.AltitudeType.AMSL))
        pos = await drone.telemetry.position().__anext__()
        if abs(pos.absolute_altitude_m - descent_amsl) < 2.5:
            break
        await asyncio.sleep(0.5)

    while True:
        await drone.offboard.set_position_global(PositionGlobalYaw(
            target_lat, target_lon, descent_amsl, yaw_deg,
            altitude_type=PositionGlobalYaw.AltitudeType.AMSL))
        pos = await drone.telemetry.position().__anext__()
        distance = haversine_distance(pos.latitude_deg, pos.longitude_deg, target_lat, target_lon)
        if distance < 50:
            break
        await asyncio.sleep(0.5)

    while True:
        await drone.offboard.set_position_global(PositionGlobalYaw(
            target_lat, target_lon, hold_amsl, yaw_deg,
            altitude_type=PositionGlobalYaw.AltitudeType.AMSL))
        pos = await drone.telemetry.position().__anext__()
        if abs(pos.absolute_altitude_m - hold_amsl) < 2.5:
            break
        await asyncio.sleep(0.5)

    await drone.offboard.stop()
    await asyncio.sleep(1)
    await drone.action.hold()

# ---------------- MAIN ----------------

async def main():
    await UAV.connect(system_address="udp://:14540")

    async for state in UAV.core.connection_state():
        if state.is_connected:
            break

    async for health in UAV.telemetry.health():
        if health.is_global_position_ok:
            break
        await asyncio.sleep(1)

    telemetry_thread = threading.Thread(target=send_telemetry, daemon=True)
    command_thread = threading.Thread(target=listen_for_commands, daemon=True)

    asyncio.create_task(get_Telem())
    telemetry_thread.start()
    command_thread.start()

    try:
        while True:
        	await asyncio.sleep(1)
        	asyncio.create_task(get_Telem())  
    except KeyboardInterrupt:
        print("\nShutting down...")
        UAV_xbee.close()

if __name__ == "__main__":
    asyncio.run(main())
