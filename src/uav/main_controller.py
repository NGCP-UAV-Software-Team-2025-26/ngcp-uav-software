import asyncio 
import logging
from turtle import pos
from mavsdk import system
#system brings the following

#drone.telemetry
#drone.action
#drone.mission
#drone.offboard
#drone.core
from mavsdk.mission import MissionItem, MissionPlan


# Enable INFO level logging by default so that INFO messages are shown
logging.basicConfig(level=logging.INFO)

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
  
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Plane Connected")
            break
    
    print("Waiting for plane to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
    
    position = await drone.telemetry.position().__aiter__().__anext__()
    #this gives lat, lon, absolute_altitude_m, relative_altitude_m
    lat = pos.latitude_deg
    lon = pos.longitude_deg
    rel_alt = 60.0

    print(f"Home-ish position: {lat}, {lon}")

    loiter_seconds = 20.0

    loiter_item = MissionItem(
        latitude_deg=lat,
        longitude_deg=lon,
        relative_altitude_m=rel_alt,
        speed_m_s=15.0,
        #speed is in meters/second
        is_fly_through=False,
        #Whether it is a fly through waypoint or a loiter waypoint
        gimbal_pitch_deg=float('nan'),
        gimbal_yaw_deg=float('nan'),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=loiter_seconds,#The loiter time 
        camera_photo_interval_s=float('nan'),
        acceptance_radius_m=50.0,#Big raidus needed for a fixed wing
        yaw_deg=float('nan'),
        camera_photo_distance_m=float('nan')
    )

    mission_plan = MissionPlan([loiter_item])

    print("Uploading mission...")
    await drone.mission.upload_mission(mission_plan)
   


    print ("Arming..")
    await drone.action.arm()
    
    print("Taking off..")
    await drone.action.takeoff()
    
  
    
    await asyncio.sleep (10)

    print("Starting mission (loiter)...")
    await drone.mission.start_mission()

    async for progress in drone.mission.mission_progress():
        print(f"Mission progress: {progress.current}/{progress.total}")
        if progress.current == progress.total:
            print("Mission complete (loiter finished).")
            break

    print("Returning to launch...")
    await drone.action.return_to_launch()
    
if __name__ == "__main__":
  asyncio.run(run())