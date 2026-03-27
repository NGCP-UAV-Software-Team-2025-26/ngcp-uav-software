import asyncio 
import logging
import time
import sys
from pathlib import Path
from math import nan
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.state_utils import load_state, update_state, STATE_FILE #For the mission_state.json
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
#System from mavsdk brings the following
#drone.telemetry
#drone.action
#drone.mission
#drone.offboard
#drone.core



# Enable INFO level logging by default so that INFO messages are shown
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S")

log = logging.getLogger("main_controller")

#Parameters
STATE_POLL_HZ = 2.0 #How often to check state file
MIN_ALT_M = 15 #Plane has to be above 15M for autonomy to engage
MIN_CONFIDENCE = 0.4 #Ignore traingulation below this (might not need depending on triangulation script)
MIN_MISSION_INTERVAL = 10 #Min seconds between mission uploads
LOITER_ALT_M = 60 #TArget loiter altitutde 
LOITER_SPEED_MS = 15 #Cruise speed
LOITER_TIME_S = 30 #Min time to circle target
LOITER_RADIUS_M = 50 #Acceptance radius for the loiter waypoint (Big cuz fixed wing)
TELEMETRY_TIMEOUT_S = 5 #How old telemetry can be before it is considered bad

def valid_fix(fix: dict) -> bool:
    """Return True if the fix dict has usable lat/lon and sufficient confidence."""
    if not fix:
        return False
    try:
        lat  = fix.get("lat")
        lon  = fix.get("lon")
        conf = fix.get("confidence", 0.0)
        if lat is None or lon is None:
            return False
        if conf < MIN_CONFIDENCE:
            return False
        return True
    except Exception:
        return False
    
def build_loiter_mission(lat: float, lon: float, alt_m: float = LOITER_ALT_M) -> MissionPlan:
    """
    MissionItem is the only MAVSDK mechanism that exposes
    loiter_time_s and acceptance_radius_m for fixed-wing behavior.

    is_fly_through=False forces the autopilot to circle until loiter_time_s
    elapses before advancing. acceptance_radius_m is large so that the
    fixed-wing aircraft's turning radius does not prevent waypoint completion.
    """
    loiter_item = MissionItem(
        latitude_deg             = lat,
        longitude_deg            = lon,
        relative_altitude_m      = alt_m,
        speed_m_s                = LOITER_SPEED_MS,
        is_fly_through           = False,
        loiter_time_s            = LOITER_TIME_S,
        acceptance_radius_m      = LOITER_RADIUS_M,
        yaw_deg                  = nan,
        vehicle_action           = MissionItem.VehicleAction.NONE,
    )
    return MissionPlan([loiter_item])


async def run():
    #Connection
    drone = System()
    await drone.connect(system_address="udp://:14606")
  
    log.info("Waiting for autopilot connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            log.info("Plane connected.")
            break
    
    log.info("Waiting for plane to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            log.info("Global position estimate OK")
            break
    
    autonomy_active = False
    last_upload_time = 0.0
    last_processed_fix_id = None
    last_telemetry_time = time.time()
    state_period_s = 1.0 / STATE_POLL_HZ
    last_state_check = 0.0

    #We need telemetry because minimum height is needed for autonomy and loiter height is set
    telemetry = {
        "rel_alt_m": 0.0,
        "flight_mode": None,
        "armed": False,
    }

    async def watch_telemetry():
        nonlocal last_telemetry_time
        async for pos in drone.telemetry.position():
            telemetry["rel_alt_m"] = pos.relative_altitude_m
            last_telemetry_time = time.time()

    async def watch_flight_mode():
        async for mode in drone.telemetry.flight_mode():
            telemetry["flight_mode"] = str(mode)

    async def watch_armed():
        async for is_armed in drone.telemetry.armed():
            telemetry["armed"] = is_armed

    asyncio.ensure_future(watch_telemetry())
    asyncio.ensure_future(watch_flight_mode())
    asyncio.ensure_future(watch_armed())

    log.info("Background telemetry listeners started. Entering main control loop.")


    #MAIN CONTROL LOOP

    while True:
        loop_start = time.time()

        if loop_start - last_state_check < state_period_s:
            await asyncio.sleep(0.05)
            continue
            
        last_state_check = loop_start

        state = load_state()
        should_autonomy = state.get("autonomy_enabled", False)
        target_fix = state.get("target_fix", {})
        pending_action = state.get("pending_action", None)
        mission_status = state.get("mission_status", {})
        rtl_requested = state.get("rtl_requested", False)

        #RTL !
        if rtl_requested:
            log.info("RTL requested. Commanding return to launch.")
            try:
                await drone.action.return_to_launch()
                update_state("rtl_requested", False)
                autonomy_active = False
                update_state("mission_status", {
                    **mission_status,
                    "current_mode": "RTL",
                })
                log.info("RTL command accepted. Autonomy paused.")
            except Exception as exc:
                log.error("RTL command failed: %s", exc)
            await asyncio.sleep(state_period_s)
            continue

        #Telemetry for safety 
        telemetry_age = time.time() - last_telemetry_time
        if telemetry_age > TELEMETRY_TIMEOUT_S:
            log.warning(
                "Telemetry stale (%.1f s). Pausing autonomy until link recovers.",
                telemetry_age,
            )
            if autonomy_active:
                autonomy_active = False
                update_state("mission_status", {**mission_status, "current_mode": "Paused_TelemetryLoss"})
            await asyncio.sleep(state_period_s)
            continue


        #Manual Override
        flight_mode = telemetry.get("flight_mode", "")
        pilot_in_control = flight_mode not in (
            "FlightMode.MISSION",
            "FlightMode.AUTO",
            "MISSION",
            "AUTO",
        ) and flight_mode is not None

        if pilot_in_control and autonomy_active:
            log.info("Pilot Override detected. Pausing Autonomy", flight_mode)
            autonomy_active = False
            update_state("mission_status",{**mission_status, "current_mode": "Paused_PilotOverride"})
            await asyncio.sleep(state_period_s)
            continue

        current_mode = mission_status.get("current_mode", "")
        if (
            not pilot_in_control
            and current_mode == "Paused_PilotOverride"
            and should_autonomy
        ):
            log.info("Autopilot mode restored. Resuming autonomy.")
            update_state("mission_status", {**mission_status, "current_mode": "Idle"})



        if should_autonomy and not autonomy_enabled:
            # Start autonomy mission
            log.info("Autonomy enabled")
            autonomy_enabled = True
            update_state("mission_status", {"current_mode": "Navigating"})
            

        elif not should_autonomy and autonomy_enabled:
            # Stopping Autonomy
            log.info("Autonomy disabled")
            autonomy_enabled = False
            update_state("mission_status", {"current_mode": "Idle"})
            await asyncio.sleep(state_period_s)
            continue
        
        if not autonomy_active:
            await asyncio.sleep(state_period_s)
            continue


        #Minimum height safety check

        rel_alt = telemetry.get("rel_alt_m", 0.0)
        armed   = telemetry.get("armed", False)

        if not armed:
            log.debug("Aircraft not armed. Waiting.")
            await asyncio.sleep(state_period_s)
            continue

        if rel_alt < MIN_ALT_M:
            log.debug(
                "Altitude %.1f m is below minimum %.1f m. Waiting for climb.",
                rel_alt, MIN_ALT_M,
            )
            await asyncio.sleep(state_period_s)
            continue

        
        await asyncio.sleep(state_period_s)


        #Checking for triangulation fix

        fix_id = target_fix.get("fix_id")

        #No fix yet
        if not valid_fix(target_fix):
            conf = target_fix.get("confidence")

            if conf is not None and conf < MIN_CONFIDENCE:
                log.info(
                    "Confidence %.2f below threshold %.2f Waiting...",
                    conf, MIN_CONFIDENCE,
                )
            else:
                log.debug("No valid fix yet")
            update_state("mission_status", {**mission_status, "current_mode": "AwaitingFix"})
            await asyncio.sleep(state_period_s)
            continue

        #If its the same fix
        if fix_id == last_processed_fix_id:
            log.debug("Fix id %s already processed. Continuing to loiter.", fix_id)
            await asyncio.sleep(state_period_s)
            continue

        #TO make sure kmissions aren't uploaded too fast
        time_since_upload = time.time() - last_upload_time
        if time_since_upload < MIN_MISSION_INTERVAL:
            log.debug(
                "Rate limit: %.1f s since last upload (min %.1f s). Holding.",
                time_since_upload, MIN_MISSION_INTERVAL,
            )
            await asyncio.sleep(state_period_s)
            continue


        #Uploading a new missio 

        fix_lat  = target_fix["lat"]
        fix_lon  = target_fix["lon"]
        fix_conf = target_fix.get("confidence", 0.0)

        log.info(
            "New fix (id=%s): lat=%.6f lon=%.6f confidence=%.2f. Uploading mission.",
            fix_id, fix_lat, fix_lon, fix_conf,
        )

        mission_plan = build_loiter_mission(fix_lat, fix_lon)

        try:
            await drone.mission.upload_mission(mission_plan)
            log.info("Mission uploaded. Setting start item to 0.")
            await drone.mission.set_current_mission_item(0)
            log.info("Starting mission.")
            await drone.mission.start_mission()
        except Exception as exc:
            log.error("Mission upload or start failed: %s", exc)
            await asyncio.sleep(state_period_s)
            continue

        #Update that fix is processed
        last_upload_time      = time.time()
        last_processed_fix_id = fix_id

        update_state("mission_status", {
            "current_mode":          "Navigating",
            "active_target_fix_id":  fix_id,
            "last_processed_fix_id": last_processed_fix_id,
            "mission_count":         mission_status.get("mission_count", 0) + 1,
        })


        #Monitoring the mission progress
        async def _watch_mission_progress(expected_fix_id):
            async for progress in drone.mission.mission_progress():
                log.info(
                    "Mission progress: %d / %d  (fix_id=%s)",
                    progress.current, progress.total, expected_fix_id,
                )
                if progress.current >= progress.total:
                    log.info(
                        "Mission complete (fix_id=%s). Aircraft now loitering.",
                        expected_fix_id,
                    )
                    current = load_state().get("mission_status", {})
                    #Only update if its still on the same fix
                    if current.get("active_target_fix_id") == expected_fix_id:
                        update_state("mission_status", {
                            **current,
                            "current_mode": "Loitering",
                        })
                    return

        asyncio.ensure_future(_watch_mission_progress(fix_id))

        await asyncio.sleep(state_period_s)



   

if __name__ == "__main__":
  asyncio.run(run())    