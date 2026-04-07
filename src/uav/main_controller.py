import asyncio 
import logging
import time
import sys
from pathlib import Path
# from math import nan
sys.path.append(str(Path(__file__).resolve().parents[1]))

from autonomy.search_logic import (
    start_search_phase,
    reset_search_phase,
    update_elapsed,
    check_time_gates,
)
from state.state_utils import load_state, update_state, STATE_FILE #For the mission_state.json
from mavsdk import System
# This is now commented out due to MissionItem and Mission Plan only working in PX4
# from mavsdk.mission import MissionItem, MissionPlan

#System from mavsdk brings the following
#drone.telemetry
#drone.action
#drone.mission
#drone.offboard
#drone.core

#It's replaced with
from pymavlink import mavutil


# Enable INFO level logging by default so that INFO messages are shown
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S")

log = logging.getLogger("main_controller")

#Parameters
STATE_POLL_HZ = 2.0 #How often to check state file
MIN_ALT_M = 15 #Plane has to be above 15M for autonomy to engage
# MIN_CONFIDENCE = 0.4 #Ignore traingulation below this (might not need depending on triangulation script)
# MIN_MISSION_INTERVAL = 10 #Min seconds between mission uploads
LOITER_ALT_M = 60 #TArget loiter altitutde 
# LOITER_SPEED_MS = 15 #Cruise speed
# LOITER_TIME_S = 30 #Min time to circle target
LOITER_RADIUS_M = 50 #Acceptance radius for the loiter waypoint (Big cuz fixed wing)
TELEMETRY_TIMEOUT_S = 5 #How old telemetry can be before it is considered bad

#NEW From ARDUPILOT
ARDUPLANE_MODES = {
    "MANUAL": 0, 
    "CIRCLE": 1, 
    "STABILIZE": 2, 
    "TRAINING": 3,
    "ACRO": 4,  
    "FBWA": 5,   
    "FBWB": 6,     
    "CRUISE": 7,
    "AUTO": 10, 
    "RTL": 11,   
    "LOITER": 12,  
    "GUIDED": 15,
}


#PYMAVLINK Functions

#HEartbeat
def mav_connect(connection_string: str):
    # Opens a pymavlink connection
    mav = mavutil.mavlink_connection(connection_string)
    log.info("pymavlink: waiting for heartbeat on %s …", connection_string)
    mav.wait_heartbeat()
    log.info(
        "pymavlink: heartbeat received  system=%d  component=%d",
        mav.target_system, mav.target_component,
    )
    return mav

#Mode
def mav_set_mode(mav, mode_name: str) -> bool:
    mode_id = ARDUPLANE_MODES.get(mode_name.upper())
    if mode_id is None:
        raise ValueError(f"Unknown ArduPlane mode: {mode_name}")
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    # Wait for heartbeat to echo the new mode (up to 5 tries)
    for _ in range(5):
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            log.info("pymavlink: mode set to %s (%d)", mode_name, mode_id)
            return True
    log.warning("pymavlink: mode ACK not confirmed for %s", mode_name)
    return False


def mav_upload_loiter_mission(
    mav,
    lat: float,
    lon: float,
    alt_m: float,
    loiter_radius_m: float = LOITER_RADIUS_M,
) -> None:
    #   0 — Home placeholder  (ArduPilot always requires item 0 = home)
    #   1 — NAV_WAYPOINT      fly to target
    #   2 — NAV_LOITER_UNLIM  circle indefinitely
    #Raises RuntimeError if the vehicle rejects the upload.

    lat_i = int(lat * 1e7)
    lon_i = int(lon * 1e7)

    items = [
        # Item 0: home / origin placeholder
        dict(
            command      = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            frame        = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current      = 0, autocontinue = 1,
            param1=0, param2=0, param3=0, param4=0,
            x=lat_i, y=lon_i, z=alt_m,
        ),
        # Item 1: fly to target waypoint
        dict(
            command      = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            frame        = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current      = 1,   # ← execution starts here
            autocontinue = 1,
            param1 = 0,          # hold time (s) — unused for fixed-wing
            param2 = 30.0,       # acceptance radius (m)
            param3 = 0,          # pass-through (0 = stop at WP)
            param4 = float("nan"),
            x=lat_i, y=lon_i, z=alt_m,
        ),
        # Item 2: loiter indefinitely
        dict(
            command      = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            frame        = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current      = 0, autocontinue = 0,
            param1 = 0,
            param2 = 0,
            param3 = loiter_radius_m,   # positive = clockwise
            param4 = float("nan"),
            x=lat_i, y=lon_i, z=alt_m,
        ),
    ]

    mav.mav.mission_count_send(
            mav.target_system,
            mav.target_component,
            len(items),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

    ack = None
    while ack is None:
        msg = mav.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
            blocking=True,
            timeout=5,
        )
        if msg is None:
            raise TimeoutError("No MISSION_REQUEST received — vehicle not responding")

        t = msg.get_type()
        if t in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq  = msg.seq
            item = items[seq]
            use_int = (t == "MISSION_REQUEST_INT")
            log.debug("Sending mission item %d (INT=%s)", seq, use_int)

            if use_int:
                mav.mav.mission_item_int_send(
                    mav.target_system, mav.target_component, seq,
                    item["frame"], item["command"],
                    item["current"], item["autocontinue"],
                    item["param1"], item["param2"],
                    item["param3"], item["param4"],
                    item["x"], item["y"], item["z"],
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            else:
                mav.mav.mission_item_send(
                    mav.target_system, mav.target_component, seq,
                    item["frame"], item["command"],
                    item["current"], item["autocontinue"],
                    item["param1"], item["param2"],
                    item["param3"], item["param4"],
                    item["x"] / 1e7, item["y"] / 1e7, item["z"],
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )

        elif t == "MISSION_ACK":
            ack = msg

    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(
            f"Mission rejected by vehicle: MAV_MISSION result={ack.type}"
        )
    log.info("Mission accepted by ArduPilot (%d items)", len(items))


def mav_start_mission(mav) -> None:
    """Switch to AUTO so ArduPilot executes the uploaded mission."""
    mav_set_mode(mav, "AUTO")
    log.info("pymavlink: AUTO mode commanded — mission executing")

def mav_loiter_in_place(mav) -> None:
    mav_set_mode(mav, "LOITER")
    log.info("pymavlink: LOITER mode commanded")

#Old build loiter mission 
# def build_loiter_mission(lat: float, lon: float, alt_m: float = LOITER_ALT_M) -> MissionPlan:
#     """
#     MissionItem is the only MAVSDK mechanism that exposes
#     loiter_time_s and acceptance_radius_m for fixed-wing behavior.

#     is_fly_through=False forces the autopilot to circle until loiter_time_s
#     elapses before advancing. acceptance_radius_m is large so that the
#     fixed-wing aircraft's turning radius does not prevent waypoint completion.
#     """
#     loiter_item = MissionItem(
#         latitude_deg    = lat,
#         longitude_deg   = lon,
#         relative_altitude_m = alt_m,
#         speed_m_s   = LOITER_SPEED_MS,
#         is_fly_through  = False,
#         gimbal_pitch_deg    =0.0,
#         gimbal_yaw_deg  =0.0,
#         camera_action   =  MissionItem.CameraAction.NONE,
#         loiter_time_s   = 0,
#         camera_photo_interval_s=0.0,
#         acceptance_radius_m = LOITER_RADIUS_M,
#         yaw_deg = nan,
#         camera_photo_distance_m=0.0,
#         vehicle_action = MissionItem.VehicleAction.NONE,
#     )
#     return MissionPlan([loiter_item])

#Search phase helpers


async def upload_and_start(mav, fix: dict, mission_status: dict) -> bool:
    """Upload mission to fix coords and switch to AUTO. Returns success."""
    try:
        await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: mav_upload_loiter_mission(
                mav, fix["lat"], fix["lon"], LOITER_ALT_M
            ),
        )
        await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: mav_start_mission(mav),
        )
        return True
    except Exception as exc:
        log.error("Mission upload/start failed: %s", exc)
        return False

async def run():
    #Connection
    drone = System()
    
    
    #For Real
    # await drone.connect(system_address="udp://:14606")

    #For Sitl
    await drone.connect(system_address="udpin://0.0.0.0:14601")
  
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
    

    #Heartbeat for pymavlink (Will need to change the UDP port when incorporated everything else)
    mav = mav_connect("udpin:0.0.0.0:14603")
    

    autonomy_active = False
    update_state("autonomy_active", autonomy_active)
    # last_upload_time = 0.0
    # last_processed_fix_id = None
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

        #New 
        pending_action = state.get("pending_action", None)

        #For new search session
        if pending_action == "new_search_session":
            state = reset_search_phase(state)
            update_state("search_phase", state["search_phase"])
            update_state("decision", state["decision"])
            update_state("pending_action", None)
            log.info("New search session started from GCS command.")
            await asyncio.sleep(state_period_s)
            continue

            
        state = update_elapsed(state)
        update_state("search_phase", state.get("search_phase", {}))

        best_fix = state.get("best_fix", {})

        mission_status = state.get("mission_status", {})
        decision = state.get("decision", {})

        state, event = check_time_gates(state)
        if event:
            update_state("decision", state["decision"])

        should_autonomy = state.get("autonomy_active", False)
        
        #END of NEW
        
        rtl_requested = state.get("rtl_requested", False)

        log.info("should_autonomy=%s", should_autonomy)
        log.info("autonomy_active=%s", autonomy_active)
        mode_str = telemetry.get("flight_mode") or "UNKNOWN"

        log.info("armed=%s alt=%.1f mode=%s",
                telemetry.get("armed"),
                telemetry.get("rel_alt_m", -1),
                mode_str)
        
        

        #RTL !
        if rtl_requested:
            log.info("RTL requested. Commanding return to launch.")
            try:
                await drone.action.return_to_launch()
                update_state("rtl_requested", False)
                autonomy_active = False
                update_state("autonomy_active", autonomy_active)
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
                update_state("autonomy_active", autonomy_active)
                update_state("mission_status", {**mission_status, "current_mode": "Paused_TelemetryLoss"})
            await asyncio.sleep(state_period_s)
            continue


        #Manual Override
        flight_mode = telemetry.get("flight_mode", "")
        manual_override_modes = {
            "MANUAL",
            "STABILIZE",
            "FBWA",
            "FBWB",
            "CRUISE",
            "TRAINING",
            "ACRO",
            "FlightMode.MANUAL",
            "FlightMode.STABILIZE",
            "FlightMode.FBWA",
            "FlightMode.FBWB",
            "FlightMode.CRUISE",
            "FlightMode.TRAINING",
            "FlightMode.ACRO",
        }
        pilot_in_control = flight_mode in manual_override_modes

        if pilot_in_control:
            if autonomy_active or mission_status.get("current_mode") != "Paused_PilotOverride":
                log.info("Pilot Override detected. Pausing Autonomy (mode=%s)", flight_mode)

                autonomy_active = False
                update_state("autonomy_active", False)
                update_state("mission_status", {
                    **mission_status,
                    "current_mode": "Paused_PilotOverride"
                })

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

        if flight_mode in ("RETURN_TO_LAUNCH", "FlightMode.RETURN_TO_LAUNCH", "RTL"):
            log.info("Aircraft is in RTL. Not re-enabling autonomy.")
            autonomy_active = False
            update_state("autonomy_active", False)
            update_state("mission_status", {
                **mission_status,
                "current_mode": "RTL",
            })
            await asyncio.sleep(state_period_s)
            continue

        if should_autonomy and not autonomy_active:

            if flight_mode in ("RETURN_TO_LAUNCH", "FlightMode.RETURN_TO_LAUNCH", "RTL"):
                # NEVER re-enable during RTL
                await asyncio.sleep(state_period_s)
                continue

            if pilot_in_control:
                # don't re-enable if pilot still flying
                await asyncio.sleep(state_period_s)
                continue
            log.info("Autonomy enabled")
            autonomy_active = True
            update_state("autonomy_active", autonomy_active)
            state = start_search_phase(state)
            update_state("search_phase", state["search_phase"])
            update_state("decision", state["decision"])

            if mission_status.get("current_mode") != "Searching":
                update_state("mission_status", {
                    **mission_status,
                    "current_mode": "Searching",
                })

        elif not should_autonomy and autonomy_active:
            # Stopping Autonomy
            log.info("Autonomy disabled")
            autonomy_active = False
            update_state("autonomy_active", autonomy_active)
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: mav_loiter_in_place(mav),
            )

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


        # #TO make sure kmissions aren't uploaded too fast
        # time_since_upload = time.time() - last_upload_time
        # if time_since_upload < MIN_MISSION_INTERVAL:
        #     log.debug(
        #         "Rate limit: %.1f s since last upload (min %.1f s). Holding.",
        #         time_since_upload, MIN_MISSION_INTERVAL,
        #     )
        #     await asyncio.sleep(state_period_s)
        #     continue
        
        #Fallbacks if search window ends

        if event == "SEARCH_TIMEOUT":
            log.warning("Search window expired. Commanding RTL.")
            try:
                await drone.action.return_to_launch()
                autonomy_active = False
                update_state("autonomy_active", autonomy_active)
                update_state("mission_status", {
                    **mission_status,
                    "current_mode": "RTL_SearchTimeout",
                })
            except Exception as exc:
                log.error("RTL on timeout failed: %s", exc)
            await asyncio.sleep(state_period_s)
            continue

        elif event == "FALLBACK_FIX":
            log.warning(
                "Flying to fallback fix  fix_id=%s lat=%.6f lon=%.6f conf=%.2f",
                best_fix.get("fix_id"),
                best_fix.get("lat"),
                best_fix.get("lon"),
                best_fix.get("confidence"),
            )

            success = await upload_and_start(mav, best_fix, mission_status)
            if success:
                update_state("mission_status", {
                    **mission_status,
                    "current_mode": "Navigating_Fallback",
                    "active_mission_fix_id": best_fix.get("fix_id"),
                })
            await asyncio.sleep(state_period_s)
            continue


        # # mission_plan = build_loiter_mission(fix_lat, fix_lon)

        # # try:
        # #     await drone.mission.upload_mission(mission_plan)
        # #     log.info("Mission uploaded. Setting start item to 0.")
        # #     await drone.mission.set_current_mission_item(0)
        # #     log.info("Starting mission.")
        # #     await drone.mission.start_mission()
        # # except Exception as exc:
        # #     log.error("Mission upload or start failed: %s", exc)
        # #     await asyncio.sleep(state_period_s)
        # #     continue

        # # #Update that fix is processed
        # # last_upload_time      = time.time()
        # # last_processed_fix_id = fix_id

        # # update_state("mission_status", {
        # #     "current_mode":          "Navigating",
        # #     "active_target_fix_id":  fix_id,
        # #     "last_processed_fix_id": last_processed_fix_id,
        # #     "mission_count":         mission_status.get("mission_count", 0) + 1,
        # # })

        # try:
        #     # Runs in a thread so it doesn't block the asyncio event loop
        #     # while waiting for MISSION_REQUEST / MISSION_ACK messages.
        #     await asyncio.get_event_loop().run_in_executor(
        #         None,
        #         lambda: mav_upload_loiter_mission(mav, fix_lat, fix_lon, LOITER_ALT_M),
        #     )
        #     await asyncio.get_event_loop().run_in_executor(
        #         None,
        #         lambda: mav_start_mission(mav),
        #     )
        # except Exception as exc:
        #     log.error("Mission upload/start failed: %s", exc)
        #     await asyncio.sleep(state_period_s)
        #     continue

        # last_upload_time      = time.time()
        # last_processed_fix_id = fix_id

        # update_state("mission_status", {
        #     "current_mode":          "Navigating",
        #     "active_target_fix_id":  fix_id,
        #     "last_processed_fix_id": last_processed_fix_id,
        #     "mission_count":         mission_status.get("mission_count", 0) + 1,
        # })

        # #Monitoring the mission progress
        # async def watch_mission_progress(expected_fix_id):
        #     async for progress in drone.mission.mission_progress():
        #         log.info(
        #             "Mission progress: %d / %d  (fix_id=%s)",
        #             progress.current, progress.total, expected_fix_id,
        #         )
        #         if progress.current >= progress.total:
        #             log.info(
        #                 "Mission complete (fix_id=%s). Aircraft now loitering.",
        #                 expected_fix_id,
        #             )
        #             current = load_state().get("mission_status", {})
        #             #Only update if its still on the same fix
        #             if current.get("active_target_fix_id") == expected_fix_id:
        #                 update_state("mission_status", {
        #                     **current,
        #                     "current_mode": "Loitering",
        #                 })
        #             return

        # asyncio.ensure_future(watch_mission_progress(fix_id))

        
        await asyncio.sleep(state_period_s)
        continue

        



   

if __name__ == "__main__":
  asyncio.run(run())    