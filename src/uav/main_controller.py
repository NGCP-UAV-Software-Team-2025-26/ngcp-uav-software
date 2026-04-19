import asyncio 
import logging
import time
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.state_utils import load_state, update_state #For the mission_state.json
from state.nav_state_utils import load_nav_state, update_nav_state #For waypoints and what not
from mavsdk import System

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
LOITER_ALT_M = 60 #TArget loiter altitutde 
LOITER_RADIUS_M = 50 #Acceptance radius for the loiter waypoint (Big cuz fixed wing)
TELEMETRY_TIMEOUT_S = 5 #How old telemetry can be before it is considered bad

#From ARDUPILOT
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

#Heartbeat
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

def mav_upload_mission_items(mav, items: list) -> None:
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
            seq = msg.seq
            item = items[seq]
            use_int = (t == "MISSION_REQUEST_INT")

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
        raise RuntimeError(f"Mission rejected by vehicle: MAV_MISSION result={ack.type}")

    log.info("Mission accepted by ArduPilot (%d items)", len(items))


def build_loiter_items(plan: dict) -> list:
    waypoints = plan.get("waypoints", [])
    if not waypoints:
        raise ValueError("single_loiter plan has no waypoint")

    wp = waypoints[0]
    lat = float(wp["lat"])
    lon = float(wp["lon"])
    alt_m = float(wp.get("alt_m", LOITER_ALT_M))
    loiter_radius_m = float(plan.get("loiter_radius_m", LOITER_RADIUS_M))

    lat_i = int(lat * 1e7)
    lon_i = int(lon * 1e7)

    return [
        dict(
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current=0, autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=lat_i, y=lon_i, z=alt_m,
        ),
        dict(
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current=1, autocontinue=1,
            param1=0, param2=30.0, param3=0, param4=float("nan"),
            x=lat_i, y=lon_i, z=alt_m,
        ),
        dict(
            command=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current=0, autocontinue=0,
            param1=0, param2=0, param3=loiter_radius_m, param4=float("nan"),
            x=lat_i, y=lon_i, z=alt_m,
        ),
    ]


def build_waypoint_items(plan: dict) -> list:
    waypoints = plan.get("waypoints", [])
    if not waypoints:
        raise ValueError("waypoint_pattern plan has no waypoints")

    items = []

    first = waypoints[0]
    first_lat_i = int(float(first["lat"]) * 1e7)
    first_lon_i = int(float(first["lon"]) * 1e7)
    first_alt_m = float(first.get("alt_m", LOITER_ALT_M))

    # ArduPilot placeholder/home item
    items.append(
        dict(
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current=0, autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=first_lat_i, y=first_lon_i, z=first_alt_m,
        )
    )

    for i, wp in enumerate(waypoints, start=1):
        lat_i = int(float(wp["lat"]) * 1e7)
        lon_i = int(float(wp["lon"]) * 1e7)
        alt_m = float(wp.get("alt_m", LOITER_ALT_M))

        items.append(
            dict(
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                current=1 if i == 1 else 0,
                autocontinue=1,
                param1=0,
                param2=30.0,
                param3=0,
                param4=float("nan"),
                x=lat_i,
                y=lon_i,
                z=alt_m,
            )
        )

    return items

def mav_upload_plan(mav, plan: dict) -> None:
    plan_type = plan.get("plan_type")

    if plan_type == "single_loiter":
        items = build_loiter_items(plan)
    elif plan_type == "waypoint_pattern":
        items = build_waypoint_items(plan)
    else:
        raise ValueError(f"Unknown plan_type: {plan_type}")

    mav_upload_mission_items(mav, items)


def mav_start_mission(mav) -> None:
    """Switch to AUTO so ArduPilot executes the uploaded mission."""
    mav_set_mode(mav, "AUTO")
    log.info("pymavlink: AUTO mode commanded — mission executing")

def mav_loiter_in_place(mav) -> None:
    mav_set_mode(mav, "LOITER")
    log.info("pymavlink: LOITER mode commanded")


    
async def run():
    #Connection
    drone = System()
    
    
    #For Real
    # await drone.connect(system_address="udp://:14604")

    #For Sitl
    await drone.connect(system_address="udpin://0.0.0.0:14604")
  
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
    controller_status = load_state().get("controller_status", {})
    update_state("controller_status", {
        **controller_status,
        "autonomy_active": False,
        "safety_hold": None,
        "last_heartbeat_utc": None,
    })
    # last_upload_time = 0.0
    # last_processed_fix_id = None
    last_telemetry_time = time.time()
    state_period_s = 1.0 / STATE_POLL_HZ
    last_state_check = 0.0
    last_executed_plan_id = None

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

        controller_status = state.get("controller_status", {})

        update_state("controller_status", {
            **controller_status,
            "autonomy_active": autonomy_active,
            "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        })

        nav_state = load_nav_state()
        active_plan = nav_state.get("active_plan", {})

        mission_status = state.get("mission_status", {})
        should_autonomy = state.get("autonomy_command", False)
       

        
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
                controller_status = load_state().get("controller_status", {})
                update_state("controller_status", {
                    **controller_status,
                    "autonomy_active": False,
                    "safety_hold": "rtl",
                    "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                })
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
                controller_status = load_state().get("controller_status", {})
                update_state("controller_status", {
                    **controller_status,
                    "autonomy_active": False,
                    "safety_hold": "telemetry_lost",
                    "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                })
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
                controller_status = load_state().get("controller_status", {})
                update_state("controller_status", {
                    **controller_status,
                    "autonomy_active": False,
                    "safety_hold": "pilot_override",
                    "last_heartbeat_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
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
            controller_status = load_state().get("controller_status", {})
            update_state("controller_status", {
                **controller_status,
                "autonomy_active": False,
            })
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
            controller_status = load_state().get("controller_status", {})
            update_state("controller_status", {
                **controller_status,
                "autonomy_active": True, #Is this right ?
                
            })

            if mission_status.get("current_mode") != "Idle":
                update_state("mission_status", {
                    **mission_status,
                    "current_mode": "Idle",
                })

        elif not should_autonomy and autonomy_active:
            # Stopping Autonomy
            log.info("Autonomy disabled")
            autonomy_active = False
            controller_status = load_state().get("controller_status", {})
            update_state("controller_status", {
                **controller_status,
                "autonomy_active": False, #is this right ?
                
            })
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

        plan_id = active_plan.get("plan_id")
        plan_type = active_plan.get("plan_type")
        plan_status = active_plan.get("status")

        if (
            plan_id is not None
            and plan_status == "ready"
            and plan_id != last_executed_plan_id
        ):
            log.info("New plan detected: id=%s type=%s", plan_id, plan_type)

            try:
                # Upload mission
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: mav_upload_plan(mav, active_plan),
                )

                # Start mission (AUTO mode)
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: mav_start_mission(mav),
                )

                last_executed_plan_id = plan_id

                # Mark plan as running
                update_nav_state("active_plan", {
                    **active_plan,
                    "status": "running",
                })

                # Update mission status
                update_state("mission_status", {
                    **mission_status,
                    "active_plan_id": plan_id,
                    "current_mode": "Executing",
                })

                log.info("Plan %s uploaded and started", plan_id)

            except Exception as exc:
                log.error("Failed to execute plan %s: %s", plan_id, exc)

                update_nav_state("active_plan", {
                    **active_plan,
                    "status": "error",
                })

                update_state("mission_status", {
                    **mission_status,
                    "active_plan_id": plan_id,
                    "current_mode": "PlanError",
                })

        await asyncio.sleep(state_period_s)
        continue

if __name__ == "__main__":
  asyncio.run(run())    
