import json
from pathlib import Path

STATE_FILE = Path(__file__).resolve().parent / "mission_state.json"

DEFAULTS = {
    "logging_enabled": False,
    "last_command": None,
    "timestamp": None,
    "last_sender_sysid": None,
    "last_sender_compid": None,
    "autonomy_command": False, #The command that GCS sends

    "kraken_log": None,
    "telemetry_log": None,
    "fusion_log": None,
    
    "controller_status": {
        "fc_mode": None,
        "autonomy_active": False, #What the controller writes 
        "autonomy_source": None,
        "rtl_reason" :None,
        "fc_mode_last_updated": None,
        "safety_hold": None, #Explains why autonomy is suspended (Pilot/below min_alt)
        "last_heartbeat_utc": None, #lets the gcs know if the controller process has failed
        "last_rtl_event": 
        {
            "reason": None,
            "source": None,
            "timestamp": None,
            "fc_mode": None,
            "previous_fc_mode": None,
        }
    },
    


    "pending_action": None,
    "rtl_requested": False,
    "loiter_requested": False,
    

    "mission_status": {
        "active_plan_id": None,
        "last_completed_plan_id": None,
        "mission_count": 0,
        "current_mode": None,
        "active_waypoint_index": None,
    },

}

def load_state() -> dict:
    try:
        if STATE_FILE.exists():
            loaded = json.loads(STATE_FILE.read_text())
            return _merge_dicts(DEFAULTS, loaded)
    except Exception:
        pass
    return json.loads(json.dumps(DEFAULTS))

def update_state(key: str, value) -> None:
    state = load_state()
    state[key] = value
    STATE_FILE.write_text(json.dumps(state, indent=2))

def _merge_dicts(default: dict, override: dict) -> dict:
    result = dict(default)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = _merge_dicts(result[k], v)
        else:
            result[k] = v
    return result