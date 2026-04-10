import json
from pathlib import Path

STATE_FILE = Path(__file__).resolve().parent / "mission_state.json"

DEFAULTS = {
    "logging_enabled": False,
    "last_command": None,
    "timestamp": None,
    "last_sender_sysid": None,
    "last_sender_compid": None,
    "kraken_log": None,
    "telemetry_log": None,
    "fusion_log": None,
    "autonomy_active": False,

    "search_phase": {
        "start_time":           None,
        "time_limit_s":         480,
        "session_id":           None
    },
    "pending_action": None,
    "rtl_requested": False,
    "target_fix": {
        "fix_id": None,
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
    },

    "best_fix": {
        "fix_id":       None,
        "lat":          None,
        "lon":          None,
        "confidence":   None,
        "timestamp":    None
    },

    "mission_status": {
        "active_target_fix_id": None,
        "last_processed_fix_id": None,
        "mission_count": 0,
        "current_mode": None,
    },

    "navigation": {
        "search_area": None,
        "mission_phase": None,
        "current_waypoint": None,
        "guidance_waypoint": None,
    }
}

def load_state() -> dict:
    try:
        if STATE_FILE.exists():
            loaded = json.loads(STATE_FILE.read_text())
            return _merge_dicts(DEFAULTS, loaded)
    except Exception:
        pass
    return dict(DEFAULTS)

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