import json
from pathlib import Path

NAV_STATE_FILE = Path(__file__).resolve().parent / "navigation_state.json"

DEFAULTS = {
    "search_area": None,

    "mra_refined_loiter_target": {
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "fix_id": None,
        "valid": False
    },

    "mra_final_estimated_location": {
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "fix_id": None,
        "valid": False
    },

    "eru_reported_location": {
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "report_id": None,
        "valid": False
    },

    "target_location": {
        "source": None,
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
        "id": None,
        "valid": False
    },

    "active_plan": {
        "plan_id": None,
        "plan_type": None,
        "status": None,
        "waypoints": [],
        "loiter_radius_m": None,
        "alt_m": None,
    },

    "next_plan": None,

    "navigation": {
        "mission_phase": None,
        "current_waypoint": None,
        "guidance_waypoint": None,
    }
}


def load_nav_state() -> dict:
    try:
        if NAV_STATE_FILE.exists():
            loaded = json.loads(NAV_STATE_FILE.read_text())
            return _merge_dicts(DEFAULTS, loaded)
    except Exception:
        pass
    return json.loads(json.dumps(DEFAULTS))


def update_nav_state(key: str, value) -> None:
    state = load_nav_state()
    state[key] = value
    NAV_STATE_FILE.write_text(json.dumps(state, indent=2))


def _merge_dicts(default: dict, override: dict) -> dict:
    result = dict(default)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = _merge_dicts(result[k], v)
        else:
            result[k] = v
    return result