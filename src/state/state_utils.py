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
    "autonomy_enabled": False,
    "pending_action": None,
    "rtl_requested": False,

    "target_fix": {
        "lat": None,
        "lon": None,
        "alt": None,
        "confidence": None,
        "fix_id": None,
        "timestamp": None,
    },

    "mission_status": {
        "active_target_fix_id": None,
        "last_processed_fix_id": None,
        "mission_count": 0,
        "current_mode": None,
    }
}

def load_state() -> dict:
    try:
        if STATE_FILE.exists():
            return {**DEFAULTS, **json.loads(STATE_FILE.read_text())}
    except Exception:
        pass
    return dict(DEFAULTS)

def update_state(key: str, value) -> None:
    state = load_state()
    state[key] = value
    STATE_FILE.write_text(json.dumps(state, indent=2))