import time
import uuid



SEARCH_LIMIT_S = 480      # 8 minutes total
FALLBACK_TRIGGER_S = 420  # 7 minutes


def has_best_fix(state: dict) -> bool:
    """Return True if best_fix has usable coordinates."""
    best = state.get("best_fix", {})
    return best.get("lat") is not None and best.get("lon") is not None


def valid_fix(fix: dict, min_confidence: float = 0.4) -> bool:
    """Return True if the fix has usable lat/lon and enough confidence."""
    if not fix:
        return False

    try:
        lat = fix.get("lat")
        lon = fix.get("lon")
        conf = fix.get("confidence", 0.0)

        if lat is None or lon is None:
            return False
        if conf < min_confidence:
            return False
        return True
    except Exception:
        return False


def get_elapsed(state: dict) -> float:
    # Search time
    start = state.get("search_phase", {}).get("start_time")
    if start is None:
        return 0.0
    return time.time() - start


def start_search_phase(state: dict) -> dict:
    # starts search timer
    phase = state.get("search_phase", {})
    if phase.get("start_time") is not None:
        return state

    session_id = str(uuid.uuid4())[:8]

    state["search_phase"] = {
        "start_time": time.time(),
        "time_limit_s": SEARCH_LIMIT_S,
        "session_id": session_id,
    }

    state["decision"] = {
        "fallback_triggered": False,
        "fallback_time": None,
        "active_mission_fix_id": None,
    }

    # Only initialize best_fix if it doesn't already exist
    if "best_fix" not in state:
        state["best_fix"] = {
            "fix_id": None,
            "lat": None,
            "lon": None,
            "confidence": None,
            "timestamp": None,
        }

    return state


def reset_search_phase(state: dict) -> dict:
   
    # Brand new search session
    state["search_phase"] = {"start_time": None}
    state["decision"] = {
        "fallback_triggered": False,
        "fallback_time": None,
        "active_mission_fix_id": None,
    }
    state["best_fix"] = {
        "fix_id": None,
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
    }
    return start_search_phase(state)


def update_elapsed(state: dict) -> dict:
    # Updates the time
    phase = state.get("search_phase", {})
    if phase.get("start_time") is not None:
        phase["elapsed_s"] = round(get_elapsed(state), 1)
        state["search_phase"] = phase
    return state


def update_best_fix(state: dict, candidate_fix: dict) -> dict:
    """
    Replace best_fix if candidate has higher confidence.
    Assumes confidence is decided elsewhere.
    """
    if not candidate_fix:
        return state

    if candidate_fix.get("lat") is None or candidate_fix.get("lon") is None:
        return state

    current_best = state.get("best_fix", {})
    current_conf = current_best.get("confidence")
    new_conf = candidate_fix.get("confidence")

    if new_conf is None:
        return state
        
    if new_conf is None or new_conf < 0.4:
        return state

    if current_conf is None or new_conf > current_conf:
        state["best_fix"] = {
            "fix_id": candidate_fix.get("fix_id"),
            "lat": candidate_fix.get("lat"),
            "lon": candidate_fix.get("lon"),
            "confidence": new_conf,
            "timestamp": candidate_fix.get("timestamp"),
        }

    return state


def check_time_gates(state: dict) -> tuple[dict, str | None]:
    """
    Returns:
      - (state, "FALLBACK_FIX") at 7 minutes if best_fix exists and fallback not used
      - (state, "SEARCH_TIMEOUT") at 8 minutes
      - (state, None) otherwise
    """
    elapsed = get_elapsed(state)
    decision = state.get("decision", {})

    if elapsed >= SEARCH_LIMIT_S:
        return state, "SEARCH_TIMEOUT"

    if (
        elapsed >= FALLBACK_TRIGGER_S
        and not decision.get("fallback_triggered")
        and has_best_fix(state)
    ):
        decision["fallback_triggered"] = True
        decision["fallback_time"] = time.time()
        state["decision"] = decision
        return state, "FALLBACK_FIX"

    return state, None