#!/usr/bin/env python3
"""
fix_simulator.py
Simulates best_fix updates and control flags for the simplified search-based
main_controller flow.

Scenarios focus on:
- starting a new search session
- updating best_fix over time
- autonomy pause/resume
- RTL requests
- fallback readiness
"""

import time
import random
import argparse
import logging
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).resolve().parents[1]))
from state.state_utils import load_state, update_state

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [SIM] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("fix_simulator")

# Canberra SITL-ish area
BASE_LAT = -35.363
BASE_LON = 149.1650
JITTER = 0.012

SCENARIOS = [
    "new_search_session",
    "best_fix_good",
    "best_fix_better",
    "best_fix_weak",
    "autonomy_off",
    "rtl",
]


def _fix(lat, lon, conf, fix_id):
    return {
        "fix_id": fix_id,
        "lat": lat,
        "lon": lon,
        "confidence": round(conf, 3),
        "timestamp": time.time(),
    }


def _good_fix(fix_id):
    return _fix(
        BASE_LAT + random.uniform(-JITTER, JITTER),
        BASE_LON + random.uniform(-JITTER, JITTER),
        random.uniform(0.65, 0.85),
        fix_id,
    )


def _better_fix(fix_id):
    return _fix(
        BASE_LAT + random.uniform(-JITTER * 0.4, JITTER * 0.4),
        BASE_LON + random.uniform(-JITTER * 0.4, JITTER * 0.4),
        random.uniform(0.86, 0.98),
        fix_id,
    )


def _weak_fix(fix_id):
    return _fix(
        BASE_LAT + random.uniform(-JITTER, JITTER),
        BASE_LON + random.uniform(-JITTER, JITTER),
        random.uniform(0.20, 0.45),
        fix_id,
    )


def clear_best_fix():
    update_state("best_fix", {
        "fix_id": None,
        "lat": None,
        "lon": None,
        "confidence": None,
        "timestamp": None,
    })


def run_simulator(interval: float, cycles: int, scenario: str | None):
    state = load_state()
    fix_id = int(state.get("best_fix", {}).get("fix_id") or 0) + 1
    scenario_list = [scenario] if scenario else SCENARIOS

    log.info(
        "Simulator starting. interval=%.1fs cycles=%d scenarios=%s",
        interval, cycles, scenario_list
    )
    log.info(
        "Writing to: %s",
        Path(__file__).resolve().parents[1] / "mission_state.json"
    )

    for cycle in range(cycles):
        sc = scenario_list[cycle % len(scenario_list)]
        log.info("── Cycle %d / %d scenario=%-18s next_fix_id=%d",
                 cycle + 1, cycles, sc, fix_id)

        state = load_state()

        if sc != "autonomy_off":
            if not state.get("autonomy_active"):
                update_state("autonomy_active", True)
                log.info("  autonomy_active → True")

        if sc == "new_search_session":
            update_state("pending_action", "new_search_session")
            clear_best_fix()
            log.info("  pending_action → new_search_session")
            time.sleep(1.0)

        elif sc == "best_fix_good":
            fix = _good_fix(fix_id)
            update_state("best_fix", fix)
            log.info(
                "  BEST FIX GOOD lat=%.5f lon=%.5f conf=%.2f fix_id=%d",
                fix["lat"], fix["lon"], fix["confidence"], fix_id
            )
            fix_id += 1

        elif sc == "best_fix_better":
            current_best = state.get("best_fix", {})
            old_conf = current_best.get("confidence")
            fix = _better_fix(fix_id)
            update_state("best_fix", fix)
            log.info(
                "  BEST FIX BETTER old_conf=%s new_conf=%.2f fix_id=%d",
                f"{old_conf:.2f}" if isinstance(old_conf, (int, float)) else "None",
                fix["confidence"],
                fix_id,
            )
            fix_id += 1

        elif sc == "best_fix_weak":
            fix = _weak_fix(fix_id)
            update_state("best_fix", fix)
            log.info(
                "  BEST FIX WEAK lat=%.5f lon=%.5f conf=%.2f fix_id=%d",
                fix["lat"], fix["lon"], fix["confidence"], fix_id
            )
            fix_id += 1

        elif sc == "autonomy_off":
            update_state("autonomy_active", False)
            log.info("  autonomy_active → False (expect controller to loiter/pause)")
            time.sleep(interval)
            update_state("autonomy_active", True)
            log.info("  autonomy_active → True (restored)")
            continue

        elif sc == "rtl":
            update_state("rtl_requested", True)
            log.info("  rtl_requested → True")
            time.sleep(interval)

            if load_state().get("rtl_requested"):
                update_state("rtl_requested", False)
                log.info("  rtl_requested reset by simulator")
            else:
                log.info("  rtl_requested cleared by controller ✓")
            continue

        time.sleep(interval)

    log.info("Simulator finished after %d cycles.", cycles)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Simulate best_fix updates for simplified main_controller"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=10.0,
        help="Seconds between scenario steps"
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=20,
        help="Total scenario steps to run"
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default=None,
        choices=SCENARIOS,
        help="Run only one scenario repeatedly instead of cycling"
    )
    args = parser.parse_args()

    run_simulator(
        interval=args.interval,
        cycles=args.cycles,
        scenario=args.scenario,
    )