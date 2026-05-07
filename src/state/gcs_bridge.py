#!/usr/bin/env python3

import time
import json
from pathlib import Path
from datetime import datetime, timezone

from state.nav_state_utils import update_nav_state

TELEMETRY_FILE = Path("/tmp/telemetry.json")
POLL_DT_S = 0.2

SEARCH_AREA_ZONE_ID = 1  # change this to whatever GCS defines as search area

def find_search_area_zone(zones):
    if not isinstance(zones, list):
        return []

    for zone in zones:
        zone_id = zone.get("zone_id")
        coords = zone.get("coordinates")

        if zone_id == SEARCH_AREA_ZONE_ID:
            return coords

    return []


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path):
    if not path.exists():
        return None

    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"[STATE BRIDGE] Failed to read {path}: {e}")
        return None


def valid_coord(lat, lon):
    if lat is None or lon is None:
        return False

    try:
        lat = float(lat)
        lon = float(lon)
    except (TypeError, ValueError):
        return False

    return lat != 0.0 and lon != 0.0


def valid_search_area(search_area):
    if not isinstance(search_area, list):
        return False

    if len(search_area) < 3:
        return False

    for point in search_area:
        if not isinstance(point, list) and not isinstance(point, tuple):
            return False

        if len(point) != 2:
            return False

        lat, lon = point
        if not valid_coord(lat, lon):
            return False

    return True


def set_target_location(lat, lon, source, timestamp):
    update_nav_state("target_location", {
        "lat": float(lat),
        "lon": float(lon),
        "source": source,
        "timestamp": timestamp,
        "valid": True
    })


def main():
    print("[STATE BRIDGE] Starting Pi5 target/search-area bridge")

    last_search_area = None
    last_refined = None
    last_final = None
    last_eru = None

    while True:
        data = read_json(TELEMETRY_FILE)

        if not data:
            time.sleep(POLL_DT_S)
            continue

        timestamp = utc_now_iso()

        # ------------------------------------------------------------
        # 1. Search area from GCS
        # ------------------------------------------------------------
        zones = data.get("zones", [])
        search_area = find_search_area_zone(zones)
        if valid_search_area(search_area):
            current = json.dumps(search_area, sort_keys=True)

            if current != last_search_area:
                print(f"[STATE BRIDGE] Search area received with {len(search_area)} points")

                update_nav_state("search_area", search_area)

                last_search_area = current

        # ------------------------------------------------------------
        # 2. MRA refined loiter target from Kraken
        # ------------------------------------------------------------
        # FIRST Kraken transmit.
        # Updates ONLY mra_refined_loiter_target.
        # Does NOT update target_location.
        refined_lat = data.get("mra_refined_lat")
        refined_lon = data.get("mra_refined_lon")

        if valid_coord(refined_lat, refined_lon):
            current = (float(refined_lat), float(refined_lon))

            if current != last_refined:
                print(f"[STATE BRIDGE] MRA refined loiter target: {current}")

                update_nav_state("mra_refined_loiter_target", {
                    "lat": float(refined_lat),
                    "lon": float(refined_lon),
                    "confidence": data.get("mra_refined_confidence"),
                    "timestamp": timestamp,
                    "fix_id": data.get("mra_refined_fix_id", "mra_refined_001"),
                    "valid": True
                })

                last_refined = current

        # ------------------------------------------------------------
        # 3. MRA final estimated location from Kraken
        # ------------------------------------------------------------
        # SECOND Kraken transmit.
        # Updates mra_final_estimated_location and target_location.
        final_lat = data.get("mra_final_lat")
        final_lon = data.get("mra_final_lon")

        if valid_coord(final_lat, final_lon):
            current = (float(final_lat), float(final_lon))

            if current != last_final:
                print(f"[STATE BRIDGE] MRA final estimated location: {current}")

                update_nav_state("mra_final_estimated_location", {
                    "lat": float(final_lat),
                    "lon": float(final_lon),
                    "confidence": data.get("mra_final_confidence"),
                    "timestamp": timestamp,
                    "fix_id": data.get("mra_final_fix_id", "mra_final_001"),
                    "valid": True
                })

                set_target_location(
                    final_lat,
                    final_lon,
                    "mra_final_estimated_location",
                    timestamp
                )

                last_final = current

        # ------------------------------------------------------------
        # 4. ERU reported location from GCS
        # ------------------------------------------------------------
        # Comes from GCS PatientLocation command.
        # Updates eru_reported_location and target_location.
        eru_lat = data.get("eru_lat")
        eru_lon = data.get("eru_lon")

        if valid_coord(eru_lat, eru_lon):
            current = (float(eru_lat), float(eru_lon))

            if current != last_eru:
                print(f"[STATE BRIDGE] ERU reported location: {current}")

                update_nav_state("eru_reported_location", {
                    "lat": float(eru_lat),
                    "lon": float(eru_lon),
                    "confidence": None,
                    "timestamp": timestamp,
                    "fix_id": data.get("eru_fix_id", "eru_001"),
                    "valid": True
                })

                set_target_location(
                    eru_lat,
                    eru_lon,
                    "eru_reported_location",
                    timestamp
                )

                last_eru = current

        time.sleep(POLL_DT_S)


if __name__ == "__main__":
    main()