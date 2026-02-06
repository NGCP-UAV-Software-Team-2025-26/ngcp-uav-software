# GCS-MRA Interface Control Document

| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|:-------------:|:--------:|:----------:|:-----------:|:-------------:|:----------------:|:-------------:|
| HEARTBEAT | MRA | GCS | 1 Hz | mra_state, flight_mode, armed, in_air, last_cmd_seq, link_rssi, battery_pct | SEARCH, AUTO, true, true, 1842, -68, 74 | Basic liveness and state heartbeat. Used to detect loss-of-link and state mismatches. |
| STATUS_REPORT | MRA | GCS | 1–2 Hz | lat, lon, alt_m, vel_mps, heading_deg, battery_pct, est_time_remaining_s, gps_fix, hdop, payload_status, current_task | 34.0219, -118.4814, 120.5, 12.2, 87, 72, 900, 3D, 0.9, READY, SEARCH | Full operational telemetry. |
| TIME_SYNC | GCS | MRA | On connect / hourly | timestamp_utc, time_source | 2026-02-06T21:14:33Z, GNSS | Synchronizes time for logs and staleness checks. |
| MISSION_ASSIGN | GCS | MRA | Event-driven | mission_id, search_area_polygon, search_alt_m, search_pattern, geofence_polygon, rtb_point, drop_constraints | M-381, [...], 120, lawnmower, [...], (34.02,-118.48), min_alt=30 | Provides mission geometry and constraints. |
| MISSION_ACK | MRA | GCS | Event-driven | mission_id, accepted, reject_reason | M-381, true, null | MRA confirms mission acceptance or rejection. |
| PREFLIGHT_STATUS | MRA | GCS | On request / pre-takeoff | checks_passed, failures[], battery_pct, gps_ok, payload_ok, storage_ok | false, [PAYLOAD_NOT_DETECTED], 88, true, false, true | Preflight readiness report. |
| TAKEOFF_REQUEST | MRA | GCS | Event-driven | requested_time_window, requested_alt_m, launch_site, reason, airspace_slot_request_id | 21:20–21:30Z, 120, (34.02,-118.48), Start mission, SLOT-77 | MRA requests permission to take off. |
| TAKEOFF_CLEARANCE | GCS | MRA | Event-driven | approved, clearance_id, valid_until, assigned_slot_id, conditions | true, CLR-991, 21:28Z, SLOT-77, max_alt=150 | GCS grants takeoff permission. |
| TAKEOFF_DENIED | GCS | MRA | Event-driven | approved=false, deny_reason, retry_after_s | false, MEA_IN_AIR, 120 | GCS denies takeoff request. |
| TAKEOFF_STARTED | MRA | GCS | Event-driven | clearance_id, takeoff_time, takeoff_latlon, target_alt_m | CLR-991, 21:23Z, (..), 120 | MRA commits to takeoff. |
| TAKEOFF_ABORTED | MRA | GCS | Event-driven | clearance_id, abort_reason, safe_state | CLR-991, MOTOR_FAULT, DISARMED | MRA aborts takeoff safely. |
| CMD_HOLD | GCS | MRA | Event-driven | hold_type, hold_point, radius_m, duration_s | LOITER, (34.02,-118.48,120), 80, 300 | Commands MRA to loiter/hold position. |
| CMD_RTB | GCS | MRA | Event-driven | rtb_reason, rtb_point, land_on_arrival | LINK_RISK, (..), true | Commands MRA to return to base. |
| CMD_ABORT_MISSION | GCS | MRA | Event-driven | abort_reason, immediate_action | AIRSPACE_CONFLICT, RTB | Terminates mission execution. |
| MODE_CHANGE | GCS | MRA | Event-driven | requested_mode, reason | AUTO, Resume mission | Explicit flight mode change. |
| SEARCH_START | GCS | MRA | Event-driven | mission_id, start_time, search_area_id | M-381, 21:24Z, AREA-A | Authorizes search task execution. |
| SEARCH_PROGRESS | MRA | GCS | 0.2–1 Hz | search_area_id, percent_covered, last_sweep_line_id, detections_count | AREA-A, 43.2, L12, 0 | Search coverage status. |
| DETECTION_REPORT | MRA | GCS | Event-driven | detection_type, lat, lon, confidence, sensor, snapshot_ref, uncertainty_m | SURVIVOR, (..), 0.87, EO, img_01821, 6.5 | Candidate detection report. |
| SURVIVOR_FOUND | MRA | GCS | Event-driven | survivor_id, lat, lon, confidence, timestamp_utc, uncertainty_m, method | S-1, (..), 0.93, 21:29Z, 4.2, VISION | MRA confirms survivor found. |
| SURVIVOR_FOUND_ACK | GCS | MRA | Event-driven | survivor_id, received, next_action | S-1, true, PROCEED_TO_DROP | GCS acknowledges survivor detection. |
| SURVIVOR_FOUND_FROM_GCS | GCS | MRA | Event-driven | survivor_id, lat, lon, source, confidence, timestamp_utc, uncertainty_m | S-1, (..), UGV, 0.88, 21:27Z, 7.0 | GCS reports survivor found by UGV. |
| LOCATION_CONFLICT | MRA | GCS | Event-driven | survivor_id, loc_a, loc_b, delta_m, recommended_action | S-1, MRA_LOC, UGV_LOC, 35.4, REQUEST_RECONFIRM | Location discrepancy detected. |
| NAV_TO_SURVIVOR_STARTED | MRA | GCS | Event-driven | survivor_id, eta_s, route_summary | S-1, 180, direct | MRA navigating to survivor. |
| DROP_ARM_REQUEST | MRA | GCS | Event-driven | survivor_id, planned_drop_point, planned_alt_m, wind_est_mps, safety_checks | S-1, (..), 35, 6.2, zone_clear=true | Requests payload arming. |
| DROP_ARM_CLEARANCE | GCS | MRA | Event-driven | approved, clearance_id, constraints | true, DROPCLR-55, min_alt=25 | Authorizes payload arming. |
| DROP_DENIED | GCS | MRA | Event-driven | approved=false, deny_reason, next_step | false, CIVILIANS_NEARBY, HOLD | Denies payload drop. |
| DROP_EXECUTE | MRA | GCS | Event-driven | clearance_id, actual_drop_point, drop_time, altitude_m | DROPCLR-55, (..), 21:34Z, 32 | MRA executes drop. |
| DROP_RESULT | MRA | GCS | Event-driven | clearance_id, success, reason_if_fail, payload_state, evidence_ref | DROPCLR-55, true, null, EMPTY, img_01910 | Drop result confirmation. |
| POST_TASK_STATUS | MRA | GCS | Event-driven | task_completed, next_intent, battery_pct, eta_rtb_s | DROP, RTB, 48, 210 | Post-task summary. |
| CMD_LAND | GCS | MRA | Event-driven | land_point, approach_heading_deg, reason | (..), 270, End mission | Commands landing. |
| LINK_STATUS | MRA | GCS | 0.5–1 Hz | link_state, rtt_ms, packet_loss_pct, rssi | DEGRADED, 180, 12.5, -79 | Communications health telemetry. |
| LOST_LINK_EVENT | MRA | GCS | Event-driven | last_contact_time, failsafe_action, current_pos | 21:35Z, RTL, (..) | MRA enters lost-link failsafe. |
| LINK_RECOVERED_EVENT | MRA | GCS | Event-driven | recovery_time, current_mode, resume_capable | 21:36Z, LOITER, true | Link restored notification. |
| FDIR_EVENT | MRA | GCS | Event-driven | fault_id, fault_class, severity, detected_by, effect, mitigation_action | BAT_LOW, POWER, WARN, BMS, reduced_endurance, RTB | Fault detection and recovery reporting. |
| HEALTH_DETAIL | MRA | GCS | On request / 0.2 Hz | subsystem, metrics, thresholds | GPS, sat=9 hdop=2.1, hdop_warn=2.0 | Detailed subsystem health. |
| CMD_REQUEST_HEALTH | GCS | MRA | Event-driven | subsystems[] | GPS, PAYLOAD, MOTORS | Requests detailed health data. |
| ACK | Either | Either | Per message | ack_for_msg_id, status, error_code, error_text | c0a8…, OK, null, null | Generic ACK/NACK message. |