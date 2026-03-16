#!/usr/bin/env python3
"""
SAR Flight Path Plotter
Watches the sar_mission.json produced by sar_homing.py and redraws the
flight plan live whenever the file changes.

Dependencies:
    pip install matplotlib

Usage:
    python sar_plot.py [--mission FILE] [--interval MS]
"""

import argparse
import json
import math
import os
import sys
import time

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.patches import FancyArrowPatch

# ──────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────
DEFAULT_MISSION_FILE = "sar_mission.json"
DEFAULT_INTERVAL_MS  = 1000          # redraw poll cadence
EARTH_RADIUS_M       = 6_371_000.0

MAV_CMD_NAV_WAYPOINT     = 16
MAV_CMD_NAV_LOITER_UNLIM = 17

COLORS = {
    "bg":        "#0d1117",
    "grid":      "#1e2a38",
    "home":      "#00d4ff",
    "waypoint":  "#f0a500",
    "target":    "#ff4d4d",
    "path":      "#39d353",
    "loiter":    "#ff4d4d",
    "arrow":     "#39d353",
    "text":      "#e6edf3",
    "subtext":   "#8b949e",
    "panel_bg":  "#161b22",
    "border":    "#30363d",
}

# ──────────────────────────────────────────────
# Geometry helpers
# ──────────────────────────────────────────────

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

def latlon_to_xy(lat, lon, ref_lat, ref_lon):
    """Project (lat, lon) to local (x, y) metres relative to reference point."""
    cos_ref = math.cos(deg2rad(ref_lat))
    x = (lon - ref_lon) * deg2rad(1) * EARTH_RADIUS_M * cos_ref
    y = (lat - ref_lat) * deg2rad(1) * EARTH_RADIUS_M
    return x, y

def loiter_circle_xy(cx, cy, radius_m, n=120):
    """Return (xs, ys) arrays for a loiter circle in local metres."""
    angles = [i * 2 * math.pi / n for i in range(n + 1)]
    xs = [cx + radius_m * math.sin(a) for a in angles]
    ys = [cy + radius_m * math.cos(a) for a in angles]
    return xs, ys

# ──────────────────────────────────────────────
# Mission parser
# ──────────────────────────────────────────────

def parse_mission(path: str):
    """
    Returns a dict with:
      waypoints  – list of {x, y, alt_m, cmd, label}
      target     – {x, y}
      loiter_r   – radius in metres
      meta       – raw _meta dict
      ref        – (ref_lat, ref_lon) used for projection
    """
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)

    items  = data["mission"]["items"]
    meta   = data.get("_meta", {})
    home   = data["mission"]["plannedHomePosition"]   # [lat, lon, alt]
    ref_lat, ref_lon = home[0], home[1]

    waypoints = []
    loiter_r  = 0.0

    for i, item in enumerate(items):
        params = item["params"]
        lat, lon, alt = params[4], params[5], params[6]
        cmd = item["command"]

        x, y = latlon_to_xy(lat, lon, ref_lat, ref_lon)

        if cmd == MAV_CMD_NAV_LOITER_UNLIM:
            loiter_r = abs(params[2])   # param3 = radius
            label = "Loiter / Target"
        elif i == 0:
            label = "Home (current pos)"
        else:
            label = f"WP{i}"

        waypoints.append({
            "x": x, "y": y,
            "alt_m": alt if not (isinstance(alt, float) and math.isnan(alt)) else 0.0,
            "cmd": cmd,
            "label": label,
            "lat": lat, "lon": lon,
        })

    target_lat = meta.get("target_lat")
    target_lon = meta.get("target_lon")
    if target_lat is not None:
        tx, ty = latlon_to_xy(target_lat, target_lon, ref_lat, ref_lon)
        target = {"x": tx, "y": ty, "lat": target_lat, "lon": target_lon}
    else:
        target = None

    return {
        "waypoints": waypoints,
        "target":    target,
        "loiter_r":  loiter_r,
        "meta":      meta,
        "ref":       (ref_lat, ref_lon),
    }

# ──────────────────────────────────────────────
# Plot renderer
# ──────────────────────────────────────────────

def draw(ax_main, ax_info, mission: dict):
    ax_main.clear()
    ax_info.clear()

    wps      = mission["waypoints"]
    target   = mission["target"]
    loiter_r = mission["loiter_r"]
    meta     = mission["meta"]

    # ── Flight path line ──────────────────────────────────────────────
    xs = [w["x"] for w in wps]
    ys = [w["y"] for w in wps]

    ax_main.plot(xs, ys,
                 color=COLORS["path"], linewidth=1.8,
                 linestyle="--", alpha=0.7, zorder=2)

    # Direction arrows along each leg
    for i in range(len(wps) - 1):
        mx = (wps[i]["x"] + wps[i+1]["x"]) / 2
        my = (wps[i]["y"] + wps[i+1]["y"]) / 2
        dx = wps[i+1]["x"] - wps[i]["x"]
        dy = wps[i+1]["y"] - wps[i]["y"]
        mag = math.hypot(dx, dy)
        if mag > 0:
            ax_main.annotate("",
                xy=(mx + dx/mag*0.1, my + dy/mag*0.1),
                xytext=(mx - dx/mag*0.1, my - dy/mag*0.1),
                arrowprops=dict(arrowstyle="-|>",
                                color=COLORS["arrow"],
                                lw=1.5),
                zorder=3)

    # ── Loiter circle ─────────────────────────────────────────────────
    loiter_wp = next((w for w in wps if w["cmd"] == MAV_CMD_NAV_LOITER_UNLIM), None)
    if loiter_wp and loiter_r > 0:
        lxs, lys = loiter_circle_xy(loiter_wp["x"], loiter_wp["y"], loiter_r)
        ax_main.plot(lxs, lys,
                     color=COLORS["loiter"], linewidth=1.2,
                     linestyle=":", alpha=0.6, zorder=2)

    # ── Waypoint markers ──────────────────────────────────────────────
    for w in wps:
        if w["cmd"] == MAV_CMD_NAV_LOITER_UNLIM:
            color  = COLORS["target"]
            marker = "*"
            size   = 220
            zorder = 6
        elif w["label"].startswith("Home"):
            color  = COLORS["home"]
            marker = "^"
            size   = 130
            zorder = 5
        else:
            color  = COLORS["waypoint"]
            marker = "o"
            size   = 90
            zorder = 5

        ax_main.scatter(w["x"], w["y"],
                        color=color, marker=marker, s=size,
                        zorder=zorder, edgecolors="#0d1117", linewidths=0.8)

        offset_x = 12
        offset_y = 12
        ax_main.annotate(
            w["label"],
            xy=(w["x"], w["y"]),
            xytext=(w["x"] + offset_x, w["y"] + offset_y),
            fontsize=7.5, color=COLORS["text"],
            fontfamily="monospace",
            zorder=7,
        )

    # ── Target ground truth marker (cross) ───────────────────────────
    if target:
        ax_main.scatter(target["x"], target["y"],
                        color=COLORS["target"], marker="x", s=160,
                        linewidths=2.5, zorder=7)

    # ── Axes styling ──────────────────────────────────────────────────
    ax_main.set_facecolor(COLORS["bg"])
    ax_main.tick_params(colors=COLORS["subtext"], labelsize=7)
    for spine in ax_main.spines.values():
        spine.set_edgecolor(COLORS["border"])
    ax_main.grid(True, color=COLORS["grid"], linewidth=0.5, linestyle="-")
    ax_main.set_xlabel("East  (m)", color=COLORS["subtext"], fontsize=8)
    ax_main.set_ylabel("North (m)", color=COLORS["subtext"], fontsize=8)
    ax_main.set_title("SAR Flight Path — Live Preview",
                      color=COLORS["text"], fontsize=10, pad=10,
                      fontfamily="monospace")
    ax_main.set_aspect("equal", adjustable="datalim")

    # Padding around extents
    all_x = xs + ([target["x"]] if target else [])
    all_y = ys + ([target["y"]] if target else [])
    if loiter_wp and loiter_r > 0:
        all_x += [loiter_wp["x"] - loiter_r, loiter_wp["x"] + loiter_r]
        all_y += [loiter_wp["y"] - loiter_r, loiter_wp["y"] + loiter_r]
    pad = max((max(all_x) - min(all_x)) * 0.12,
              (max(all_y) - min(all_y)) * 0.12, 50)
    ax_main.set_xlim(min(all_x) - pad, max(all_x) + pad)
    ax_main.set_ylim(min(all_y) - pad, max(all_y) + pad)

    # ── Legend ────────────────────────────────────────────────────────
    legend_items = [
        mpatches.Patch(color=COLORS["home"],     label="Home / current pos"),
        mpatches.Patch(color=COLORS["waypoint"], label="Waypoint"),
        mpatches.Patch(color=COLORS["target"],   label="Target / loiter"),
        mpatches.Patch(color=COLORS["path"],     label="Flight path"),
        mpatches.Patch(color=COLORS["loiter"],   label="Loiter circle"),
    ]
    leg = ax_main.legend(
        handles=legend_items,
        loc="lower left", fontsize=7,
        facecolor=COLORS["panel_bg"],
        edgecolor=COLORS["border"],
        labelcolor=COLORS["text"],
    )

    # ── Info panel ────────────────────────────────────────────────────
    ax_info.set_facecolor(COLORS["panel_bg"])
    ax_info.set_xlim(0, 1)
    ax_info.set_ylim(0, 1)
    ax_info.axis("off")

    ref_lat, ref_lon = mission["ref"]
    loiter_wp_data = next((w for w in wps if w["cmd"] == MAV_CMD_NAV_LOITER_UNLIM), None)

    lines = [
        ("MISSION SUMMARY", None, COLORS["text"], 9, True),
        ("", None, None, 6, False),
        ("Generated",      meta.get("generated_utc", "—"),      COLORS["subtext"], 7.5, False),
        ("Distance",       f"{meta.get('dist_to_target_m', '—')} m",               COLORS["text"],    8, False),
        ("Heading",        f"{meta.get('hdg_to_target_deg', '—')}°",               COLORS["text"],    8, False),
        ("Turn radius",    f"{meta.get('turn_radius_m', '—')} m",                  COLORS["text"],    8, False),
        ("Max bank",       f"{meta.get('max_bank_deg', '—')}°",                    COLORS["text"],    8, False),
        ("Approach alt",   f"{meta.get('approach_alt_ft', '—')} ft AGL",           COLORS["text"],    8, False),
        ("Loiter radius",  f"{meta.get('loiter_radius_ft', '—')} ft",              COLORS["text"],    8, False),
        ("", None, None, 5, False),
        ("HOME", None, COLORS["home"], 8, True),
        ("Lat",  f"{ref_lat:.6f}°",  COLORS["text"], 7.5, False),
        ("Lon",  f"{ref_lon:.6f}°",  COLORS["text"], 7.5, False),
        ("", None, None, 5, False),
        ("TARGET", None, COLORS["target"], 8, True),
        ("Lat",  f"{meta.get('target_lat', '—'):.6f}°" if meta.get('target_lat') else "—", COLORS["text"], 7.5, False),
        ("Lon",  f"{meta.get('target_lon', '—'):.6f}°" if meta.get('target_lon') else "—", COLORS["text"], 7.5, False),
    ]

    y = 0.97
    for label, value, color, size, bold in lines:
        if color is None:
            y -= 0.025
            continue
        weight = "bold" if bold else "normal"
        if value is None:
            ax_info.text(0.08, y, label,
                         color=color, fontsize=size, fontfamily="monospace",
                         fontweight=weight, va="top", transform=ax_info.transAxes)
        else:
            ax_info.text(0.08, y, f"{label}:",
                         color=COLORS["subtext"], fontsize=size, fontfamily="monospace",
                         va="top", transform=ax_info.transAxes)
            ax_info.text(0.52, y, value,
                         color=color, fontsize=size, fontfamily="monospace",
                         va="top", transform=ax_info.transAxes)
        y -= 0.055

    # Divider line between panels
    for spine in ax_info.spines.values():
        spine.set_edgecolor(COLORS["border"])


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="SAR Flight Path Plotter")
    parser.add_argument("--mission",  default=DEFAULT_MISSION_FILE, help="Path to sar_mission.json")
    parser.add_argument("--interval", type=int, default=DEFAULT_INTERVAL_MS,
                        help="Redraw poll interval in ms (default 1000)")
    args = parser.parse_args()

    matplotlib.rcParams.update({
        "figure.facecolor":  COLORS["bg"],
        "text.color":        COLORS["text"],
        "axes.labelcolor":   COLORS["text"],
        "xtick.color":       COLORS["subtext"],
        "ytick.color":       COLORS["subtext"],
        "font.family":       "monospace",
    })

    fig = plt.figure(figsize=(13, 7), facecolor=COLORS["bg"])
    fig.canvas.manager.set_window_title("SAR — Flight Path Plotter")

    # Layout: 75% map | 25% info panel
    gs = fig.add_gridspec(1, 2, width_ratios=[3, 1], wspace=0.03,
                          left=0.06, right=0.97, top=0.93, bottom=0.08)
    ax_main = fig.add_subplot(gs[0])
    ax_info = fig.add_subplot(gs[1])

    last_mtime = None
    last_mission_data = None

    def update(_frame=None):
        nonlocal last_mtime, last_mission_data

        if not os.path.isfile(args.mission):
            ax_main.clear()
            ax_main.set_facecolor(COLORS["bg"])
            ax_main.text(0.5, 0.5,
                         f"Waiting for\n{args.mission}",
                         ha="center", va="center",
                         color=COLORS["subtext"], fontsize=11,
                         transform=ax_main.transAxes, fontfamily="monospace")
            ax_info.clear()
            ax_info.set_facecolor(COLORS["panel_bg"])
            ax_info.axis("off")
            fig.canvas.draw_idle()
            return

        try:
            mtime = os.path.getmtime(args.mission)
        except OSError:
            return

        if mtime == last_mtime and last_mission_data is not None:
            return  # nothing changed

        try:
            mission = parse_mission(args.mission)
        except Exception as e:
            print(f"[PLOT] Parse error: {e}")
            return

        last_mtime = mtime
        last_mission_data = mission

        draw(ax_main, ax_info, mission)
        fig.canvas.draw_idle()
        print(f"[PLOT] Refreshed — {time.strftime('%H:%M:%S')}")

    # Initial draw
    update()

    # Poll via matplotlib timer (non-blocking, works on all backends)
    timer = fig.canvas.new_timer(interval=args.interval)
    timer.add_callback(update)
    timer.start()

    plt.show()
    print("[PLOT] Window closed.")


if __name__ == "__main__":
    main()