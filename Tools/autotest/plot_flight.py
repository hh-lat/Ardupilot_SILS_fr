#!/usr/bin/env python3
"""
plot_flight.py  -  plot the CSV produced by auto_flight2.py --log.

Reads the telemetry CSV and produces stacked, time-aligned plots with the flight
phases shaded in the background, so you can judge stability across the whole
mission profile at a glance.

    python3 ./Tools/autotest/plot_flight.py flight.csv
    python3 ./Tools/autotest/plot_flight.py flight.csv --save flight.png
    python3 ./Tools/autotest/plot_flight.py flight.csv --no-show --save flight.png

Panels (vs time):
    1. altitude (AGL)  +  airspeed  +  ground speed
    2. Euler angles    (roll, pitch, yaw)
    3. body rates      (p, q, r)
    4. flight-path angle (gamma) + climb rate  +  throttle

Requires matplotlib (pip install matplotlib).  Uses only the std-lib csv reader,
so pandas is NOT needed.
"""

import argparse
import csv
import sys

PHASE_COLORS = ["tab:blue", "tab:green", "tab:orange", "tab:red",
                "tab:purple", "tab:brown", "tab:gray", "tab:olive"]


def load_rows(path):
    try:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
    except FileNotFoundError:
        sys.exit("file not found: %s" % path)
    if not rows:
        sys.exit("no data rows in %s" % path)
    return rows


def column(rows, key):
    """Return a column as a list of floats (NaN where missing/blank)."""
    out = []
    for r in rows:
        try:
            out.append(float(r.get(key, "")))
        except (ValueError, TypeError):
            out.append(float("nan"))
    return out


def phase_segments(rows, t):
    """List of (phase, t_start, t_end) for shading and labelling."""
    segments, current, start = [], None, None
    for i, r in enumerate(rows):
        ph = r.get("phase", "")
        if ph != current:
            if current is not None:
                segments.append((current, start, t[i]))
            current, start = ph, t[i]
    if current is not None:
        segments.append((current, start, t[-1]))
    return segments


def shade(ax, segments):
    for i, (_, t0, t1) in enumerate(segments):
        ax.axvspan(t0, t1, color=PHASE_COLORS[i % len(PHASE_COLORS)], alpha=0.07)


def combined_legend(primary, secondary, **kw):
    lines = primary.get_lines() + secondary.get_lines()
    primary.legend(lines, [ln.get_label() for ln in lines], **kw)


def main():
    ap = argparse.ArgumentParser(description="Plot auto_flight2.py CSV telemetry logs.")
    ap.add_argument("csv", nargs="?", default="flight.csv", help="CSV file (default: flight.csv)")
    ap.add_argument("--save", metavar="PNG", default=None, help="save the figure to this file")
    ap.add_argument("--no-show", action="store_true", help="do not open an interactive window")
    args = ap.parse_args()

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib is required:  pip install matplotlib")

    rows = load_rows(args.csv)
    t = column(rows, "time_s")
    segments = phase_segments(rows, t)

    fig, axes = plt.subplots(5, 1, figsize=(13, 13.5), sharex=True)
    fig.suptitle("flight telemetry  -  %s" % args.csv, fontsize=13)

    # --- 1. altitude + speeds --------------------------------------------- #
    ax = axes[0]
    shade(ax, segments)
    ax.plot(t, column(rows, "alt_agl_m"), color="black", label="alt AGL (m)")
    ax.set_ylabel("altitude (m)")
    ax.grid(True, alpha=0.3)
    axspd = ax.twinx()
    axspd.plot(t, column(rows, "airspeed"), color="tab:red", label="airspeed (m/s)")
    axspd.plot(t, column(rows, "groundspeed"), color="tab:orange", ls="--", label="ground speed (m/s)")
    axspd.set_ylabel("speed (m/s)")
    combined_legend(ax, axspd, loc="upper left", fontsize=8)
    # phase labels along the top
    top = ax.get_ylim()[1]
    for ph, t0, t1 in segments:
        ax.text((t0 + t1) / 2.0, top, ph, ha="center", va="bottom", fontsize=7)

    # --- 2. Euler angles -------------------------------------------------- #
    ax = axes[1]
    shade(ax, segments)
    ax.plot(t, column(rows, "roll_deg"), label="roll")
    ax.plot(t, column(rows, "pitch_deg"), label="pitch")
    ax.plot(t, column(rows, "yaw_deg"), label="yaw")
    # commanded attitude (only present in newer logs; NaN columns draw nothing)
    ax.plot(t, column(rows, "navroll_deg"), label="roll cmd", ls="--", alpha=0.6)
    ax.plot(t, column(rows, "navpitch_deg"), label="pitch cmd", ls=":", alpha=0.6)
    ax.set_ylabel("Euler (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # --- 3. body rates ---------------------------------------------------- #
    ax = axes[2]
    shade(ax, segments)
    ax.plot(t, column(rows, "p_dps"), label="p (roll rate)")
    ax.plot(t, column(rows, "q_dps"), label="q (pitch rate)")
    ax.plot(t, column(rows, "r_dps"), label="r (yaw rate)")
    ax.set_ylabel("body rates (deg/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # --- 4. flight-path angle + climb + throttle -------------------------- #
    ax = axes[3]
    shade(ax, segments)
    ax.plot(t, column(rows, "gamma_deg"), color="tab:purple", label="flight-path angle (deg)")
    ax.plot(t, column(rows, "climb_mps"), color="tab:green", ls="--", label="climb (m/s)")
    ax.set_ylabel("gamma (deg) / climb (m/s)")
    ax.grid(True, alpha=0.3)
    axthr = ax.twinx()
    axthr.plot(t, column(rows, "throttle_pct"), color="tab:gray", alpha=0.6, label="throttle (%)")
    axthr.set_ylabel("throttle (%)")
    axthr.set_ylim(0, 100)
    combined_legend(ax, axthr, loc="upper left", fontsize=8)

    # --- 5. angle of attack + sideslip ------------------------------------ #
    ax = axes[4]
    shade(ax, segments)
    ax.plot(t, column(rows, "aoa_deg"), color="tab:red", label="angle of attack alpha (deg)")
    ax.plot(t, column(rows, "ssa_deg"), color="tab:blue", label="sideslip beta (deg)")
    ax.axhline(0, color="k", lw=0.6, alpha=0.4)
    ax.set_ylabel("alpha / beta (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_xlabel("time (s)")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    if args.save:
        fig.savefig(args.save, dpi=120)
        print("saved %s" % args.save)
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
