#!/usr/bin/env python3
"""
plot_forces.py — plot Thrust / Lift / Drag vs time from a LAT SITL sim_output CSV.

The C++ Output.cpp logger writes (per run) sim_output_<ts>_pid<N>.csv with:
    Time_s, ..., total_rotor_force (Thrust, N), Lift_N, Drag_N, Side_N,
    Lift_Coeff (CL), Drag_Coeff (CD), Moment_Coeff (Cm)

Usage:
    python3 plot_forces.py <sim_output.csv | case_dir | mc_results_dir>
    python3 plot_forces.py                 # auto-pick newest sim_output under mc_results_*
    python3 plot_forces.py <path> --coeffs # also plot CL/CD/Cm

Saves a PNG next to the CSV (forces_<csvname>.png) — no display needed (WSL/headless).
"""
import argparse
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def find_csv(arg):
    """Resolve a CSV path from a file, a case dir, an mc_results dir, or auto."""
    if arg and os.path.isfile(arg):
        return arg
    search_roots = []
    if arg and os.path.isdir(arg):
        search_roots = [arg, os.path.join(arg, "*")]
    else:
        search_roots = [os.path.join(SCRIPT_DIR, "mc_results_*", "*")]
    cands = []
    for root in search_roots:
        cands += glob.glob(os.path.join(root, "sim_output_*.csv"))
        cands += glob.glob(os.path.join(root, "**", "sim_output_*.csv"), recursive=True)
    if not cands:
        sys.exit(f"No sim_output_*.csv found for '{arg or '(auto)'}'.")
    return max(set(cands), key=os.path.getmtime)


def load(csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{csv_path} is empty.")
    cols = rows[0].keys()
    for needed in ("Time_s", "total_rotor_force"):
        if needed not in cols:
            sys.exit(f"{csv_path} missing column '{needed}'.")
    have_forces = "Lift_N" in cols and "Drag_N" in cols
    if not have_forces:
        print("WARNING: this CSV predates the Lift_N/Drag_N columns — only Thrust "
              "will be plotted. Re-run a case after rebuilding to get Lift/Drag.")
    def col(name):
        return [float(r[name]) for r in rows] if name in cols else None
    return {
        "t": col("Time_s"),
        "thrust": col("total_rotor_force"),
        "lift": col("Lift_N"),
        "drag": col("Drag_N"),
        "CL": col("Lift_Coeff"),
        "CD": col("Drag_Coeff"),
        "Cm": col("Moment_Coeff"),
    }


def main():
    ap = argparse.ArgumentParser(description="Plot Thrust/Lift/Drag vs time.")
    ap.add_argument("path", nargs="?", default=None,
                    help="sim_output CSV, a case dir, or mc_results dir (default: newest)")
    ap.add_argument("--coeffs", action="store_true", help="also plot CL/CD/Cm")
    args = ap.parse_args()

    csv_path = find_csv(args.path)
    print(f"Plotting: {csv_path}")
    d = load(csv_path)

    nplots = 2 if args.coeffs else 1
    fig, axes = plt.subplots(nplots, 1, figsize=(11, 4.5 * nplots), squeeze=False)

    ax = axes[0][0]
    ax.plot(d["t"], d["thrust"], label="Thrust (N)", color="tab:green", lw=1.3)
    if d["lift"] is not None:
        ax.plot(d["t"], d["lift"], label="Lift (N)", color="tab:blue", lw=1.3)
    if d["drag"] is not None:
        ax.plot(d["t"], d["drag"], label="Drag (N)", color="tab:red", lw=1.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.set_title(f"Thrust / Lift / Drag vs time\n{os.path.basename(csv_path)}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    if args.coeffs:
        ax2 = axes[1][0]
        for key, lab, c in (("CL", "C_L", "tab:blue"),
                            ("CD", "C_D", "tab:red"),
                            ("Cm", "C_m", "tab:purple")):
            if d[key] is not None:
                ax2.plot(d["t"], d[key], label=lab, color=c, lw=1.2)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Coefficient")
        ax2.set_title("Aero coefficients vs time")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="best")

    fig.tight_layout()
    out = os.path.join(os.path.dirname(csv_path),
                       "forces_" + os.path.splitext(os.path.basename(csv_path))[0] + ".png")
    fig.savefig(out, dpi=150)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
