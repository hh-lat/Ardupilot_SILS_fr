#!/usr/bin/env python3
"""
fft_pitch.py — full-picture oscillation diagnosis for a uSTOL sim_output CSV.

Time series (pitch / energy / vertical / aero groups) + a metrics table
(detrended pk-pk + dominant frequency per signal) + a normalised FFT overlay
(shared peak = one coupled mode, e.g. a phugoid in theta/airspeed/altitude/lift).

Judge the oscillation by theta / airspeed / altitude, NOT q (q = dtheta/dt
suppresses low-frequency phugoid content and can look "fine" while theta swings).

Run standalone (edit CSV_NAME / WIN below):
    python3 ustol_sims/fft_pitch.py
or call as a function (used by mission_ustol.py):
    import fft_pitch; fft_pitch.analyze("")     # "" = newest CSV in ustol_sims/logs/

PNG saves to ustol_sims/plots/fft/ (visible, not the hidden logs/ dir).
"""
# ============================ EDIT THIS ============================ #
CSV_NAME = "sim_output_20260613_103104_pid60681.csv"   # "" = newest in ustol_sims/logs/
WIN      = (None, None)                                  # (t0,t1) s, or auto-detect climb
FMAX_HZ  = 2.0                                           # x-limit of the FFT plot
# ================================================================== #

import csv, glob, math, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

R2D   = 180.0 / math.pi
HERE  = os.path.dirname(os.path.abspath(__file__))
LOGS  = os.path.join(HERE, "logs")
PLOTS = os.path.join(HERE, "plots", "fft")               # ustol_sims/plots/fft/ (visible)


def analyze(csv_name=CSV_NAME, win=WIN, fmax_hz=FMAX_HZ):
    # --- locate CSV ---
    if csv_name:
        f = csv_name if os.path.isabs(csv_name) else os.path.join(LOGS, csv_name)
        if not os.path.isfile(f):
            raise SystemExit(f"CSV not found: {f}")
    else:
        c = glob.glob(os.path.join(LOGS, "sim_output_*.csv"))
        if not c:
            raise SystemExit(f"no sim_output_*.csv in {LOGS}")
        f = max(c, key=os.path.getmtime)

    rows = list(csv.DictReader(open(f)))
    def col(k, sc=1.0):
        return np.array([float(r[k]) for r in rows]) * sc if k in rows[0] else None
    t   = col("Time_s")
    alt = col("alt_agl_m")

    u_b, w_b = col("V_b_tas_0"), col("V_b_tas_2")
    vN, vE, vD = col("V_ned_gnd_0"), col("V_ned_gnd_1"), col("V_ned_gnd_2")
    alpha = np.degrees(np.arctan2(w_b, u_b))
    gamma = np.degrees(np.arctan2(-vD, np.hypot(vN, vE)))
    climb = -vD

    SIG = {
        "theta":    (col("theta", R2D),          "deg",   "C2"),
        "q":        (col("q", R2D),               "deg/s", "C3"),
        "elevator": (col("delta_e", R2D),         "deg",   "C4"),
        "TAS":      (col("TAS_mps"),              "m/s",   "C0"),
        "thrust":   (col("total_rotor_force"),    "N",     "C5"),  # fleet thrust (all 18 EDFs)
        "alt":      (alt,                         "m",     "C6"),
        "Lift":     (col("Lift_N"),               "N",     "C8"),
        "Drag":     (col("Drag_N"),               "N",     "C9"),
        "climb":    (climb,                       "m/s",   "C7"),
        "alpha":    (alpha,                       "deg",   "tab:brown"),
        "gamma":    (gamma,                       "deg",   "tab:pink"),
    }
    SIG = {k: v for k, v in SIG.items() if v[0] is not None}

    # --- window ---
    if win[0] is not None and win[1] is not None:
        t0, t1 = win
    else:
        above = np.where(alt > 10.0)[0]
        t0 = t[above[0]] if len(above) else t[0]
        near = np.where(alt > 0.97 * alt.max())[0]
        t1 = t[near[0]] if len(near) else t[-1]
    m  = (t >= t0) & (t <= t1)
    tw = t[m]
    print(f"file {os.path.basename(f)}  window t={t0:.0f}..{t1:.0f}s  ({m.sum()} samples)\n")

    def detrend(y):
        n = len(y); c = np.polyfit(np.arange(n), y, 1); return y - np.polyval(c, np.arange(n))
    def spectrum(y):
        n = len(y); tu = np.linspace(tw[0], tw[-1], n); su = np.interp(tu, tw, y)
        su = detrend(su) * np.hanning(n)      # linear detrend: a climb ramp must not fake a low-freq peak
        fr = np.fft.rfftfreq(n, (tu[-1] - tu[0]) / (n - 1))
        return fr, np.abs(np.fft.rfft(su))

    print(f"{'signal':9s} {'pk-pk(detr)':>12s} {'unit':>5s} {'dom.freq':>10s} {'period':>8s}")
    print("-" * 50)
    metrics = {}
    for k, (arr, unit, _) in SIG.items():
        yw = arr[m]; yd = detrend(yw); fr, mg = spectrum(yw)
        pk = fr[1:][np.argmax(mg[1:])] if len(fr) > 2 else 0.0
        metrics[k] = (yd.max() - yd.min(), pk)
        per = (1.0 / pk) if pk > 0 else float("inf")
        print(f"{k:9s} {yd.max()-yd.min():12.2f} {unit:>5s} {pk:9.3f}Hz {per:7.1f}s")
    theta_pk = metrics["theta"][1]
    print(f"\nHEADLINE: theta pk-pk={metrics['theta'][0]:.1f} deg, "
          f"TAS pk-pk={metrics.get('TAS',(0,0))[0]:.2f} m/s, "
          f"phugoid ~{1/theta_pk:.1f}s ({theta_pk:.3f} Hz)")

    fig, ax = plt.subplots(5, 1, figsize=(13, 17))
    def tline(a, keys, twin_keys=None):
        for k in keys:
            arr, unit, c = SIG[k]; a.plot(tw, arr[m], c, lw=1.0, label=f"{k} ({unit})")
        a.grid(alpha=0.3); a.legend(fontsize=8, loc="upper left"); a.set_xlabel("Time (s)")
        if twin_keys:
            at = a.twinx()
            for k in twin_keys:
                arr, unit, c = SIG[k]; at.plot(tw, arr[m], c, lw=1.0, ls="--", label=f"{k} ({unit})")
            at.legend(fontsize=8, loc="upper right")

    tline(ax[0], ["theta", "q", "elevator"]);             ax[0].set_title("Pitch group  (theta, q, elevator)")
    tline(ax[1], ["TAS"], ["thrust"]);                    ax[1].set_title("Energy group  (airspeed | thrust N dashed — all 18 EDFs)")
    tline(ax[2], ["alt", "climb"], ["Lift", "Drag"]);     ax[2].set_title("Vertical group  (alt, climb | Lift, Drag N dashed)")
    tline(ax[3], ["alpha", "gamma", "theta"]);            ax[3].set_title("Aero group  (AoA, FPA, pitch)")

    # Overlay only the kinematic mode variables that share the fundamental
    # (alt is a ramp; Lift rings at 2x via V^2; throttle is the forcing -> read those as time traces).
    for k in ["theta", "TAS", "gamma", "q"]:
        if k not in SIG: continue
        fr, mg = spectrum(SIG[k][0][m]); mg = mg / (mg[1:].max() or 1.0)
        ax[4].plot(fr, mg, SIG[k][2], lw=1.1, label=k)
    ax[4].axvline(theta_pk, color="k", ls="--", lw=0.9, label=f"theta peak {theta_pk:.3f}Hz ({1/theta_pk:.1f}s)")
    ax[4].set_xlim(0, fmax_hz); ax[4].set_xlabel("Frequency (Hz)"); ax[4].set_ylabel("normalised")
    ax[4].grid(alpha=0.3); ax[4].legend(fontsize=8, loc="best")
    ax[4].set_title("Normalised FFT overlay — kinematic mode variables (theta, TAS, gamma, q)")

    fig.suptitle(f"Full longitudinal picture — {os.path.basename(f)}", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.99])
    os.makedirs(PLOTS, exist_ok=True)
    out = os.path.join(PLOTS, "fft_pitch_" + os.path.splitext(os.path.basename(f))[0] + ".png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("\nsaved:", out)
    return out


if __name__ == "__main__":
    analyze()
