#!/usr/bin/env python3
"""
plot_ustol.py — plot ONE uSTOL FDM sim_output CSV with the same panels as
Tools/autotest/monte_carlo/plot_case_full.py.

Run standalone (edit CSV_NAME / TMAX below):
    python3 ustol_sims/plot_ustol.py
or call as a function (used by mission_ustol.py):
    import plot_ustol; plot_ustol.plot("")     # "" = newest CSV in ustol_sims/logs/

PNG saves to ustol_sims/plots/case_plots/ (visible, not the hidden logs/ dir).
"""
# ============================ EDIT THIS ============================ #
CSV_NAME = "sim_output_20260613_091025_pid47142.csv"   # "" = newest in ustol_sims/logs/
TMAX     = None                                          # e.g. 120 to clip to first 120 s
# ================================================================== #

import csv, glob, math, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

R2D   = 180.0 / math.pi
HERE  = os.path.dirname(os.path.abspath(__file__))       # ustol_sims/
LOGS  = os.path.join(HERE, "logs")                       # ustol_sims/logs/  (gitignored)
PLOTS = os.path.join(HERE, "plots", "case_plots")        # ustol_sims/plots/case_plots/ (visible)


def plot(csv_name=CSV_NAME, tmax=TMAX):
    # --- locate the CSV ---
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
    if tmax is not None:
        rows = [r for r in rows if float(r["Time_s"]) <= tmax]
    print(f"plotting {len(rows)} rows from {os.path.basename(f)}" + (f"  (t<={tmax}s)" if tmax else ""))

    def C(k, sc=1.0): return [float(r[k]) * sc for r in rows] if k in rows[0] else None
    t = C("Time_s")

    # --- derived quantities ---
    def alpha_row(r):
        u, w = float(r["V_b_tas_0"]), float(r["V_b_tas_2"])
        return math.atan2(w, u) * R2D if math.hypot(u, w) > 1.0 else 0.0
    def gamma_row(r):
        vN, vE, vD = float(r["V_ned_gnd_0"]), float(r["V_ned_gnd_1"]), float(r["V_ned_gnd_2"])
        h = math.hypot(vN, vE)
        return math.atan2(-vD, h) * R2D if (h > 0.5 or abs(vD) > 0.5) else 0.0
    alpha    = [alpha_row(r) for r in rows]
    gamma    = [gamma_row(r) for r in rows]

    def read_mass(csv_path):
        ov = os.path.join(os.path.dirname(csv_path), "overrides.txt")
        if os.path.isfile(ov):
            for line in open(ov):
                if line.startswith("mass="): return float(line.split("=", 1)[1])
        return 17.0
    mass = read_mass(f); g = 9.81; W = mass * g
    liftN = C("Lift_N")
    LWcos = ([liftN[i] / (W * math.cos(math.radians(gamma[i]))) for i in range(len(rows))]
             if liftN else None)

    # From the dataflash *.BIN under <ws>/logs/:
    #   cmd_rates : commanded body rates   (PIDR/PIDP/PIDY .Tar)
    #   desired   : commanded pitch (ATT.DesPitch), FPA (asin(TECS.dhdem/spdem)) and
    #               alpha (= des pitch - des FPA), to overlay vs the actual states.
    cmd_rates = None
    desired = None
    try:
        import numpy as np
        from pymavlink import mavutil
        ws = os.path.dirname(HERE)
        bins = glob.glob(os.path.join(ws, "logs", "*.BIN"))
        if bins:
            mb = mavutil.mavlink_connection(max(bins, key=os.path.getmtime))
            d = {"PIDR": [[], []], "PIDP": [[], []], "PIDY": [[], []]}
            att = [[], []]                       # t, DesPitch (deg)
            tec = [[], [], []]                   # t, spdem, dhdem
            while True:
                msg = mb.recv_match(type=["PIDR", "PIDP", "PIDY", "ATT", "TECS"], blocking=False)
                if msg is None: break
                ty = msg.get_type()
                if ty in d:
                    dd = d[ty]; dd[0].append(msg.TimeUS / 1e6); dd[1].append(msg.Tar)
                elif ty == "ATT" and hasattr(msg, "DesPitch"):
                    att[0].append(msg.TimeUS / 1e6); att[1].append(msg.DesPitch)
                elif ty == "TECS":
                    sp, dh = getattr(msg, "spdem", None), getattr(msg, "dhdem", None)
                    if sp is not None and dh is not None:
                        tec[0].append(msg.TimeUS / 1e6); tec[1].append(sp); tec[2].append(dh)
            # The dataflash BIN and the FDM CSV share the SAME sim clock (verified: BIN's
            # logged actual pitch at raw TimeUS matches CSV theta at the same Time_s), so
            # use raw BIN time directly — do NOT zero-shift, or the overlays slide left.
            def clip(xx, yy):                    # apply tmax window to a (t,y) pair
                if tmax is None: return xx, yy
                m = xx <= tmax; return xx[m], yy[m]

            if d["PIDP"][0]:
                cmd_rates = {}
                for key, typ in [("roll", "PIDR"), ("pitch", "PIDP"), ("yaw", "PIDY")]:
                    tt = list(d[typ][0]); yy = d[typ][1]
                    if tmax is not None:
                        keep = [i for i, x in enumerate(tt) if x <= tmax]
                        tt = [tt[i] for i in keep]; yy = [yy[i] for i in keep]
                    cmd_rates[key] = (tt, yy)
                print(f"commanded rates overlaid from {len(d['PIDP'][0])} PID samples")

            if att[0] or tec[0]:
                desired = {}
                ta = ga_des = None
                if att[0]:
                    ta = np.array(att[0]); th_des = np.array(att[1])
                    desired["theta"] = clip(ta, th_des)
                if tec[0]:
                    tt = np.array(tec[0])
                    sp = np.array(tec[1]); dh = np.array(tec[2])
                    ratio = np.clip(np.where(sp > 0.5, dh / np.where(sp > 0.5, sp, 1.0), 0.0), -1, 1)
                    ga_des = np.degrees(np.arcsin(ratio))
                    desired["gamma"] = clip(tt, ga_des)
                if att[0] and tec[0]:            # alpha_des = pitch_des - FPA_des (FPA interp onto ATT t)
                    desired["alpha"] = clip(ta, th_des - np.interp(ta, tt, ga_des))
                print(f"desired overlay: pitch={'Y' if att[0] else 'n'} "
                      f"FPA/alpha={'Y' if tec[0] else 'n'}")
    except Exception as e:
        print("commanded/desired overlay skipped:", e)

    fig, ax = plt.subplots(6, 3, figsize=(17, 19))
    def P(a, series, title, ylab):
        for lab, y, in series:
            if y is not None: a.plot(t, y, lw=1.0, label=lab)
        a.set_title(title, fontsize=10); a.set_ylabel(ylab); a.grid(alpha=0.3)
        a.legend(fontsize=7, loc="best"); a.set_xlabel("Time (s)")

    P(ax[0,0], [("alt AGL (m)", C("alt_agl_m"))], "Altitude", "m")
    P(ax[0,1], [("TAS (m/s)", C("TAS_mps"))], "True airspeed", "m/s")
    P(ax[0,2], [("phi", C("phi", R2D)), ("theta", C("theta", R2D)), ("psi", C("psi", R2D))], "Euler angles", "deg")
    _a = ax[1,0]
    for i, (lab, y) in enumerate([("p (roll)", C("p", R2D)), ("q (pitch)", C("q", R2D)), ("r (yaw)", C("r", R2D))]):
        if y is not None: _a.plot(t, y, color=f"C{i}", lw=1.0, label=lab + " act")
    if cmd_rates:
        for i, key in enumerate(["roll", "pitch", "yaw"]):
            tt, yy = cmd_rates[key]
            _a.plot(tt, yy, color=f"C{i}", lw=1.0, ls="--", label=key + " cmd")
    _a.set_title("Body rates (solid=actual, dashed=commanded)", fontsize=10)
    _a.set_ylabel("deg/s"); _a.grid(alpha=0.3); _a.legend(fontsize=6, loc="best"); _a.set_xlabel("Time (s)")
    P(ax[1,1], [("vN", C("V_ned_gnd_0")), ("vE", C("V_ned_gnd_1")), ("vD", C("V_ned_gnd_2"))], "NED ground velocity", "m/s")
    P(ax[1,2], [("u_b", C("V_b_tas_0")), ("v_b", C("V_b_tas_1")), ("w_b", C("V_b_tas_2"))], "Body airspeed comps", "m/s")
    P(ax[2,0], [("elev", C("delta_e", R2D)), ("ailL", C("delta_aL", R2D)), ("ailR", C("delta_aR", R2D)),
                ("rud", C("delta_r", R2D)), ("flap", C("delta_f", R2D))], "Control-surface deflections", "deg")
    P(ax[2,1], [("thrust (N) — 18 EDFs", C("total_rotor_force")), ("mot0 cmd (1/18, NOT fleet)", C("mot0_thr_cmd", 100.0))], "Propulsion (use thrust; mot0 is one EDF)", "N / %")
    P(ax[2,2], [("Lift (N)", C("Lift_N")), ("Drag (N)", C("Drag_N")), ("Side (N)", C("Side_N"))], "Aero forces", "N")
    P(ax[3,0], [("p_dot", C("p_dot", R2D)), ("q_dot", C("q_dot", R2D)), ("r_dot", C("r_dot", R2D))], "Angular accel", "deg/s^2")
    P(ax[3,1], [("ax_b", C("Accel_b_0")), ("ay_b", C("Accel_b_1")), ("az_b", C("Accel_b_2"))], "Body accel", "m/s^2")
    P(ax[3,2], [("CL", C("Lift_Coeff")), ("CD", C("Drag_Coeff")), ("Cm", C("Moment_Coeff"))], "Aero coefficients", "-")
    _c = ax[4,0]                                 # actual (solid) vs desired (dashed)
    for i, (lab, y) in enumerate([("alpha", alpha), ("gamma", gamma), ("theta", C("theta", R2D))]):
        if y is not None: _c.plot(t, y, color=f"C{i}", lw=1.0, label=lab + " act")
    if desired:
        for i, key in enumerate(["alpha", "gamma", "theta"]):
            if key in desired:
                xx, yy = desired[key]; _c.plot(xx, yy, color=f"C{i}", lw=1.0, ls="--", label=key + " des")
    _c.set_title("AoA, FPA, pitch (solid=actual, dashed=desired)", fontsize=10)
    _c.set_ylabel("deg"); _c.grid(alpha=0.3); _c.legend(fontsize=6, loc="best"); _c.set_xlabel("Time (s)")
    P(ax[4,1], [("T/W (thrust/W)", [v/(mass*9.81) for v in C("total_rotor_force")])], "Thrust-to-weight (all 18 EDFs)", "-")
    P(ax[4,2], [("MLG_NR", C("MLG_NR")), ("FLG_NR", C("FLG_NR"))], "Gear normal reaction", "N")
    P(ax[5,0], [("L/(W cos g)", LWcos)], f"Lift / (W cos gamma)   (m={mass:.1f} kg, W={W:.0f} N)", "-")
    ax[5,0].axhline(1.0, color='k', ls='--', lw=0.9)
    ax[5,1].axis('off'); ax[5,2].axis('off')

    fig.suptitle(f"uSTOL case telemetry — {os.path.basename(f)}", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    suffix = f"_t{int(tmax)}" if tmax else ""
    os.makedirs(PLOTS, exist_ok=True)
    out = os.path.join(PLOTS, "case_full_" + os.path.splitext(os.path.basename(f))[0] + suffix + ".png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("saved:", out)
    return out


if __name__ == "__main__":
    plot()
