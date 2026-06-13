#!/usr/bin/env python3
"""
plot_case_full.py [sim_output.csv]

Comprehensive plot of one LAT FDM sim_output CSV: states, control inputs,
forces, moments/angular-accel, and aero coefficients. Saves a multi-panel PNG
next to the CSV. If no path given, uses the newest sim_output under
Logs_Simulations/ (the live/most-recent run).
"""
import csv, glob, math, os, sys
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

R2D = 180.0 / math.pi
HERE = os.path.dirname(os.path.abspath(__file__))
WS   = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

def find_csv(arg):
    if arg and os.path.isfile(arg): return arg
    if arg and os.path.isdir(arg):
        c = glob.glob(os.path.join(arg, "**", "sim_output_*.csv"), recursive=True)
    else:
        c = glob.glob(os.path.join(WS, "Logs_Simulations", "sim_output_*.csv"))
    if not c: sys.exit("no sim_output CSV found")
    return max(c, key=os.path.getmtime)

args = sys.argv[1:]
tmax = None
if "--tmax" in args:
    i = args.index("--tmax"); tmax = float(args[i+1]); del args[i:i+2]
f = find_csv(args[0] if args else None)
rows = list(csv.DictReader(open(f)))
if tmax is not None:
    rows = [r for r in rows if float(r["Time_s"]) <= tmax]
print(f"plotting {len(rows)} rows from {os.path.basename(f)}" + (f"  (t<={tmax}s)" if tmax else ""))
def C(k, sc=1.0): return [float(r[k]) * sc for r in rows] if k in rows[0] else None
t = C("Time_s")

# --- derived quantities ---
def alpha_row(r):   # angle of attack = atan2(w_b, u_b)
    u, w = float(r["V_b_tas_0"]), float(r["V_b_tas_2"])
    return math.atan2(w, u) * R2D if math.hypot(u, w) > 1.0 else 0.0
def gamma_row(r):   # flight-path angle = atan2(-vD, |V_horiz|)
    vN, vE, vD = float(r["V_ned_gnd_0"]), float(r["V_ned_gnd_1"]), float(r["V_ned_gnd_2"])
    h = math.hypot(vN, vE)
    return math.atan2(-vD, h) * R2D if (h > 0.5 or abs(vD) > 0.5) else 0.0
alpha    = [alpha_row(r) for r in rows]
gamma    = [gamma_row(r) for r in rows]
throttle = C("mot0_thr_cmd", 100.0)     # 0-1 -> %

# Lift / (W cos gamma): lift sufficiency along the flight path (1 = balanced).
# Uses this case's perturbed mass (from overrides.txt next to the CSV).
def read_mass(csv_path):
    ov = os.path.join(os.path.dirname(csv_path), "overrides.txt")
    if os.path.isfile(ov):
        for line in open(ov):
            if line.startswith("mass="): return float(line.split("=",1)[1])
    return 17.0
mass = read_mass(f); g = 9.81; W = mass * g
liftN = C("Lift_N")
LWcos = ([liftN[i] / (W * math.cos(math.radians(gamma[i]))) for i in range(len(rows))]
         if liftN else None)

# Commanded body rates from the dataflash (PIDR/PIDP/PIDY .Tar) if a .BIN exists
# in this run's tree. ArduPlane logs the rate-loop target there (deg/s).
cmd_rates = None
try:
    from pymavlink import mavutil
    run_root = os.path.dirname(os.path.dirname(f))
    bins = glob.glob(os.path.join(run_root, "**", "*.BIN"), recursive=True)
    if bins:
        mb = mavutil.mavlink_connection(max(bins, key=os.path.getmtime))
        d = {"PIDR": [[], []], "PIDP": [[], []], "PIDY": [[], []]}
        while True:
            msg = mb.recv_match(type=["PIDR", "PIDP", "PIDY"], blocking=False)
            if msg is None: break
            dd = d[msg.get_type()]; dd[0].append(msg.TimeUS / 1e6); dd[1].append(msg.Tar)
        if d["PIDP"][0]:
            bt0 = min(dd[0][0] for dd in d.values() if dd[0])
            cmd_rates = {}
            for key, typ in [("roll", "PIDR"), ("pitch", "PIDP"), ("yaw", "PIDY")]:
                tt = [x - bt0 for x in d[typ][0]]; yy = d[typ][1]
                if tmax is not None:
                    keep = [i for i, x in enumerate(tt) if x <= tmax]
                    tt = [tt[i] for i in keep]; yy = [yy[i] for i in keep]
                cmd_rates[key] = (tt, yy)
            print(f"commanded rates overlaid from {len(d['PIDP'][0])} PID samples")
except Exception as e:
    print("commanded-rate overlay skipped:", e)

fig, ax = plt.subplots(6, 3, figsize=(17, 19))
def P(a, series, title, ylab):
    for lab, y, in series:
        if y is not None: a.plot(t, y, lw=1.0, label=lab)
    a.set_title(title, fontsize=10); a.set_ylabel(ylab); a.grid(alpha=0.3)
    a.legend(fontsize=7, loc="best"); a.set_xlabel("Time (s)")

# --- STATES ---
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
# --- CONTROL INPUTS ---
P(ax[2,0], [("elev", C("delta_e", R2D)), ("ailL", C("delta_aL", R2D)), ("ailR", C("delta_aR", R2D)),
            ("rud", C("delta_r", R2D)), ("flap", C("delta_f", R2D))], "Control-surface deflections", "deg")
P(ax[2,1], [("thrust (N)", C("total_rotor_force")), ("mot0 thr cmd", C("mot0_thr_cmd"))], "Propulsion", "N / pwm")
# --- FORCES ---
P(ax[2,2], [("Lift (N)", C("Lift_N")), ("Drag (N)", C("Drag_N")), ("Side (N)", C("Side_N"))], "Aero forces", "N")
# --- MOMENTS / ANGULAR ACCEL ---
P(ax[3,0], [("p_dot", C("p_dot", R2D)), ("q_dot", C("q_dot", R2D)), ("r_dot", C("r_dot", R2D))], "Angular accel", "deg/s^2")
P(ax[3,1], [("ax_b", C("Accel_b_0")), ("ay_b", C("Accel_b_1")), ("az_b", C("Accel_b_2"))], "Body accel", "m/s^2")
# --- COEFFICIENTS ---
P(ax[3,2], [("CL", C("Lift_Coeff")), ("CD", C("Drag_Coeff")), ("Cm", C("Moment_Coeff"))], "Aero coefficients", "-")
# --- AoA / FPA / THROTTLE ---
P(ax[4,0], [("AoA alpha", alpha), ("FPA gamma", gamma), ("pitch theta", C("theta", R2D))], "AoA, flight-path angle, pitch", "deg")
P(ax[4,1], [("throttle (%)", throttle)], "Throttle command", "%")
P(ax[4,2], [("MLG_NR", C("MLG_NR")), ("FLG_NR", C("FLG_NR"))], "Gear normal reaction", "N")
# --- LIFT SUFFICIENCY ---
P(ax[5,0], [("L/(W cos g)", LWcos)], f"Lift / (W cos gamma)   (m={mass:.1f} kg, W={W:.0f} N)", "-")
ax[5,0].axhline(1.0, color='k', ls='--', lw=0.9)   # =1: lift balances weight component
ax[5,1].axis('off'); ax[5,2].axis('off')

fig.suptitle(f"Full case telemetry — {os.path.basename(f)}", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.98])
suffix = f"_t{int(tmax)}" if tmax else ""
out = os.path.join(os.path.dirname(f), "case_full_" + os.path.splitext(os.path.basename(f))[0] + suffix + ".png")
fig.savefig(out, dpi=130)
print("saved:", out)
