#!/usr/bin/env python3
"""
sweep_thrdamp.py — sweep a TECS gain across values, each in its OWN fresh SITL,
fly the GUIDED 17.46-deg climb, and compare phugoid damping (zeta / decay ratio).

For each value it: launches arduplane SITL (--speedup for speed) with the uSTOL
params, sets the climb params + the swept gain at runtime, flies takeoff->GUIDED
climb, captures the FDM sim_output CSV (matched by SITL pid), kills SITL, and
measures the theta-phugoid decay. Finally it stacks the theta(t) decay curves and
plots zeta vs the swept value.

    python3 ustol_sims/sweep_thrdamp.py

Output: ustol_sims/plots/sweep_<PARAM>.png  (+ printed table)
"""
import csv, glob, math, os, signal, subprocess, sys, time
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pymavlink import mavutil

# ============================ EDIT THIS ============================ #
SWEEP_PARAM = "TECS_THR_DAMP"
SWEEP_VALS  = [0.5, 0.7, 0.9]
FIXED       = {"TECS_INTEG_GAIN": 0.1, "TECS_PTCH_DAMP": 0.6, "TECS_TIME_CONST": 5.0}
SPEEDUP     = 10                      # SITL sim-speed multiplier
# ================================================================== #

HERE   = os.path.dirname(os.path.abspath(__file__))
WS     = os.path.dirname(HERE)
LOGS   = os.path.join(HERE, "logs")
PLOTS  = os.path.join(HERE, "plots")
BINARY = os.path.join(WS, "build", "sitl", "bin", "arduplane")
PARM   = os.path.join(HERE, "params_ustol.parm")
HOME   = "28.559741,77.11745,237,285"
RWY_HDG, CLIMB_AS, FPA_DEG, HANDOFF_ALT = 285.0, 11.0, 17.4576031, 9.5
CLIMB_TGT_ALT, STOP_ALT = 800.0, 500.0
CLMB_MAX = round(CLIMB_AS * math.sin(math.radians(FPA_DEG)), 2)
FAR_DIST = round((CLIMB_TGT_ALT - HANDOFF_ALT) / math.tan(math.radians(FPA_DEG)), 2)
PORT = 5760

def offset(lat, lon, brg, d):
    b = math.radians(brg)
    return (lat + d*math.cos(b)/111320.0, lon + d*math.sin(b)/(111320.0*math.cos(math.radians(lat))))

def fly_one(value):
    """Launch SITL, fly the climb with SWEEP_PARAM=value, return the CSV path."""
    env = os.environ.copy(); env["LAT_SIM_LOG_DIR"] = LOGS + "/"; env["DISPLAY"] = ""
    os.makedirs(LOGS, exist_ok=True)
    cmd = [BINARY, "-w", "--model", "plane", "--defaults", PARM,
           "--home", HOME, "--sim-address", "127.0.0.1", "--speedup", str(SPEEDUP), "-I0"]
    proc = subprocess.Popen(cmd, env=env, cwd=WS, stdout=open(os.path.join(LOGS, "sweep_sitl.log"), "w"),
                            stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    print(f"  [{SWEEP_PARAM}={value}] SITL pid {proc.pid}", flush=True)
    try:
        c = None
        for _ in range(40):
            try:
                c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{PORT}", source_system=250)
                if c.recv_match(type="HEARTBEAT", blocking=True, timeout=2): break
            except Exception: time.sleep(1.0)
        if c is None: raise RuntimeError("no SITL connection")
        c.mav.request_data_stream_send(c.target_system, c.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

        def sp(n, v):
            for _ in range(5):
                c.mav.param_set_send(c.target_system, c.target_component, n.encode(),
                                     float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                a = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
                if a and a.param_id.replace('\x00', '') == n: return
        for n, v in {"ARMING_CHECK": 0, "AIRSPEED_CRUISE": CLIMB_AS, "TECS_CLMB_MAX": CLMB_MAX,
                     "PTCH_LIM_MAX_DEG": 25, "TECS_PITCH_MAX": 25, **FIXED, SWEEP_PARAM: value}.items():
            sp(n, v)

        st = {"alt": 0.0, "lat": None, "lon": None}
        def pump():
            m = c.recv_match(type=["GLOBAL_POSITION_INT"], blocking=True, timeout=2)
            if m: st["alt"], st["lat"], st["lon"] = m.relative_alt/1000.0, m.lat/1e7, m.lon/1e7
        def mode(name):
            mid = c.mode_mapping()[name]
            for _ in range(8):
                c.set_mode(mid); time.sleep(0.3)
                hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                if hb and hb.custom_mode == mid: return
        mode("FBWA")
        t0 = time.time()                              # arm (force after a few tries)
        while time.time()-t0 < 30:
            c.mav.command_long_send(c.target_system, c.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0,0,0,0,0)
            hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): break
        mode("TAKEOFF")
        t0 = time.time()
        while time.time()-t0 < 60:
            pump()
            if st["alt"] >= HANDOFF_ALT: break
        mode("GUIDED"); time.sleep(0.5); pump()
        tlat, tlon = offset(st["lat"], st["lon"], RWY_HDG, FAR_DIST)
        for _ in range(3):
            c.mav.command_int_send(c.target_system, c.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1, 0, 0, float('nan'),
                int(tlat*1e7), int(tlon*1e7), CLIMB_TGT_ALT); time.sleep(0.3)
        t0 = time.time()                              # climb until STOP_ALT (wall timeout generous)
        while time.time()-t0 < 120:
            pump()
            if st["alt"] >= STOP_ALT: break
        time.sleep(1.0)
        csvs = glob.glob(os.path.join(LOGS, f"sim_output_*_pid{proc.pid}.csv"))
        return max(csvs, key=os.path.getmtime) if csvs else None
    finally:
        try: os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception: pass
        time.sleep(2.0)

def damping(path):
    """Return (theta pk-pk, period, decay a3/a1, zeta, t, theta_detrended) over the climb."""
    r = list(csv.DictReader(open(path)))
    t = np.array([float(x["Time_s"]) for x in r]); th = np.array([float(x["theta"])*180/math.pi for x in r])
    alt = np.array([float(x["alt_agl_m"]) for x in r])
    above = np.where(alt > 15)[0]; t0 = t[above[0]] if len(above) else t[0]
    near = np.where(alt > 0.97*alt.max())[0]; t1 = (t[near[0]] if len(near) else t[-1]) - 5
    m = (t >= t0) & (t <= t1); tw, thw = t[m], th[m]
    c = np.polyfit(np.arange(len(thw)), thw, 1); thd = thw - np.polyval(c, np.arange(len(thw)))
    n = len(thd); tu = np.linspace(tw[0], tw[-1], n); su = np.interp(tu, tw, thd)*np.hanning(n)
    fr = np.fft.rfftfreq(n, (tu[-1]-tu[0])/(n-1)); pk = fr[1:][np.argmax(np.abs(np.fft.rfft(su))[1:])]
    k = n//3; a1 = thd[:k].max()-thd[:k].min(); a3 = thd[-k:].max()-thd[-k:].min()
    ncyc = (tw[-1]-tw[0])*2/3*pk; ratio = a3/max(a1, 1e-6)
    zeta = (-math.log(ratio)/(2*math.pi*ncyc)) if 0 < ratio < 1 and ncyc > 0 else 0.0
    return thw.max()-thw.min(), 1/pk, ratio, zeta, tw-tw[0], thd

# ---- run the sweep ----
print(f"sweeping {SWEEP_PARAM} over {SWEEP_VALS}  (fixed {FIXED}, speedup {SPEEDUP})", flush=True)
results = []
for v in SWEEP_VALS:
    csvp = fly_one(v)
    if not csvp: print(f"  value {v}: NO CSV (run failed)"); continue
    thpp, per, ratio, zeta, tt, thd = damping(csvp)
    results.append((v, thpp, per, ratio, zeta, tt, thd))
    print(f"  {SWEEP_PARAM}={v}: theta pk-pk={thpp:.1f}  period={per:.1f}s  decay={ratio:.2f}  zeta={zeta:.3f}", flush=True)

if results:
    print(f"\n{'value':>7s} {'th pk-pk':>9s} {'period':>7s} {'decay':>6s} {'zeta':>6s}")
    for v, thpp, per, ratio, zeta, *_ in results:
        print(f"{v:7.2f} {thpp:9.1f} {per:7.1f} {ratio:6.2f} {zeta:6.3f}")
    best = min(results, key=lambda r: r[3])
    print(f"\nBEST damping: {SWEEP_PARAM}={best[0]}  (decay {best[3]:.2f}, zeta {best[4]:.3f})")

    fig, ax = plt.subplots(1, 2, figsize=(15, 6))
    for v, thpp, per, ratio, zeta, tt, thd in results:
        ax[0].plot(tt, thd, lw=1.0, label=f"{SWEEP_PARAM}={v} (z={zeta:.3f})")
    ax[0].set_title("theta (detrended) — phugoid decay"); ax[0].set_xlabel("Time in climb (s)")
    ax[0].set_ylabel("theta dev (deg)"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    vs = [r[0] for r in results]; zs = [r[4] for r in results]; ds = [r[3] for r in results]
    ax[1].plot(vs, zs, "o-", label="zeta (higher=better)")
    ax[1].plot(vs, ds, "s--", label="decay ratio (lower=better)")
    ax[1].set_title(f"damping vs {SWEEP_PARAM}"); ax[1].set_xlabel(SWEEP_PARAM)
    ax[1].grid(alpha=0.3); ax[1].legend()
    os.makedirs(PLOTS, exist_ok=True)
    out = os.path.join(PLOTS, f"sweep_{SWEEP_PARAM}.png")
    fig.tight_layout(); fig.savefig(out, dpi=130); print("saved:", out)
