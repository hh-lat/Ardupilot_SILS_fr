#!/usr/bin/env python3
"""
sweep_entry.py — sweep one TECS gain and measure the CLIMB-ENTRY DIP, each value in
its own fresh SITL, flying the SAME AUTO mission as mission_ustol_1.py
(NAV_TAKEOFF -> far/high anchor on the 17.46 deg ray), so the entry transient matches.

For each value it: launches arduplane SITL (--speedup) with the uSTOL params, sets the
base climb params + the swept gain, uploads the 3-item AUTO mission, arms, AUTO,
climbs past STOP_ALT, captures the FDM CSV (matched by SITL pid), kills SITL, and
measures the entry transient:
    gamma_min      : how deep the flight-path angle dips  (negative = brief descent = bad)
    TAS_overshoot  : peak airspeed - target               (how far speed runs past 11)
    gamma_pp_15s   : gamma peak-to-peak in the first 15 s  (size of the swing)
    gamma_settle   : mean gamma over the last 5 s of the window (does it recover to ~17.46?)

    python3 ustol_sims/sweep_entry.py

Output: ustol_sims/plots/sweep_entry_<PARAM>.png  (+ printed table)
"""
import csv, glob, math, os, signal, subprocess, time
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pymavlink import mavutil

# ============================ EDIT THIS ============================ #
SWEEP_PARAM = "TECS_SPDWEIGHT"           # try also: TECS_TIME_CONST, TECS_THR_DAMP, TECS_VERT_ACC
SWEEP_VALS  = [2.0, 1.5, 1.0, 0.5]
FIXED       = {"TECS_TIME_CONST": 5.0, "TECS_THR_DAMP": 0.5, "TECS_INTEG_GAIN": 0.1}
SPEEDUP     = 10                          # SITL sim-speed multiplier
# ================================================================== #

HERE   = os.path.dirname(os.path.abspath(__file__))
WS     = os.path.dirname(HERE)
LOGS   = os.path.join(HERE, "logs")
PLOTS  = os.path.join(HERE, "plots")
BINARY = os.path.join(WS, "build", "sitl", "bin", "arduplane")
PARM   = os.path.join(HERE, "params_ustol.parm")
HOME   = "28.559741,77.11745,237,285"
HOME_LAT, HOME_LON, RWY_HDG = 28.559741, 77.11745, 285.0
CLIMB_AS, FPA_DEG, TKOFF_PITCH, TKOFF_ALT = 11.0, 17.4576031, 8.0, 20.0
CLIMB_TGT_ALT, STOP_ALT = 300.0, 120.0    # anchor far/high; only need the entry, so stop early
CLMB_MAX  = round(CLIMB_AS * math.sin(math.radians(FPA_DEG)), 2)
FAR_DIST  = round((CLIMB_TGT_ALT - TKOFF_ALT) / math.tan(math.radians(FPA_DEG)), 2)
PORT, TARGET_GAMMA = 5760, FPA_DEG

def offset(lat, lon, brg, d):
    b = math.radians(brg)
    return (lat + d*math.cos(b)/111320.0, lon + d*math.sin(b)/(111320.0*math.cos(math.radians(lat))))

def fly_one(value):
    """Launch SITL, fly the AUTO mission with SWEEP_PARAM=value, return the CSV path."""
    env = os.environ.copy(); env["LAT_SIM_LOG_DIR"] = LOGS + "/"; env["DISPLAY"] = ""
    os.makedirs(LOGS, exist_ok=True)
    cmd = [BINARY, "-w", "--model", "plane", "--defaults", PARM,
           "--home", HOME, "--sim-address", "127.0.0.1", "--speedup", str(SPEEDUP), "-I0"]
    proc = subprocess.Popen(cmd, env=env, cwd=WS,
                            stdout=open(os.path.join(LOGS, "sweep_sitl.log"), "w"),
                            stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    print(f"  [{SWEEP_PARAM}={value}] SITL pid {proc.pid}", flush=True)
    try:
        c = None
        for _ in range(40):
            try:
                c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{PORT}", source_system=250)
                if c.recv_match(type="HEARTBEAT", blocking=True, timeout=2):
                    break
            except Exception:
                time.sleep(1.0)
        if c is None:
            raise RuntimeError("no SITL connection")
        c.mav.request_data_stream_send(c.target_system, c.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

        def sp(n, v):
            for _ in range(5):
                c.mav.param_set_send(c.target_system, c.target_component, n.encode(),
                                     float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                a = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
                if a and a.param_id.replace('\x00', '') == n:
                    return
        base = {"ARMING_CHECK": 0, "TKOFF_ROTATE_SPD": 6, "TKOFF_LVL_ALT": 15, "AIRSPEED_MIN": 7,
                "AIRSPEED_CRUISE": CLIMB_AS, "TECS_CLMB_MAX": CLMB_MAX, "PTCH_LIM_MAX_DEG": 18,
                "TECS_PITCH_MAX": 18, "TRIM_THROTTLE": 90}
        for n, v in {**base, **FIXED, SWEEP_PARAM: value}.items():
            sp(n, v)

        # --- upload the SAME 3-item AUTO mission ---
        def mi(seq, c_, p1=0.0, lat=0.0, lon=0.0, alt=0.0, frame=None, cur=0):
            if frame is None:
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            return mavutil.mavlink.MAVLink_mission_item_int_message(
                c.target_system, c.target_component, seq, frame, c_, cur, 1, p1, 0, 0, float('nan'),
                int(lat*1e7), int(lon*1e7), float(alt), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        clb_lat, clb_lon = offset(HOME_LAT, HOME_LON, RWY_HDG, FAR_DIST)
        items = [mi(0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat=HOME_LAT, lon=HOME_LON, alt=0,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL, cur=1),
                 mi(1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p1=TKOFF_PITCH, alt=TKOFF_ALT),
                 mi(2, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat=clb_lat, lon=clb_lon, alt=CLIMB_TGT_ALT)]
        while c.recv_match(type=["MISSION_ACK", "MISSION_REQUEST", "MISSION_REQUEST_INT"], blocking=False):
            pass
        c.mav.mission_clear_all_send(c.target_system, c.target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        c.recv_match(type="MISSION_ACK", blocking=True, timeout=3)
        c.mav.mission_count_send(c.target_system, c.target_component, len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent_last = False; t0 = time.time()
        while time.time()-t0 < 20:
            m = c.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"], blocking=True, timeout=3)
            if not m:
                continue
            if m.get_type() == "MISSION_ACK":
                if sent_last:
                    break
                continue
            c.mav.send(items[m.seq])
            if m.seq == len(items)-1:
                sent_last = True

        st = {"alt": 0.0}
        def pump():
            m = c.recv_match(type=["GLOBAL_POSITION_INT"], blocking=True, timeout=2)
            if m:
                st["alt"] = m.relative_alt/1000.0
        def mode(name):
            mid = c.mode_mapping()[name]
            for _ in range(8):
                c.set_mode(mid); time.sleep(0.3)
                hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                if hb and hb.custom_mode == mid:
                    return
        mode("FBWA")
        t0 = time.time()
        while time.time()-t0 < 30:
            c.mav.command_long_send(c.target_system, c.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
            hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                break
        mode("AUTO")
        t0 = time.time()
        while time.time()-t0 < 120:
            pump()
            if st["alt"] >= STOP_ALT:
                break
        time.sleep(1.0)
        csvs = glob.glob(os.path.join(LOGS, f"sim_output_*_pid{proc.pid}.csv"))
        return max(csvs, key=os.path.getmtime) if csvs else None
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        time.sleep(2.0)

def entry_metrics(path):
    """Return (t_rel, gamma, TAS, gamma_min, tas_overshoot, gamma_pp15, gamma_settle)."""
    r = list(csv.DictReader(open(path)))
    t = np.array([float(x["Time_s"]) for x in r])
    alt = np.array([float(x["alt_agl_m"]) for x in r])
    tas = np.array([float(x["TAS_mps"]) for x in r])
    vN = np.array([float(x["V_ned_gnd_0"]) for x in r])
    vE = np.array([float(x["V_ned_gnd_1"]) for x in r])
    vD = np.array([float(x["V_ned_gnd_2"]) for x in r])
    h = np.hypot(vN, vE)
    gamma = np.degrees(np.arctan2(-vD, np.where(h > 0.5, h, 0.5)))
    above = np.where(alt > 12.0)[0]
    i0 = above[0] if len(above) else 0
    t0 = t[i0]
    win = (t >= t0) & (t <= t0 + 40)             # 40 s window covers the entry transient
    tr, gw, vw = t[win] - t0, gamma[win], tas[win]
    first15 = tr <= 15.0
    last5 = tr >= (tr[-1] - 5.0)
    return (tr, gw, vw,
            gw.min(),                            # gamma_min (dip depth)
            vw.max() - CLIMB_AS,                 # TAS overshoot past 11
            (gw[first15].max() - gw[first15].min()) if first15.any() else 0.0,
            gw[last5].mean() if last5.any() else gw[-1])

# ---- run the sweep ----
print(f"ENTRY-DIP sweep of {SWEEP_PARAM} over {SWEEP_VALS}  (fixed {FIXED}, speedup {SPEEDUP})", flush=True)
print(f"target gamma = {TARGET_GAMMA:.2f} deg\n", flush=True)
results = []
for v in SWEEP_VALS:
    p = fly_one(v)
    if not p:
        print(f"  value {v}: NO CSV (run failed)", flush=True); continue
    tr, gw, vw, gmin, vos, gpp, gset = entry_metrics(p)
    results.append((v, tr, gw, vw, gmin, vos, gpp, gset))
    print(f"  {SWEEP_PARAM}={v}: gamma_min={gmin:6.1f}  TAS_overshoot={vos:5.2f}  "
          f"gamma_pp15s={gpp:5.1f}  gamma_settle={gset:5.1f}", flush=True)

if results:
    print(f"\n{'value':>7s} {'g_min':>7s} {'TAS_os':>7s} {'g_pp15':>7s} {'g_settle':>8s}")
    for v, _, _, _, gmin, vos, gpp, gset in results:
        print(f"{v:7.2f} {gmin:7.1f} {vos:7.2f} {gpp:7.1f} {gset:8.1f}")
    best = max(results, key=lambda r: r[4])      # least-negative gamma_min = shallowest dip
    print(f"\nSHALLOWEST dip: {SWEEP_PARAM}={best[0]}  (gamma_min={best[4]:.1f} deg)")

    fig, ax = plt.subplots(1, 2, figsize=(15, 6))
    for v, tr, gw, vw, gmin, *_ in results:
        ax[0].plot(tr, gw, lw=1.1, label=f"{SWEEP_PARAM}={v} (min {gmin:.1f})")
    ax[0].axhline(TARGET_GAMMA, color="k", ls="--", lw=0.9, label=f"target {TARGET_GAMMA:.1f}")
    ax[0].axhline(0, color="r", ls=":", lw=0.8)
    ax[0].set_title("Flight-path angle through climb entry"); ax[0].set_xlabel("Time since climb start (s)")
    ax[0].set_ylabel("gamma (deg)"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    vs = [r[0] for r in results]
    ax[1].plot(vs, [r[4] for r in results], "o-", label="gamma_min (higher=better)")
    ax[1].plot(vs, [r[5] for r in results], "s--", label="TAS overshoot (lower=better)")
    ax[1].plot(vs, [r[6] for r in results], "^-.", label="gamma p-p 15s (lower=better)")
    ax[1].axhline(0, color="r", ls=":", lw=0.8)
    ax[1].set_title(f"entry dip vs {SWEEP_PARAM}"); ax[1].set_xlabel(SWEEP_PARAM)
    ax[1].grid(alpha=0.3); ax[1].legend()
    os.makedirs(PLOTS, exist_ok=True)
    out = os.path.join(PLOTS, f"sweep_entry_{SWEEP_PARAM}.png")
    fig.tight_layout(); fig.savefig(out, dpi=130); print("saved:", out, flush=True)
