#!/usr/bin/env python3
"""
Monte Carlo Runner — uSTOL ALL-FBWA PROFILE w/ LANDING, NO LOITER (mission_ustol_7)
==================================================================================
Each case launches one headless SITL `arduplane`, perturbs the custom uSTOL
FDM parameters via the LAT_MC_OVERRIDE_FILE mechanism, then flies the EXACT
profile from ustol_sims/mission_ustol_7.py — takeoff/climb, cruise, then a
decoupled FBWA approach + POWERED flare landing.  The aircraft stays in FBWA the
WHOLE flight — NO nav mode, NO loiter (this is the only difference vs the
mission_ustol_8 runner, aws_monte_carlo_3.py):

    GROUND    nose-down hold during ground roll (V < V_R)
    CLIMB     airspeed-scheduled, rate-limited pitch with airspeed/AoA guards
    (level)   smoothstep pitch -> 0 + throttle ramp to cruise-trim over the
              last LEVEL_BAND metres up to CRUISE_ALT (arrive LEVEL)
    CRUISE    open-loop level hold (speed-on-pitch + cruise throttle) for
              CRUISE_HOLD_T seconds, then stream straight into the approach
    APPROACH  still FBWA; PITCH holds the approach speed (speed-on-pitch),
              THROTTLE holds the sink rate (sink-on-throttle) down to FLARE_ALT
    FLARE     POWERED flare — THROTTLE arrests the sink around a height-scheduled
              sink target, elevator holds only a modest nose-up, to touchdown
    ROLLOUT   idle throttle + slight nose-down, then DISARM.  Run ends.

WHY a POWERED flare: on this airframe tail/elevator authority scales with dynamic
pressure (~V^2), so at the ~12 m/s approach the tail has ~1/7 of its climb-speed
authority; the thrust line sits below the CG with propwash over the tail, so
THROTTLE trims pitch.  A power-off flare saturates the elevator yet the nose still
falls — so power is held through the flare and sink is arrested with it.

SELF-CONTAINED: the flight logic is implemented here with raw pymavlink (RC
overrides), exactly like monte_carlo_runner_flap_retract.py — there is NO
dependency on auto_flight4.  This is headless (no MAVProxy / GCS viewer): the
MAVLink TCP connection exists only to command each worker's own SITL instance.

This file owns the Monte Carlo concerns:
  • sampling the FDM params from N(nominal, sigma_3/3) clipped to ±3σ,
  • writing the per-case override file the FDM reads at startup,
  • launching N headless SITL processes in parallel (one TCP port each),
  • optional --speedup (run SITL faster than real time — the big throughput win),
  • collecting per-case telemetry + a campaign summary.csv.

PARALLELISM: `--workers` independent SITL processes at once, one instance ID
each (MAVLink TCP port = 5760 + 10*instance_id).  No MAVProxy is used.

Usage:
    python3 aws_monte_carlo_4.py --runs 10 --workers 10
    python3 aws_monte_carlo_4.py --runs 500 --workers 30 --speedup 10
"""

import argparse
import csv
import glob
import json
import math
import multiprocessing
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path

import numpy as np

try:
    from pymavlink import mavutil
except ImportError:                                       # pragma: no cover
    print("ERROR: pymavlink is required.  Install with:  pip install pymavlink")
    sys.exit(1)

# ---------------------------------------------------------------------------
#  Paths (derived from this script's location: Tools/autotest/monte_carlo/)
# ---------------------------------------------------------------------------
SCRIPT_DIR    = Path(__file__).resolve().parent           # Tools/autotest/monte_carlo/
AUTOTEST_DIR  = SCRIPT_DIR.parent                         # Tools/autotest/
WORKSPACE     = AUTOTEST_DIR.parent.parent                # repo root
USTOL_SIMS    = WORKSPACE / "ustol_sims"                  # tuned uSTOL params live here
BINARY        = WORKSPACE / "build" / "sitl" / "bin" / "arduplane"
DEFAULT_CFG   = SCRIPT_DIR / "monte_carlo_config_ustol_v1.json"
DEFAULTS_PARM = USTOL_SIMS / "params_imp_v3.param"        # uSTOL tuned params (v3)
FALLBACK_PARM = AUTOTEST_DIR / "models" / "plane.parm"


# ===================================================================
#  Configuration helpers
# ===================================================================
def load_config(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def resolve_defaults_parm() -> str:
    """Return path to the ArduPilot parameter defaults file (tuned uSTOL params)."""
    for cand in (DEFAULTS_PARM, USTOL_SIMS / "params_ustol.parm", FALLBACK_PARM):
        if cand.exists():
            return str(cand)
    raise FileNotFoundError("Cannot find params_imp_v3.param or models/plane.parm")


def auto_workers() -> int:
    """Auto-size the parallel-SITL pool to ~3/4 of this machine's cores.

    Each worker is one CPU-bound `arduplane` SITL process plus a mostly-idle
    Python flight driver. We deliberately leave ~1/4 of the cores free as a
    safety margin (OS, MAVLink I/O, the orchestrator, and so each SITL can hold
    its requested --speedup without CPU starvation). On a 192-core box this is
    144 workers; on a 96-core box 72; on a 16-core dev box 12. Override with
    --workers to tune.
    """
    cores = os.cpu_count() or 4
    return max(1, (cores * 3) // 4)


# ===================================================================
#  Parameter sampling
# ===================================================================
def sample_parameters(params_cfg: dict, rng: np.random.Generator) -> dict:
    """Return {param_name: perturbed_value}, N(0, sigma_3/3) clipped to ±sigma_3."""
    perturbed = {}
    for name, cfg in params_cfg.items():
        nom   = cfg["nominal"]
        s3    = cfg["sigma_3"]
        sigma = s3 / 3.0
        delta = rng.normal(0.0, sigma)
        delta = float(np.clip(delta, -s3, s3))
        perturbed[name] = nom + delta
    return perturbed


def write_override_file(filepath: str, params: dict):
    """Write the key=value text file consumed by LAT_SIM_MonteCarlo.cpp."""
    with open(filepath, "w") as f:
        for k, v in params.items():
            f.write(f"{k}={v:.10f}\n")


# ===================================================================
#  MAVLink helpers (raw pymavlink — no auto_flight4)
# ===================================================================
def wait_for_sitl_port(port: int, timeout: float = 120.0) -> bool:
    """Block until SITL's MAVLink TCP server is accepting connections."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=2):
                return True
        except OSError:
            time.sleep(1.0)
    return False


def connect_mavlink(port: int, source_system: int = 250,
                    retries: int = 40, delay: float = 2.0):
    """Connect to SITL and lock onto the AUTOPILOT heartbeat (skip GCS heartbeats),
    pinning target/source so RC overrides are addressed correctly. Returns the
    connection or None."""
    conn_str = f"tcp:127.0.0.1:{port}"
    for _ in range(retries):
        try:
            conn = mavutil.mavlink_connection(conn_str, source_system=source_system)
            t0 = time.time()
            while time.time() - t0 < 10:
                hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
                if hb is None:
                    break
                if hb.type != mavutil.mavlink.MAV_TYPE_GCS:
                    conn.target_system = hb.get_srcSystem()
                    conn.sysid = hb.get_srcSystem()
                    return conn
            conn.close()
        except Exception:
            pass
        time.sleep(delay)
    return None


def request_data_streams(conn):
    """Ask SITL to stream telemetry at 10 Hz."""
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    time.sleep(0.5)


def set_param(conn, name: str, value: float, retries: int = 4) -> bool:
    """Set one parameter and wait for the autopilot to echo it back."""
    for _ in range(retries):
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode("ascii"), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        if ack and ack.param_id.replace("\x00", "") == name:
            return True
    return False


def set_mode(conn, name: str, timeout: float = 15.0) -> bool:
    """Switch flight mode by name (e.g. 'FBWA') and confirm via heartbeat."""
    mode_map = conn.mode_mapping()
    if name not in mode_map:
        return False
    mid = mode_map[name]
    for _ in range(3):
        conn.set_mode(mid)
        time.sleep(0.3)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.custom_mode == mid:
            return True
        conn.set_mode(mid)
    return False


def arm_vehicle(conn, timeout: float = 40.0) -> bool:
    """Arm the vehicle; falls back to force-arm after a few tries."""
    t0 = time.time()
    n = 0
    while time.time() - t0 < timeout:
        n += 1
        force = 21196 if n > 4 else 0
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, force, 0, 0, 0, 0, 0)
        for _ in range(5):
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        time.sleep(1.5)
    return False


def disarm_vehicle(conn, timeout: float = 20.0) -> bool:
    """Disarm the vehicle after touchdown; falls back to force-disarm after a few tries."""
    t0 = time.time()
    n = 0
    while time.time() - t0 < timeout:
        n += 1
        force = 21196 if n > 3 else 0
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, force, 0, 0, 0, 0, 0)
        for _ in range(5):
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        time.sleep(1.0)
    return False


def wait_ekf_ready(conn, timeout: float = 60) -> bool:
    """Block until the EKF is healthy + GPS-aided so its attitude re-alignment
    happens on the GROUND, not mid-climb (otherwise the ~20 s GPS-pickup
    alignment lands in the climb and spikes the pitch ESTIMATE)."""
    need = 0x01 | 0x02 | 0x10        # EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_ABS
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = conn.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        if s and (s.flags & need) == need:
            return True
    return False


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance (m) between two lat/lon points (degrees)."""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl   = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


# ===================================================================
#  Flight profile — mission_ustol_2 open-loop FBWA takeoff/climb/flare/hold
# ===================================================================
_PUMP_TYPES = ["GLOBAL_POSITION_INT", "VFR_HUD", "ATTITUDE"]


def _pump(conn, st):
    """Drain ALL buffered telemetry each call and keep the LATEST values.

    Read the whole buffer, not one message: the AP streams faster than this loop
    consumes, so a single recv per call lets the socket buffer back up and every
    read returns an OLDER message -> st['alt']/as lag and the flare trigger
    never fires.  First read blocks briefly so we always advance.
    """
    m = conn.recv_match(type=_PUMP_TYPES, blocking=True, timeout=2)
    while m is not None:
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            st["alt"] = m.relative_alt / 1000.0
            st["lat"] = m.lat / 1e7
            st["lon"] = m.lon / 1e7
        elif t == "VFR_HUD":
            st["as"]    = m.airspeed
            st["climb"] = m.climb
            st["thr"]   = m.throttle
        elif t == "ATTITUDE":
            st["roll"]  = math.degrees(m.roll)
            st["pitch"] = math.degrees(m.pitch)
        m = conn.recv_match(type=_PUMP_TYPES, blocking=False)


def fly_ustol2(conn, m, writer, st, perf, sitl_proc, t0_flight, deadline):
    """Fly the mission_ustol_7 ALL-FBWA profile (takeoff -> cruise -> approach ->
    POWERED flare -> touchdown -> rollout/disarm) via RC overrides. NO loiter —
    the override stream runs straight from the cruise hold into the approach.

    m       : the config["mission"] dict; profile knobs live in m["ustol2"]
    writer  : csv.writer with the per-case header already written
    st/perf : mutable state / performance dicts (updated in place)
    Returns : 'landed'        touchdown reached, then rolled out + disarmed
              'no_touchdown'   reached the ground sequence but never crossed TD_ALT
              'crashed_climb' | 'crashed_cruise' |
              'crashed_approach' | 'crashed_flare'   SITL process died in that phase
              'climb_timeout'  never reached cruise altitude
              'deadline'       hard per-case deadline hit mid-mission
    """
    # --- knobs (config "mission"."ustol2" block; defaults match mission_ustol_2.py) ---
    u = m.get("ustol2", {})
    CRUISE_ALT     = u.get("cruise_alt_m", m.get("takeoff_alt_m", 100.0))
    CRUISE_AS      = u.get("cruise_as_mps", 12.0)
    CRUISE_CAP_THR = u.get("cruise_cap_thr_pwm", 1660)
    K_CAP          = u.get("k_cap", 3.0)
    HOLD_T         = u.get("cruise_hold_s", 10.0)
    LEVEL_BAND     = u.get("level_band_m", 10.0)
    LEVEL_K        = u.get("level_k", 0.5)        # exp level-off rate (1/m): blend = e^(-K*|alt-CRUISE_ALT|)
    V_R            = u.get("v_rotate_mps", 6.0)
    V_LO           = u.get("v_liftoff_mps", 8.0)
    V_CLIMB        = u.get("v_climb_mps", 10.0)
    TH_LO          = u.get("theta_liftoff_deg", 7.0)
    TH_CLIMB       = u.get("theta_climb_deg", 16.5)
    RATE_LIM       = u.get("pitch_rate_lim_dps", 5.0)
    AOA_CAP        = u.get("aoa_cap_deg", 7.0)
    V_HOLD         = u.get("v_hold_mps", 10.0)
    K_SPD          = u.get("k_spd", 1.0)
    GND_PITCH      = u.get("ground_pitch_deg", -2.0)
    RC2_TRIM       = u.get("rc2_trim", 1500)
    RC2_MAX        = u.get("rc2_max", 2000)
    PTCH_LIM_MAX   = u.get("ptch_lim_max_deg", 20.0)
    PITCH_SIGN     = u.get("pitch_sign", -1)
    THR_PWM        = u.get("throttle_climb_pwm", 1900)
    CLIMB_TO       = u.get("climb_timeout_s", 120.0)

    # --- landing: approach schedule + POWERED flare (defaults match mission_ustol_7.py) ---
    V_APP          = u.get("v_app_mps", 12.0)         # approach airspeed held on PITCH
    SINK_APP       = u.get("sink_app_mps", -1.5)      # target approach sink rate (m/s, -=down)
    SINK_TD        = u.get("sink_td_mps", -0.3)       # target touchdown sink rate (m/s)
    FLARE_ALT      = u.get("flare_alt_m", 8.0)        # begin flare below this height (m AGL)
    TD_ALT         = u.get("td_alt_m", 0.5)           # touchdown when alt drops below this (m)
    ROLLOUT_T      = u.get("rollout_s", 4.0)          # idle + nose-down on the ground, then disarm
    K_APP_PITCH    = u.get("k_app_pitch", 1.0)        # approach pitch-on-speed gain (deg per m/s)
    APP_PITCH_MIN  = u.get("app_pitch_min_deg", -10.0)
    APP_PITCH_MAX  = u.get("app_pitch_max_deg", 4.0)
    THR_APP_TRIM   = u.get("thr_app_trim_pwm", 1450)  # approach throttle trim (pwm, ~45%)
    K_THR_SINK     = u.get("k_thr_sink", 80.0)        # approach throttle gain: pwm per (m/s) sink err
    THR_APP_MIN    = u.get("thr_app_min_pwm", 1150)
    THR_APP_MAX    = u.get("thr_app_max_pwm", 1720)
    V_MIN_APP      = u.get("v_min_app_mps", 9.0)      # stall guard: no nose-up below this airspeed
    TH_FLARE_HOLD  = u.get("th_flare_hold_deg", 3.0)  # modest nose-up held in the flare (deg)
    K_THR_FLARE    = u.get("k_thr_flare", 110.0)      # flare throttle gain: pwm per (m/s) sink err
    THR_FLARE_MIN  = u.get("thr_flare_min_pwm", 1450) # authority floor (~45%) until touchdown
    THR_FLARE_MAX  = u.get("thr_flare_max_pwm", 1800) # ~80%: let power arrest the sink
    IDLE_THR_PWM   = u.get("idle_thr_pwm", 1000)      # idle throttle for AFTER touchdown only
    APP_TO         = u.get("approach_timeout_s", 120.0)
    FLARE_TO       = u.get("flare_timeout_s", 30.0)

    def _smooth(x):
        x = 0.0 if x < 0 else (1.0 if x > 1 else x)
        return x * x * (3.0 - 2.0 * x)

    def theta_target(V):
        if V <= V_R:     return 0.0
        if V <= V_LO:    return TH_LO * _smooth((V - V_R) / (V_LO - V_R))
        if V <= V_CLIMB: return TH_LO + (TH_CLIMB - TH_LO) * _smooth((V - V_LO) / (V_CLIMB - V_LO))
        return TH_CLIMB

    def rate_limit(prev, target, dt):
        step = RATE_LIM * dt
        return max(prev - step, min(target, prev + step))

    def pitch_pwm(theta_deg):
        frac = PITCH_SIGN * theta_deg / PTCH_LIM_MAX
        frac = -1.0 if frac < -1 else (1.0 if frac > 1 else frac)
        return int(RC2_TRIM + frac * (RC2_MAX - RC2_TRIM))

    def send_sticks(theta_deg, thr):
        # AETR override order: ch1 roll=neutral, ch2 pitch, ch3 throttle, ch4 yaw=neutral.
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component,
            1500, pitch_pwm(theta_deg), int(thr), 1500, 0, 0, 0, 0)

    def release_sticks():
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

    _last_csv = [0.0]

    def write_row(phase, theta):
        now = time.time()
        if now - _last_csv[0] < 0.2:          # ~5 Hz
            return
        _last_csv[0] = now
        writer.writerow([
            "%.2f" % (now - t0_flight), phase,
            "%.2f" % st["alt"], "%.2f" % st["as"],
            "%.2f" % st["roll"], "%.2f" % st["pitch"],
            "%.2f" % st["climb"], "%.0f" % st["thr"],
            "%.2f" % theta,
            "%.7f" % (st["lat"] or 0.0), "%.7f" % (st["lon"] or 0.0)])

    start_lat = start_lon = None

    # ---- ground roll + climb + flare ----
    v_peak = 0.0
    theta = 0.0
    flare0 = None
    t_climb = time.time()
    t_prev = t_climb
    while True:
        if sitl_proc.poll() is not None:
            return "crashed_climb"
        now = time.time()
        if now > deadline or now - t_climb > CLIMB_TO:
            return "climb_timeout"
        _pump(conn, st)
        dt = min(now - t_prev, 0.15)
        t_prev = now
        v_peak = max(v_peak, st["as"])
        if start_lat is None and st["lat"] is not None:
            start_lat, start_lon = st["lat"], st["lon"]
        # Liftoff detection -> Vlof + ground-roll distance (start pos -> liftoff pos).
        if perf["liftoff_time"] is None and st["alt"] >= 0.5:
            perf["liftoff_time"]  = now - t0_flight
            perf["liftoff_speed"] = st["as"]
            if start_lat is not None and st["lat"] is not None:
                perf["ground_roll_m"] = haversine_m(start_lat, start_lon,
                                                     st["lat"], st["lon"])
        fpa = math.degrees(math.asin(max(-1.0, min(1.0, st["climb"] / max(st["as"], 0.1)))))
        spd_trim = K_SPD * max(0.0, V_HOLD - st["as"])
        thr_cmd = THR_PWM
        phase = "CLIMB"
        if st["alt"] >= CRUISE_ALT - LEVEL_BAND:
            # EXPONENTIAL level-off: blend s = e^(-K*|alt-CRUISE_ALT|) goes 0 far below
            # cruise (full climb pitch+throttle held) -> 1 at CRUISE_ALT (level + cruise trim).
            # Unlike the old smoothstep (which started bleeding power at CRUISE_ALT-LEVEL_BAND
            # and stranded marginal draws ~95-98 m with throttle already pulled to ~71%), this
            # holds near-full climb authority until the aircraft is within a few metres of
            # CRUISE_ALT, forcing it across the climb-complete gate and into the cruise phase.
            if flare0 is None:
                flare0 = theta
            s = math.exp(-LEVEL_K * abs(st["alt"] - CRUISE_ALT))
            target  = flare0 * (1.0 - s)
            thr_cmd = THR_PWM + (CRUISE_CAP_THR - THR_PWM) * s
        elif v_peak < V_R:
            target = GND_PITCH
            phase = "GROUND"
        else:
            # AoA guard: never command past measured FPA + AOA_CAP; never below horizon.
            target = min(theta_target(v_peak) - spd_trim, fpa + AOA_CAP)
            target = max(0.0, target)
        theta = rate_limit(theta, target, dt)
        send_sticks(theta, int(thr_cmd))
        write_row(phase, theta)
        if st["alt"] >= CRUISE_ALT - 1.0:
            break
        time.sleep(0.05)                       # ~20 Hz override stream

    # ---- level cruise hold (open-loop FBWA; speed-on-pitch + cruise throttle) ----
    t_hold = time.time()
    while time.time() - t_hold < HOLD_T:
        if sitl_proc.poll() is not None:
            return "crashed_cruise"
        if time.time() > deadline:
            return "deadline"
        _pump(conn, st)
        theta_cap = max(-5.0, min(TH_CLIMB, K_CAP * (st["as"] - CRUISE_AS)))
        send_sticks(theta_cap, CRUISE_CAP_THR)
        write_row("CRUISE", theta_cap)
        time.sleep(0.05)

    # NO LOITER: keep streaming overrides straight into the approach — the aircraft
    # never leaves FBWA, so there is no nav handoff / RC-loss failsafe to manage.

    # ---- APPROACH / glide (FBWA, DECOUPLED): PITCH holds approach speed (speed-on-pitch),
    #      THROTTLE holds the sink rate (sink-on-throttle around THR_APP_TRIM). To FLARE_ALT.
    theta   = 0.0
    thr_app = float(THR_APP_TRIM)
    t_app   = time.time()
    t_prev  = t_app
    while time.time() - t_app < APP_TO:
        if sitl_proc.poll() is not None:
            return "crashed_approach"
        if time.time() > deadline:
            return "deadline"
        _pump(conn, st)
        now = time.time()
        dt = min(now - t_prev, 0.15)
        t_prev = now
        # PITCH = speed-on-pitch (stall guard: never command nose-up below V_MIN_APP).
        target = K_APP_PITCH * (st["as"] - V_APP)
        hi = APP_PITCH_MAX if st["as"] > V_MIN_APP else 0.0
        target = max(APP_PITCH_MIN, min(target, hi))
        theta = rate_limit(theta, target, dt)
        # THROTTLE = sink-on-throttle (proportional around the approach trim).
        thr_app = max(THR_APP_MIN,
                      min(THR_APP_TRIM + K_THR_SINK * (SINK_APP - st["climb"]), THR_APP_MAX))
        send_sticks(theta, int(thr_app))
        write_row("APPROACH", theta)
        if st["alt"] <= FLARE_ALT:
            break
        time.sleep(0.05)

    # ---- POWERED FLARE: THROTTLE arrests the sink around a height-scheduled sink target;
    #      elevator only sets a modest, achievable nose-up. Throttle is cut to idle ONLY
    #      after touchdown (rollout below). ----
    touchdown = False
    t_flare = time.time()
    t_prev  = t_flare
    while time.time() - t_flare < FLARE_TO:
        if sitl_proc.poll() is not None:
            return "crashed_flare"
        if time.time() > deadline:
            return "deadline"
        _pump(conn, st)
        now = time.time()
        dt = min(now - t_prev, 0.15)
        t_prev = now
        h = max(0.0, st["alt"])
        sink_cmd  = SINK_TD + (SINK_APP - SINK_TD) * max(0.0, min(h / FLARE_ALT, 1.0))
        theta     = rate_limit(theta, TH_FLARE_HOLD, dt)            # ease to a modest nose-up
        thr_flare = max(THR_FLARE_MIN,
                        min(THR_APP_TRIM + K_THR_FLARE * (sink_cmd - st["climb"]), THR_FLARE_MAX))
        send_sticks(theta, int(thr_flare))
        write_row("FLARE", theta)
        if st["alt"] <= TD_ALT:
            touchdown = True
            break
        time.sleep(0.05)

    # ---- ROLLOUT + DISARM. NOW cut throttle to idle (first time since takeoff) + slight
    #      nose-down while speed bleeds, then release the overrides and disarm. ----
    t_roll = time.time()
    while time.time() - t_roll < ROLLOUT_T:
        if sitl_proc.poll() is not None:
            break
        if time.time() > deadline:
            break
        _pump(conn, st)
        send_sticks(GND_PITCH, IDLE_THR_PWM)
        write_row("ROLLOUT", GND_PITCH)
        time.sleep(0.05)
    release_sticks()
    disarm_vehicle(conn)

    return "landed" if touchdown else "no_touchdown"


# ===================================================================
#  Per-case metrics (from the per-case telemetry CSV this runner writes)
# ===================================================================
def metrics_from_csv(csv_path: str, home_lat: float, home_lon: float) -> dict:
    """Derive the frozen-schema flight metrics from the per-case telemetry CSV."""
    m = dict(max_alt_m=0.0, max_roll_deg=0.0, max_pitch_deg=0.0,
             min_airspeed=None, max_airspeed=0.0,
             landing_lat=None, landing_lon=None, landing_err_m=None, landing_roll_m=None,
             time_to_takeoff_s=None, time_to_cruise_s=None, time_to_land_s=None)
    last_lat = last_lon = None
    td_lat = td_lon = None                       # first ROLLOUT-phase fix ~= touchdown point

    def _f(row, key):
        try:
            return float(row.get(key, "") or 0.0)
        except (ValueError, TypeError):
            return 0.0

    try:
        with open(csv_path, newline="") as f:
            for row in csv.DictReader(f):
                alt = _f(row, "alt_agl_m")
                m["max_alt_m"] = max(m["max_alt_m"], alt)
                m["max_roll_deg"] = max(m["max_roll_deg"], abs(_f(row, "roll_deg")))
                m["max_pitch_deg"] = max(m["max_pitch_deg"], abs(_f(row, "pitch_deg")))
                a = _f(row, "airspeed")
                m["max_airspeed"] = max(m["max_airspeed"], a)
                if a > 1.0:
                    m["min_airspeed"] = a if m["min_airspeed"] is None else min(m["min_airspeed"], a)
                t = _f(row, "time_s")
                phase = row.get("phase", "")
                if m["time_to_takeoff_s"] is None and alt >= 0.8:
                    m["time_to_takeoff_s"] = t
                if m["time_to_cruise_s"] is None and phase == "CRUISE":
                    m["time_to_cruise_s"] = t
                # ROLLOUT begins the instant after touchdown -> use it as the land time.
                if m["time_to_land_s"] is None and phase == "ROLLOUT":
                    m["time_to_land_s"] = t
                la, lo = row.get("lat"), row.get("lon")
                if la and lo:
                    try:
                        last_lat, last_lon = float(la), float(lo)
                        if td_lat is None and phase == "ROLLOUT":
                            td_lat, td_lon = last_lat, last_lon
                    except ValueError:
                        pass
    except FileNotFoundError:
        return m

    if last_lat is not None:
        m["landing_lat"], m["landing_lon"] = last_lat, last_lon
        m["landing_err_m"] = haversine_m(home_lat, home_lon, last_lat, last_lon)
        # Ground roll on landing = touchdown fix -> final (stopped) fix.
        if td_lat is not None:
            m["landing_roll_m"] = haversine_m(td_lat, td_lon, last_lat, last_lon)
    return m


def plot_case(csv_path: str):
    """Optional per-case PNG (headless Agg backend); never raises out."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t, alt, aspd, pitch, theta = [], [], [], [], []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t.append(float(row["time_s"]))
                alt.append(float(row["alt_agl_m"]))
                aspd.append(float(row["airspeed"]))
                pitch.append(float(row["pitch_deg"]))
                theta.append(float(row["theta_cmd_deg"]))
            except (ValueError, KeyError):
                pass
    if not t:
        return
    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(t, alt);  ax[0].set_ylabel("alt AGL (m)"); ax[0].grid(True)
    ax[1].plot(t, aspd); ax[1].set_ylabel("airspeed (m/s)"); ax[1].grid(True)
    ax[2].plot(t, pitch, label="pitch")
    ax[2].plot(t, theta, "--", label="theta cmd")
    ax[2].set_ylabel("deg"); ax[2].set_xlabel("t (s)"); ax[2].legend(); ax[2].grid(True)
    fig.tight_layout()
    fig.savefig(csv_path.replace(".csv", ".png"), dpi=90)
    plt.close(fig)


# ===================================================================
#  Per-case report
# ===================================================================
def write_case_report(filepath, case_id, perturbed, nominals, result):
    with open(filepath, "w") as f:
        f.write(f"Monte Carlo Case #{case_id:04d}  [uSTOL all-FBWA + landing profile, no loiter]\n")
        f.write("=" * 70 + "\n\n")
        f.write(f"Timestamp : {datetime.now().isoformat()}\n")
        f.write(f"Duration  : {result.get('duration_s', 0):.1f} s\n\n")

        f.write(f"{'Parameter':<22} {'Nominal':>14} {'Perturbed':>14} {'Delta%':>9}\n")
        f.write("-" * 70 + "\n")
        for name in perturbed:
            nom  = nominals[name]
            pert = perturbed[name]
            pct  = ((pert - nom) / abs(nom) * 100) if abs(nom) > 1e-12 else 0.0
            f.write(f"{name:<22} {nom:>14.6f} {pert:>14.6f} {pct:>+8.2f}%\n")

        f.write("\n" + "=" * 70 + "\nResult\n" + "-" * 70 + "\n")
        f.write(f"  Profile result    : {result.get('land_result', 'unknown')}\n")
        f.write(f"  Takeoff success   : {result.get('takeoff_success', False)}\n")
        f.write(f"  Landed (touchdown): {result.get('mission_complete', False)}\n")
        f.write(f"  Max altitude (m)  : {result.get('max_alt_m', 0):.1f}\n")
        f.write(f"  Ground roll (m)   : {result.get('ground_roll_m')}\n")
        f.write(f"  Liftoff Vlof (m/s): {result.get('liftoff_speed')}\n")
        f.write(f"  Landing roll (m)  : {result.get('landing_roll_m')}\n")
        f.write(f"  Landing err (m)   : {result.get('landing_err_m')}\n")
        f.write(f"  Time to land (s)  : {result.get('time_to_land_s')}\n")
        f.write(f"  Exit reason       : {result.get('exit_reason', 'unknown')}\n")


# ===================================================================
#  Single-case worker
# ===================================================================
def _run_case(case_id, case_seed, instance_id, config, perturbed, output_dir,
              workspace, speedup, save_plot, hard_timeout, params_file):
    """Execute one Monte Carlo case: launch SITL, fly the uSTOL all-FBWA + landing profile (no loiter)."""
    result = dict(
        case_id=case_id, case_seed=case_seed,
        takeoff_success=False, mission_complete=False,
        duration_s=0.0, max_alt_m=0.0, land_result="unknown",
        ground_roll_m=None, liftoff_speed=None, landing_roll_m=None,
        exit_reason="unknown")

    mission_cfg = config["mission"]

    # ---- Directories & override file ----
    case_dir   = os.path.join(output_dir, f"case_{case_id:04d}")
    worker_dir = os.path.join(output_dir, f"worker_{instance_id}")
    os.makedirs(case_dir, exist_ok=True)
    os.makedirs(worker_dir, exist_ok=True)
    override_file = os.path.join(case_dir, "overrides.txt")
    write_override_file(override_file, perturbed)

    # ---- SITL launch command ----
    binary   = os.path.join(workspace, "build", "sitl", "bin", "arduplane")
    parm     = params_file
    rwy_hdg  = mission_cfg.get("runway_heading_deg", 285)
    home_str = (f"{mission_cfg['home_lat']},{mission_cfg['home_lon']},"
                f"{mission_cfg['home_alt_m']},{rwy_hdg}")
    cmd = [
        binary, "-w",
        "--model", "plane",
        "--defaults", parm,
        "--home", home_str,
        "--sim-address", "127.0.0.1",
        "--speedup", str(speedup),
        f"-I{instance_id}",
    ]

    env = os.environ.copy()
    env["LAT_MC_OVERRIDE_FILE"] = override_file
    # Each case's FDM time-series CSV goes straight into its own case dir
    # (avoids PID collisions in a shared logs/ dir across parallel workers).
    env["LAT_SIM_LOG_DIR"] = case_dir + os.sep
    env["DISPLAY"] = ""

    tcp_port  = 5760 + 10 * instance_id
    case_csv  = os.path.join(case_dir, f"case_{case_id:04d}_flight.csv")
    sitl_proc = None
    conn      = None
    watchdog  = None
    csv_fp    = None
    t_start   = time.time()
    ucfg      = mission_cfg.get("ustol2", {})
    gcs_sysid = int(ucfg.get("gcs_sysid", 250))

    try:
        sitl_proc = subprocess.Popen(
            cmd, cwd=worker_dir, env=env,
            stdout=open(os.path.join(case_dir, "sitl_stdout.log"), "w"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid)
        _log(case_id, f"SITL PID {sitl_proc.pid}  inst {instance_id}  "
                       f"TCP {tcp_port}  speedup {speedup}x")

        # Hard watchdog: force-kill this case's SITL if it outlives hard_timeout,
        # so a single hung case can never freeze a worker slot for the whole run.
        watchdog = threading.Timer(hard_timeout, _kill_proc, args=(sitl_proc,))
        watchdog.daemon = True
        watchdog.start()
        deadline = t_start + hard_timeout

        if not wait_for_sitl_port(tcp_port, timeout=120):
            result["exit_reason"] = "sitl_port_never_opened"
            return result
        time.sleep(1.0)

        conn = connect_mavlink(tcp_port, source_system=gcs_sysid, retries=40, delay=2.0)
        if conn is None:
            result["exit_reason"] = "mavlink_connect_failed"
            return result
        request_data_streams(conn)

        # RC override acceptance: the sender sysid must equal the AP's GCS sysid
        # (param renamed SYSID_MYGCS -> MAV_GCS_SYSID on newer builds; set whichever exists).
        set_param(conn, "SYSID_MYGCS", gcs_sysid) or set_param(conn, "MAV_GCS_SYSID", gcs_sysid)
        for n, v in [
            ("ARMING_CHECK",     0),                              # headless arming
            ("AIRSPEED_MIN",     7),                              # liftoff sits above the floor
            ("PTCH_LIM_MAX_DEG", ucfg.get("ptch_lim_max_deg", 20)),
            ("PTCH_LIM_MIN_DEG", -20),                            # allow nose-down for the glide
            ("TECS_PITCH_MAX",   20),
            ("AIRSPEED_CRUISE",  ucfg.get("cruise_as_mps", 12.0)),
        ]:                                                         # no WP_LOITER_RAD: this profile never loiters
            set_param(conn, n, v)

        if not set_mode(conn, "FBWA"):
            result["exit_reason"] = "fbwa_mode_failed"
            return result
        wait_ekf_ready(conn, timeout=ucfg.get("ekf_timeout_s", 60))
        if not arm_vehicle(conn):
            result["exit_reason"] = "arm_failed"
            return result
        _log(case_id, "armed — flying uSTOL FBWA open-loop profile")

        # Per-case telemetry CSV (drives metrics_from_csv + summary).
        csv_fp = open(case_csv, "w", newline="")
        writer = csv.writer(csv_fp)
        writer.writerow([
            "time_s", "phase", "alt_agl_m", "airspeed", "roll_deg",
            "pitch_deg", "climb_mps", "throttle_pct", "theta_cmd_deg", "lat", "lon"])

        st = {"alt": 0.0, "as": 0.0, "climb": 0.0, "lat": None, "lon": None,
              "thr": 0.0, "roll": 0.0, "pitch": 0.0}
        perf = {"liftoff_time": None, "liftoff_speed": None, "ground_roll_m": None}
        t0_flight = time.time()

        land_result = fly_ustol2(conn, mission_cfg, writer, st, perf,
                                 sitl_proc, t0_flight, deadline)
        _log(case_id, f"profile returned: {land_result}")

        # ---- Outcome ----
        result["land_result"]      = land_result
        result["takeoff_success"]  = perf["liftoff_time"] is not None
        result["mission_complete"] = (land_result == "landed")    # full mission = touchdown
        result["ground_roll_m"]    = perf["ground_roll_m"]
        result["liftoff_speed"]    = perf["liftoff_speed"]
        result["landing_roll_m"]   = None                 # derived from the CSV (metrics_from_csv)
        result["exit_reason"]      = land_result

    except Exception as exc:
        result["exit_reason"] = f"exception: {exc}"
        _log(case_id, f"ERROR: {exc}")
        with open(os.path.join(case_dir, "exception.txt"), "w") as f:
            traceback.print_exc(file=f)

    finally:
        if csv_fp:
            try:
                csv_fp.close()
            except Exception:
                pass
        if watchdog:
            watchdog.cancel()
        if conn:
            try:
                conn.close()
            except Exception:
                pass
        result["duration_s"] = time.time() - t_start
        # Derived flight metrics (frozen summary schema) from the telemetry CSV.
        result.update(metrics_from_csv(
            case_csv, mission_cfg["home_lat"], mission_cfg["home_lon"]))
        result["params"] = perturbed

        if sitl_proc:
            sitl_pid = sitl_proc.pid
            _kill_proc(sitl_proc)
            # The FDM CSV (Output.cpp) should already be in case_dir via LAT_SIM_LOG_DIR;
            # fall back to moving it from the default log dirs if an older binary ignored the env.
            for d in (case_dir,
                      os.path.join(workspace, "ustol_sims", "logs"),
                      os.path.join(workspace, "Logs_Simulations")):
                if d == case_dir:
                    continue
                for csv_file in glob.glob(os.path.join(d, f"sim_output_*_pid{sitl_pid}.csv")):
                    try:
                        shutil.move(csv_file, os.path.join(
                            case_dir, os.path.basename(csv_file)))
                    except Exception:
                        pass
            # Drop the dataflash .BIN logs (large, not needed for MC).
            bin_log_dir = os.path.join(worker_dir, "logs")
            if os.path.isdir(bin_log_dir):
                shutil.rmtree(bin_log_dir, ignore_errors=True)

        # Optional per-case PNG (headless Agg backend).
        if save_plot:
            try:
                plot_case(case_csv)
            except Exception:
                pass

        nominals = {k: v["nominal"] for k, v in config["params"].items()}
        write_case_report(
            os.path.join(case_dir, f"case_{case_id:04d}_report.txt"),
            case_id, perturbed, nominals, result)
        # Machine-readable result — drives summary.csv rebuild + --resume.
        try:
            with open(os.path.join(case_dir, "result.json"), "w") as jf:
                json.dump(result, jf)
        except Exception:
            pass

    return result


# ===================================================================
#  Process management
# ===================================================================
def _kill_proc(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=10)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def _log(case_id, msg):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"  [{ts}] Case {case_id:04d}: {msg}", file=sys.__stdout__, flush=True)


# ===================================================================
#  Instance-pool wrapper
# ===================================================================
_instance_queue = None


def _init_worker(q):
    global _instance_queue
    _instance_queue = q


def _worker_entry(args):
    (case_id, case_seed, config, perturbed, output_dir, workspace,
     speedup, save_plot, hard_timeout, params_file) = args
    inst = _instance_queue.get()
    try:
        return _run_case(case_id, case_seed, inst, config, perturbed, output_dir,
                         workspace, speedup, save_plot, hard_timeout, params_file)
    finally:
        _instance_queue.put(inst)


# ===================================================================
#  Summary output
# ===================================================================
# Frozen output schema — DO NOT add/remove columns after a campaign starts
# (re-deriving metrics from thousands of logs afterward is painful).
SUMMARY_COLUMNS = [
    "case_id", "case_seed", "takeoff_success", "mission_complete", "land_result",
    "exit_reason", "duration_s",
    "ground_roll_m", "liftoff_speed", "landing_roll_m",
    "max_alt_m", "max_roll_deg", "max_pitch_deg", "min_airspeed", "max_airspeed",
    "landing_lat", "landing_lon", "landing_err_m",
    "time_to_takeoff_s", "time_to_cruise_s", "time_to_land_s",
]


def load_results(output_dir):
    """Load every completed case's result.json (covers this run + resumed/prior)."""
    out = []
    for rj in glob.glob(os.path.join(output_dir, "case_*", "result.json")):
        try:
            with open(rj) as f:
                out.append(json.load(f))
        except Exception:
            pass
    return out


def write_summary_csv(path, output_dir, param_names):
    """Rebuild summary.csv from all per-case result.json files (resume-safe)."""
    results = load_results(output_dir)
    fieldnames = SUMMARY_COLUMNS + [f"p_{n}" for n in param_names]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for r in sorted(results, key=lambda x: x.get("case_id", 0)):
            row = {k: r.get(k) for k in SUMMARY_COLUMNS}
            params = r.get("params") or {}
            for n in param_names:
                row[f"p_{n}"] = params.get(n)
            writer.writerow(row)
    return results


# ===================================================================
#  Main
# ===================================================================
def prompt_num_runs(default=5000):
    """Ask the user how many Monte Carlo runs to do (Enter = default).

    Falls back to `default` on a non-interactive shell (headless AWS / piped
    input) so the runner never blocks waiting on stdin.
    """
    if not sys.stdin.isatty():
        print(f"  (non-interactive shell) using default {default} runs")
        return default
    while True:
        try:
            resp = input(f"Number of Monte Carlo runs [{default}]: ").strip()
        except EOFError:
            return default
        if not resp:
            return default
        try:
            n = int(resp)
            if n > 0:
                return n
        except ValueError:
            pass
        print(f"  please enter a positive integer (or Enter for {default})")


def main():
    parser = argparse.ArgumentParser(
        description="Monte Carlo Runner — uSTOL all-FBWA + landing profile, no loiter (mission_ustol_7)")
    parser.add_argument("--config", default=str(DEFAULT_CFG))
    parser.add_argument("--runs", type=int, default=None,
                        help="number of cases. If omitted, prompt interactively "
                             "(default 5000); on a non-interactive shell use 5000.")
    parser.add_argument("--workers", type=int, default=None,
                        help="parallel SITL instances (default: auto-size to "
                             "this machine, ~cores-4)")
    parser.add_argument("--speedup", type=int, default=1,
                        help="SITL sim speed multiplier (default 1 = real-time, "
                             "matches the tuned profile exactly)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--base-instance", type=int, default=100,
                        help="starting SITL instance ID (ports = 5760+10*id)")
    parser.add_argument("--plot", action="store_true",
                        help="save a per-case telemetry PNG (slower)")
    parser.add_argument("--output-dir", default=None,
                        help="campaign output dir (default: timestamped). Use a "
                             "FIXED dir with --resume to continue an interrupted run.")
    parser.add_argument("--resume", action="store_true",
                        help="skip cases that already have a result.json in "
                             "--output-dir (crash / spot-interrupt recovery)")
    parser.add_argument("--hard-timeout", type=float, default=None,
                        help="seconds before a stuck case's SITL is force-killed "
                             "(default: config mission.hard_timeout_s or 420)")
    parser.add_argument("--params", default=None,
                        help="ArduPilot defaults/param file fed to every SITL "
                             "(--defaults). Default: auto-resolve "
                             "ustol_sims/params_imp_v3.param. Pass e.g. "
                             "ustol_sims/params_imp_v4land.param to use the landing-tuned set.")
    args = parser.parse_args()

    config      = load_config(args.config)
    # Param/defaults file: --params wins; else the historical auto-resolve order.
    if args.params:
        params_file = os.path.abspath(args.params)
        if not os.path.isfile(params_file):
            print(f"\nERROR: --params file not found: {params_file}")
            sys.exit(1)
    else:
        params_file = resolve_defaults_parm()
    # Runs: --runs wins; otherwise prompt the user (default 5000); on a
    # non-interactive shell (e.g. headless AWS) fall back to the default.
    num_runs    = args.runs if args.runs is not None else prompt_num_runs(5000)
    # Worker precedence: explicit --workers > auto-size to this machine.
    max_workers = args.workers or auto_workers()
    # Never spin up more SITL instances than there are cases to run.
    max_workers = max(1, min(max_workers, num_runs))
    base_inst   = args.base_instance
    hard_timeout = (args.hard_timeout
                    or config["mission"].get("hard_timeout_s", 420))

    print("=" * 70)
    print("  LAT Monte Carlo Runner — uSTOL all-FBWA + landing profile, no loiter (mission_ustol_7)")
    print("=" * 70)
    print(f"  Runs          : {num_runs}")
    print(f"  CPU cores     : {os.cpu_count()}")
    print(f"  Workers       : {max_workers}"
          f"{'' if args.workers else '  (auto)'}")
    print(f"  Speedup       : {args.speedup}x")
    if args.speedup > 1:
        print("  NOTE          : speedup>1 + high worker count starves each "
              "SITL of CPU;\n"
              "                  validate pass-rate against a 1x baseline.")
    print(f"  Seed          : {args.seed}")
    print(f"  Hard timeout  : {hard_timeout}s/case")
    print(f"  Resume        : {args.resume}")
    print(f"  Base instance : {base_inst}  (ports {5760+10*base_inst}"
          f"–{5760+10*(base_inst+max_workers-1)})")
    print(f"  Binary        : {BINARY}")
    print(f"  Defaults      : {params_file}"
          f"{'  (--params)' if args.params else '  (auto)'}")
    print(f"  Config        : {args.config}  ({len(config['params'])} params)")
    print("  Profile       : GROUND→CLIMB→CRUISE→APPROACH→FLARE→LAND  (all-FBWA, no loiter)")
    print("=" * 70)

    if not BINARY.exists():
        print(f"\nERROR: Binary not found: {BINARY}\n       Run  ./waf plane  first.")
        sys.exit(1)

    if args.output_dir:
        output_dir = os.path.abspath(args.output_dir)
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = str(SCRIPT_DIR / f"mc_{ts}")     # e.g. monte_carlo/mc_20260618_162027/
    os.makedirs(output_dir, exist_ok=True)
    print(f"\n  Output → {output_dir}\n")

    # Per-case independent, standalone-reproducible RNG: case `cid` is keyed by
    # (seed, cid), so any single case reproduces via default_rng([seed, cid]).
    all_perturbed = {cid: sample_parameters(config["params"],
                                            np.random.default_rng([args.seed, cid]))
                     for cid in range(num_runs)}

    # Resume: skip cases that already finished (have a result.json).
    done = set()
    if args.resume:
        for rj in glob.glob(os.path.join(output_dir, "case_*", "result.json")):
            try:
                with open(rj) as f:
                    done.add(int(json.load(f)["case_id"]))
            except Exception:
                pass
        print(f"  RESUME: {len(done)} cases already complete — skipping them\n")

    manager = multiprocessing.Manager()
    inst_q  = manager.Queue()
    for w in range(max_workers):
        inst_q.put(base_inst + w)

    workspace = str(WORKSPACE)
    tasks = [(cid, f"{args.seed}:{cid}", config, all_perturbed[cid], output_dir,
              workspace, args.speedup, args.plot, hard_timeout, params_file)
             for cid in range(num_runs) if cid not in done]
    print(f"  Cases to run this invocation: {len(tasks)}\n")

    results = []
    t_global = time.time()
    with ProcessPoolExecutor(max_workers=max_workers,
                             initializer=_init_worker,
                             initargs=(inst_q,)) as pool:
        futures = {pool.submit(_worker_entry, t): t[0] for t in tasks}
        for fut in as_completed(futures):
            cid = futures[fut]
            try:
                res = fut.result()
                results.append(res)
                tk = "TKOFF" if res["takeoff_success"] else "NO-TKOFF"
                ms = "LAND" if res["mission_complete"] else res["land_result"]
                _log(cid, f"{tk} | {ms} | {res['duration_s']:.0f}s | "
                          f"alt {res['max_alt_m']:.0f}m")
            except Exception as exc:
                _log(cid, f"WORKER EXCEPTION: {exc}")
                results.append(dict(
                    case_id=cid, takeoff_success=False, mission_complete=False,
                    duration_s=0, max_alt_m=0, land_result="worker_exception",
                    exit_reason=f"worker_exception: {exc}"))

    elapsed = time.time() - t_global
    summary_path = os.path.join(output_dir, "summary.csv")
    # Rebuild summary from ALL result.json (this run + any resumed/prior cases).
    all_results = write_summary_csv(summary_path, output_dir,
                                    list(config["params"].keys()))

    n_tot = len(all_results)
    n_tk = sum(1 for r in all_results if r.get("takeoff_success"))
    n_ms = sum(1 for r in all_results if r.get("mission_complete"))
    print("\n" + "=" * 70)
    print("  SUMMARY — uSTOL ALL-FBWA + LANDING CAMPAIGN (no loiter)")
    print("=" * 70)
    print(f"  Cases completed  : {n_tot}  (this invocation ran {len(tasks)})")
    print(f"  Takeoff success  : {n_tk}/{n_tot}  "
          f"({100*n_tk/max(n_tot,1):.1f}%)")
    print(f"  Landed (touchdown): {n_ms}/{n_tot}  "
          f"({100*n_ms/max(n_tot,1):.1f}%)")
    print(f"  Wall-clock time  : {elapsed:.0f}s  ({elapsed/60:.1f} min)")
    print(f"  Summary CSV      : {summary_path}")

    # Completion marker — a simple text file in the campaign folder saying it's done.
    done_path = os.path.join(output_dir, "DONE.txt")
    with open(done_path, "w") as f:
        f.write("Monte Carlo campaign complete.\n")
        f.write(f"Finished        : {datetime.now().isoformat()}\n")
        f.write(f"Cases completed : {n_tot}\n")
        f.write(f"Takeoff success : {n_tk}/{n_tot}\n")
        f.write(f"Landed (td)     : {n_ms}/{n_tot}\n")
        f.write(f"Wall-clock time : {elapsed:.0f}s\n")
        f.write(f"Summary CSV     : {summary_path}\n")
    print(f"  Done marker      : {done_path}")
    print("=" * 70)


if __name__ == "__main__":
    main()
