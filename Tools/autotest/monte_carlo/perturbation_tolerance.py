#!/usr/bin/env python3
"""
Perturbation Tolerance Finder
==============================
For each of N important parameters, binary-search the maximum single-parameter
perturbation (in both +/- directions) at which the vehicle still passes ALL
pass/fail flags.  All other parameters held at nominal.

Produces a CSV and console table of {param, direction, max_pass_pct, first_fail_pct, flags_at_fail}.

Usage:
    python perturbation_tolerance.py                      # 11 params, 10 workers
    python perturbation_tolerance.py --workers 5           # fewer workers
    python perturbation_tolerance.py --pct-step 5          # coarser initial grid
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
import subprocess
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
#  pymavlink
# ---------------------------------------------------------------------------
try:
    from pymavlink import mavutil, mavwp
except ImportError:
    print("ERROR: pymavlink required.  pip install pymavlink")
    sys.exit(1)

# ---------------------------------------------------------------------------
#  Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR    = Path(__file__).resolve().parent
AUTOTEST_DIR  = SCRIPT_DIR.parent
WORKSPACE     = AUTOTEST_DIR.parent.parent
BINARY        = WORKSPACE / "build" / "sitl" / "bin" / "arduplane"
DEFAULTS_PARM = WORKSPACE / "Latest.parm"
FALLBACK_PARM = AUTOTEST_DIR / "models" / "plane.parm"
LOGS_SIM_DIR  = WORKSPACE / "Logs_Simulations"

# ---------------------------------------------------------------------------
#  Mission / flight constants  (matching MC runner exactly)
# ---------------------------------------------------------------------------
HOME_LAT     = 28.559741
HOME_LON     = 77.117450
HOME_ALT_M   = 237
RWY_HDG      = 285
TAKEOFF_ALT  = 100        # NAV_TAKEOFF target  (m AGL)
TAKEOFF_DET  = 15         # takeoff detect  (m AGL)
GUIDED_DIST  = 400        # guided offset   (m)
LOITER_SW    = 100        # switch-to-LOITER alt (m AGL)
MISSION_TO   = 90         # wall-clock timeout (s)

# ---------------------------------------------------------------------------
#  All 30 parameter nominals  (from monte_carlo_config.json)
# ---------------------------------------------------------------------------
ALL_PARAMS = {
    "Ixx":          14.658,
    "Iyy":          16.944,
    "Izz":          27.412,
    "rho":          1.15,
    "mass":         65.0,
    "cg_x":         1.03,
    "cg_z":         0.043,
    "CL_0":         0.18,
    "CL_delta_e":   0.69,
    "CL_alpha":     5.718,
    "CL_q":         12.5,
    "CD_0":         0.0763,
    "CD_delta_e":   0.012,
    "CD_delta_f":   0.23,
    "CD_delta_e2":  0.004,
    "CD_alpha":     0.3,
    "Cl_0":        -0.000117,
    "Cl_beta":     -0.000171887339,
    "Cl_delta_r":   0.008307888027,
    "Cl_delta_aL":  0.062337808096,
    "Cl_delta_aR": -0.062853470112,
    "Cl_r":         0.08,
    "Cm_0":         0.249992,
    "Cm_alpha":    -4.205166440623,
    "Cm_delta_e":  -3.303273575514,
    "Cm_Cmu":       0.199495,
    "Cm_q":        -77.0,
    "Cm_delta_f":   0.163407563134,
    "Cm_beta2":    -3.722702399213,
    "max_thrust":   57.63375,
}

# ---------------------------------------------------------------------------
#  11 focus parameters to sweep
# ---------------------------------------------------------------------------
FOCUS_PARAMS = [
    "Ixx",
    "cg_x",
    "CL_0",
    "CL_delta_e",
    "CL_alpha",
    "Cl_delta_aL",
    "Cl_delta_aR",
    "Cm_0",
    "Cm_alpha",
    "Cm_delta_e",
    "Cm_q",
]

# ---------------------------------------------------------------------------
#  Pass/fail thresholds  (must match mc_postprocess.m exactly)
# ---------------------------------------------------------------------------
LIM_LIFTOFF_ALT      = 0.11      # m AGL — liftoff gate
LIM_ALT_ERROR        = 10.0      # m — |mean(alt_SS) - 100|
LIM_ROLL_RATE        = 150.0     # deg/s
LIM_PITCH_RATE       = 150.0     # deg/s
LIM_ROLL_ANGLE       = 35.0      # deg (SS only)
LIM_PITCH_UP         = 25.0      # deg
LIM_PITCH_DN         = -25.0     # deg
LIM_CIRCLE_RADIUS_PCT = 20.0     # %
LIM_STALL_AOA        = 30.0      # deg
LIM_TAS_LOW          = 24.0      # m/s (SS, movmean-filtered)
LIM_TAS_HIGH         = 40.0      # m/s (SS, movmean-filtered)
CMD_RADIUS           = 200.0     # m — commanded loiter radius

R2D = 180.0 / math.pi

# ===================================================================
#  Override file I/O
# ===================================================================
def write_override_file(filepath: str, params: dict):
    with open(filepath, "w") as f:
        for k, v in params.items():
            f.write(f"{k}={v:.10f}\n")


# ===================================================================
#  Mission construction  (identical to MC runner)
# ===================================================================
def build_mission():
    wp = mavwp.MAVWPLoader()
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1, 0, 0, 0, 0,
        HOME_LAT, HOME_LON, HOME_ALT_M))
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 1, 5, 0, 0, 0,
        HOME_LAT, HOME_LON, TAKEOFF_ALT))
    return wp


# ===================================================================
#  MAVLink helpers  (copied from MC runner for independence)
# ===================================================================
def compute_offset_point(lat, lon, heading_deg, dist_m):
    bearing_rad = np.radians(heading_deg)
    dlat = dist_m * np.cos(bearing_rad) / 111320.0
    dlon = dist_m * np.sin(bearing_rad) / (111320.0 * np.cos(np.radians(lat)))
    return float(lat + dlat), float(lon + dlon)


def send_guided_target(conn, lat, lon, alt_rel_m):
    conn.mav.set_position_target_global_int_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), float(alt_rel_m),
        0, 0, 0, 0, 0, 0, 0, 0)


def _wait_for_tcp_port(host, port, timeout=90):
    """Block until the TCP port accepts a connection (SITL is listening)."""
    import socket as _socket
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            s = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
            s.settimeout(2)
            s.connect((host, port))
            s.close()
            return True
        except (_socket.timeout, ConnectionRefusedError, OSError):
            pass
        finally:
            try:
                s.close()
            except Exception:
                pass
        time.sleep(1.0)
    return False


def connect_mavlink(port, retries=40, delay=2.0):
    conn_str = f"tcp:127.0.0.1:{port}"

    # Wait for the TCP port to be open before pymavlink tries (avoids EOF spam)
    if not _wait_for_tcp_port("127.0.0.1", port, timeout=80):
        raise RuntimeError(f"SITL never opened TCP port {port}")

    for attempt in range(retries):
        try:
            conn = mavutil.mavlink_connection(conn_str, source_system=255)
            # Silence pymavlink's noisy "EOF on TCP socket" prints
            conn.handle_eof = lambda *a, **kw: conn.reconnect()
            conn.handle_disconnect = lambda *a, **kw: conn.reconnect()
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            if hb is not None:
                return conn
            conn.close()
        except Exception:
            pass
        time.sleep(delay)
    raise RuntimeError(f"Could not connect to {conn_str}")


def upload_mission(conn, wp_loader):
    conn.waypoint_clear_all_send()
    time.sleep(0.5)
    conn.waypoint_count_send(wp_loader.count())
    for _ in range(wp_loader.count()):
        msg = conn.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
            blocking=True, timeout=10)
        if msg is None:
            raise RuntimeError("Timeout waiting for MISSION_REQUEST")
        conn.mav.send(wp_loader.wp(msg.seq))
    ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError("Mission rejected")


def request_data_streams(conn):
    for sid in [mavutil.mavlink.MAV_DATA_STREAM_ALL]:
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component, sid, 10, 1)
    time.sleep(0.5)


def set_param(conn, name, value, retries=3):
    for _ in range(retries):
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode('utf-8'), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if ack and ack.param_id.replace('\x00', '') == name:
            return True
    return False


def set_mode(conn, mode_name, timeout=15.0):
    mode_map = conn.mode_mapping()
    if mode_name not in mode_map:
        raise ValueError(f"Unknown mode '{mode_name}'")
    mode_id = mode_map[mode_name]
    for _ in range(3):
        conn.set_mode(mode_id)
        time.sleep(0.3)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            return True
        conn.set_mode(mode_id)
    return False


def arm_vehicle(conn, timeout=60.0):
    t0 = time.time()
    attempt = 0
    while time.time() - t0 < timeout:
        attempt += 1
        p2 = 0 if attempt <= 4 else 21196
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, p2, 0, 0, 0, 0, 0)
        for _ in range(5):
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        time.sleep(2.0)
    return False


def _wait_ekf_ready(conn, timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        if msg is not None and (msg.flags & 0x1F) == 0x1F:
            return


def _kill_proc(proc):
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
        proc.wait(timeout=10)
    except Exception:
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass


def resolve_defaults_parm():
    if DEFAULTS_PARM.exists():
        return str(DEFAULTS_PARM)
    if FALLBACK_PARM.exists():
        return str(FALLBACK_PARM)
    raise FileNotFoundError("Cannot find defaults .parm file")


# ===================================================================
#  Post-processing evaluator  (Python port of mc_postprocess.m logic)
# ===================================================================
def movmean(arr, k):
    """Simple centred moving-average with edge handling."""
    n = len(arr)
    out = np.empty(n)
    half = k // 2
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = np.mean(arr[lo:hi])
    return out


def kasa_circle_fit(x, y):
    """Kåsa algebraic circle fit.  Returns (cx, cy, R_mean, R_std)."""
    n = len(x)
    if n < 20:
        return 0, 0, 0, 999
    A = np.column_stack([x, y, np.ones(n)])
    b = x**2 + y**2
    c, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    cx = c[0] / 2.0
    cy = c[1] / 2.0
    R_sq = c[2] + cx**2 + cy**2
    if R_sq < 0:
        return cx, cy, 0, 999
    radii = np.sqrt((x - cx)**2 + (y - cy)**2)
    return cx, cy, float(np.mean(radii)), float(np.std(radii))


def evaluate_case(case_dir: str) -> dict:
    """
    Read sim_output CSV and state_log CSV, compute all pass/fail flags.
    Returns dict with flag names (bool) and scalar metrics.
    """
    flags = {
        "flag_no_liftoff": True,
        "flag_alt_error": False,
        "flag_roll_rate": False,
        "flag_pitch_rate": False,
        "flag_roll_angle": False,
        "flag_pitch_angle": False,
        "flag_circle_radius": False,
        "flag_stall_aoa": False,
        "flag_airspeed_low": False,
        "flag_airspeed_high": False,
    }
    metrics = {}

    # ---- Find sim_output CSV ----
    sim_files = glob.glob(os.path.join(case_dir, "sim_output_*.csv"))
    if not sim_files:
        return {"passed": False, "flags": flags, "metrics": metrics,
                "fail_reasons": "NO_SIM_OUTPUT"}

    # ---- Load sim_output ----
    try:
        sim = np.genfromtxt(sim_files[0], delimiter=',', skip_header=1)
    except Exception:
        return {"passed": False, "flags": flags, "metrics": metrics,
                "fail_reasons": "BAD_SIM_OUTPUT"}

    if sim.ndim != 2 or sim.shape[0] < 10:
        return {"passed": False, "flags": flags, "metrics": metrics,
                "fail_reasons": "SHORT_SIM"}

    # Column indices (0-based)
    COL_TIME   = 0
    COL_TAS    = 2
    COL_ALT    = 3
    COL_PHI    = 9
    COL_THETA  = 10
    COL_P      = 12
    COL_Q      = 13
    COL_VBX    = 15   # V_b_tas_0
    COL_VBZ    = 17   # V_b_tas_2

    alt   = sim[:, COL_ALT]
    tas   = sim[:, COL_TAS]
    phi   = sim[:, COL_PHI] * R2D   # rad → deg
    theta = sim[:, COL_THETA] * R2D
    p_dps = sim[:, COL_P] * R2D
    q_dps = sim[:, COL_Q] * R2D
    vbx   = sim[:, COL_VBX]
    vbz   = sim[:, COL_VBZ]

    # ---- Liftoff detection ----
    liftoff_idx = -1
    for i in range(len(alt)):
        if alt[i] > LIM_LIFTOFF_ALT:
            liftoff_idx = i
            break

    if liftoff_idx < 0:
        return {"passed": False, "flags": flags, "metrics": metrics,
                "fail_reasons": "NO_LIFTOFF"}

    flags["flag_no_liftoff"] = False

    # ---- Post-liftoff slices ----
    alt_pl   = alt[liftoff_idx:]
    phi_pl   = phi[liftoff_idx:]
    theta_pl = theta[liftoff_idx:]
    p_pl     = p_dps[liftoff_idx:]
    q_pl     = q_dps[liftoff_idx:]
    tas_pl   = tas[liftoff_idx:]
    vbx_pl   = vbx[liftoff_idx:]
    vbz_pl   = vbz[liftoff_idx:]

    N = len(alt_pl)

    # ---- Steady-state window ----
    ss_start = max(1, int(round(N * 0.60)))
    alt_ss   = alt_pl[ss_start:]
    phi_ss   = phi_pl[ss_start:]
    tas_ss   = tas_pl[ss_start:]

    # ---- AoA (full post-liftoff) ----
    aoa_rad = np.arctan2(vbz_pl, np.maximum(vbx_pl, 0.1))
    aoa_deg = aoa_rad * R2D

    # ---- Flag: alt error ----
    if len(alt_ss) > 0:
        alt_ss_mean = float(np.mean(alt_ss))
        alt_err = abs(alt_ss_mean - 100.0)
        flags["flag_alt_error"] = alt_err > LIM_ALT_ERROR
        metrics["alt_ss_mean"] = alt_ss_mean
        metrics["alt_ss_error"] = alt_err

    # ---- Flag: roll rate ----
    if len(p_pl) > 0:
        max_p = float(np.max(np.abs(p_pl)))
        flags["flag_roll_rate"] = max_p > LIM_ROLL_RATE
        metrics["max_roll_rate"] = max_p

    # ---- Flag: pitch rate ----
    if len(q_pl) > 0:
        max_q = float(np.max(np.abs(q_pl)))
        flags["flag_pitch_rate"] = max_q > LIM_PITCH_RATE
        metrics["max_pitch_rate"] = max_q

    # ---- Flag: roll angle (SS) ----
    if len(phi_ss) > 0:
        max_phi_ss = float(np.max(np.abs(phi_ss)))
        flags["flag_roll_angle"] = max_phi_ss > LIM_ROLL_ANGLE
        metrics["max_roll_ss"] = max_phi_ss

    # ---- Flag: pitch angle (full flight) ----
    if len(theta_pl) > 0:
        max_theta = float(np.max(theta_pl))
        min_theta = float(np.min(theta_pl))
        flags["flag_pitch_angle"] = (max_theta > LIM_PITCH_UP or
                                     min_theta < LIM_PITCH_DN)
        metrics["max_pitch"] = max_theta
        metrics["min_pitch"] = min_theta

    # ---- Flag: stall AoA ----
    if len(aoa_deg) > 0:
        max_aoa = float(np.max(aoa_deg))
        flags["flag_stall_aoa"] = max_aoa > LIM_STALL_AOA
        metrics["max_aoa"] = max_aoa

    # ---- Flag: airspeed (SS, movmean filtered) ----
    if len(tas_ss) > 5:
        tas_valid = tas_ss[np.isfinite(tas_ss) & (tas_ss > 0.5)]
        if len(tas_valid) > 5:
            tas_filt = movmean(tas_valid, 10)
            max_tas = float(np.max(tas_filt))
            min_tas = float(np.min(tas_filt))
            flags["flag_airspeed_low"] = min_tas < LIM_TAS_LOW
            flags["flag_airspeed_high"] = max_tas > LIM_TAS_HIGH
            metrics["max_tas_ss"] = max_tas
            metrics["min_tas_ss"] = min_tas

    # ---- Flag: circle radius (from state_log) ----
    state_files = glob.glob(os.path.join(case_dir, "*state_log*.csv"))
    if state_files:
        try:
            sl = np.genfromtxt(state_files[0], delimiter=',', skip_header=1)
            if sl.ndim == 2 and sl.shape[0] > 30:
                lx = sl[:, 15]   # local_x_m  (0-based col 15)
                ly = sl[:, 16]   # local_y_m

                # SS window on state_log too
                sl_N = sl.shape[0]
                sl_ss = max(1, int(round(sl_N * 0.60)))
                lx_ss = lx[sl_ss:]
                ly_ss = ly[sl_ss:]

                if len(lx_ss) >= 20:
                    _, _, r_mean, _ = kasa_circle_fit(lx_ss, ly_ss)
                    r_err_pct = abs(r_mean - CMD_RADIUS) / CMD_RADIUS * 100.0
                    flags["flag_circle_radius"] = r_err_pct > LIM_CIRCLE_RADIUS_PCT
                    metrics["circle_radius"] = r_mean
                    metrics["circle_err_pct"] = r_err_pct
        except Exception:
            pass

    # ---- Aggregate ----
    any_fail = any(flags.values())
    fail_reasons = ", ".join(k for k, v in flags.items() if v)

    return {
        "passed": not any_fail,
        "flags": flags,
        "metrics": metrics,
        "fail_reasons": fail_reasons if any_fail else "",
    }


# ===================================================================
#  Run a single SITL case  (one parameter perturbed, rest nominal)
# ===================================================================
def run_single_case(param_name: str, pct: float, direction: int,
                    instance_id: int, output_dir: str) -> dict:
    """
    Run one SITL case with `param_name` perturbed by `pct`% in `direction`.
    direction: +1 or -1.

    Returns dict with case metadata + pass/fail evaluation.
    """
    tag  = f"{param_name}_{'+' if direction > 0 else '-'}{abs(pct):.1f}pct"
    case_dir   = os.path.join(output_dir, tag)
    worker_dir = os.path.join(output_dir, f"worker_{instance_id}")
    os.makedirs(case_dir, exist_ok=True)
    os.makedirs(worker_dir, exist_ok=True)

    # Build override with nominal everything + this one perturbed
    override = dict(ALL_PARAMS)   # copy nominals
    nominal  = override[param_name]
    perturbed_val = nominal * (1.0 + direction * pct / 100.0)
    override[param_name] = perturbed_val

    override_file = os.path.join(case_dir, "overrides.txt")
    write_override_file(override_file, override)

    parm = resolve_defaults_parm()
    home_str = f"{HOME_LAT},{HOME_LON},{HOME_ALT_M},{RWY_HDG}"
    cmd = [
        str(BINARY), "-w", "--model", "plane",
        "--defaults", parm,
        "--home", home_str,
        "--sim-address", "127.0.0.1",
        f"-I{instance_id}",
    ]

    env = os.environ.copy()
    env["LAT_MC_OVERRIDE_FILE"] = override_file
    env["DISPLAY"] = ""

    tcp_port = 5760 + 10 * instance_id
    sitl_proc = None
    conn = None
    t_start = time.time()

    result = {
        "param": param_name,
        "direction": direction,
        "pct": pct,
        "perturbed_val": perturbed_val,
        "nominal_val": nominal,
        "passed": False,
        "fail_reasons": "",
        "tag": tag,
    }

    try:
        # Launch SITL
        sitl_proc = subprocess.Popen(
            cmd, cwd=worker_dir, env=env,
            stdout=open(os.path.join(case_dir, "sitl_stdout.log"), "w"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid)

        _log(tag, f"SITL PID {sitl_proc.pid}  inst {instance_id}  TCP {tcp_port}")

        # Give SITL a moment to initialise before hammering the port
        time.sleep(3.0)

        conn = connect_mavlink(tcp_port, retries=30, delay=2.0)
        request_data_streams(conn)
        set_param(conn, 'ARMING_CHECK', 0)
        set_param(conn, 'TKOFF_THR_MAX', 100)
        set_param(conn, 'TKOFF_THR_DELAY', 0)
        set_param(conn, 'THR_MAX', 100)
        set_param(conn, 'TKOFF_FLAP_PCNT', 50)

        _wait_ekf_ready(conn, timeout=90)
        time.sleep(3.0)

        # Upload mission
        wp = build_mission()
        upload_mission(conn, wp)

        # Arm in FBWA → AUTO
        if not set_mode(conn, "FBWA"):
            set_mode(conn, "MANUAL")
        time.sleep(1.0)

        if not arm_vehicle(conn):
            result["fail_reasons"] = "arm_failed"
            return result

        if not set_mode(conn, "AUTO"):
            result["fail_reasons"] = "auto_mode_failed"
            return result

        _log(tag, "armed + AUTO — flying")

        # ---- Open state_log CSV ----
        state_csv_path = os.path.join(case_dir, f"{tag}_state_log.csv")
        state_fp = open(state_csv_path, "w", newline="")
        state_writer = csv.writer(state_fp)
        state_writer.writerow([
            "wall_time_s", "ardu_mode",
            "lat_degE7", "lon_degE7", "alt_msl_mm", "alt_agl_mm",
            "vx_cmps", "vy_cmps", "vz_cmps",
            "roll_rad", "pitch_rad", "yaw_rad",
            "rollspeed_rps", "pitchspeed_rps", "yawspeed_rps",
            "local_x_m", "local_y_m", "local_z_m",
            "local_vx_mps", "local_vy_mps", "local_vz_mps",
            "airspeed_mps", "groundspeed_mps", "heading_deg",
            "throttle_pct", "climb_mps",
            "xacc_mg", "yacc_mg", "zacc_mg",
            "xgyro_mradps", "ygyro_mradps", "zgyro_mradps",
            "nav_roll_deg", "nav_pitch_deg", "alt_error_m",
            "aspd_error_mps", "xtrack_error_m",
        ])

        st = {
            "mode": -1,
            "lat": 0, "lon": 0, "alt": 0, "alt_rel": 0,
            "vx": 0, "vy": 0, "vz": 0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "p": 0.0, "q": 0.0, "r": 0.0,
            "lx": 0.0, "ly": 0.0, "lz": 0.0,
            "lvx": 0.0, "lvy": 0.0, "lvz": 0.0,
            "airspeed": 0.0, "groundspeed": 0.0, "heading": 0,
            "throttle": 0, "climb": 0.0,
            "xacc": 0, "yacc": 0, "zacc": 0,
            "xgyro": 0, "ygyro": 0, "zgyro": 0,
            "nav_roll": 0.0, "nav_pitch": 0.0, "alt_error": 0.0,
            "aspd_error": 0.0, "xtrack_error": 0.0,
        }

        # ---- Flight monitor ----
        max_alt = 0.0
        takeoff = False
        guided_phase = False
        loiter_phase = False

        tgt_lat, tgt_lon = compute_offset_point(HOME_LAT, HOME_LON,
                                                RWY_HDG, GUIDED_DIST)

        while (time.time() - t_start) < MISSION_TO:
            if sitl_proc.poll() is not None:
                result["fail_reasons"] = "sitl_crashed"
                break

            msg = conn.recv_match(
                type=["GLOBAL_POSITION_INT", "HEARTBEAT",
                      "ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD",
                      "SCALED_IMU", "NAV_CONTROLLER_OUTPUT"],
                blocking=True, timeout=2)
            if msg is None:
                continue

            mtype = msg.get_type()

            if mtype == "ATTITUDE":
                st["roll"]  = msg.roll
                st["pitch"] = msg.pitch
                st["yaw"]   = msg.yaw
                st["p"]     = msg.rollspeed
                st["q"]     = msg.pitchspeed
                st["r"]     = msg.yawspeed
                state_writer.writerow([
                    f"{time.time() - t_start:.3f}",
                    st["mode"],
                    st["lat"], st["lon"], st["alt"], st["alt_rel"],
                    st["vx"], st["vy"], st["vz"],
                    f"{st['roll']:.5f}", f"{st['pitch']:.5f}", f"{st['yaw']:.5f}",
                    f"{st['p']:.5f}", f"{st['q']:.5f}", f"{st['r']:.5f}",
                    f"{st['lx']:.3f}", f"{st['ly']:.3f}", f"{st['lz']:.3f}",
                    f"{st['lvx']:.3f}", f"{st['lvy']:.3f}", f"{st['lvz']:.3f}",
                    f"{st['airspeed']:.2f}", f"{st['groundspeed']:.2f}",
                    st["heading"],
                    st["throttle"], f"{st['climb']:.2f}",
                    st["xacc"], st["yacc"], st["zacc"],
                    st["xgyro"], st["ygyro"], st["zgyro"],
                    f"{st['nav_roll']:.2f}", f"{st['nav_pitch']:.2f}",
                    f"{st['alt_error']:.2f}",
                    f"{st['aspd_error']:.2f}", f"{st['xtrack_error']:.2f}",
                ])
                continue

            if mtype == "LOCAL_POSITION_NED":
                st["lx"] = msg.x; st["ly"] = msg.y; st["lz"] = msg.z
                st["lvx"] = msg.vx; st["lvy"] = msg.vy; st["lvz"] = msg.vz
                continue

            if mtype == "VFR_HUD":
                st["airspeed"] = msg.airspeed
                st["groundspeed"] = msg.groundspeed
                st["heading"] = msg.heading
                st["throttle"] = msg.throttle
                st["climb"] = msg.climb
                continue

            if mtype == "SCALED_IMU":
                st["xacc"] = msg.xacc; st["yacc"] = msg.yacc; st["zacc"] = msg.zacc
                st["xgyro"] = msg.xgyro; st["ygyro"] = msg.ygyro; st["zgyro"] = msg.zgyro
                continue

            if mtype == "NAV_CONTROLLER_OUTPUT":
                st["nav_roll"] = msg.nav_roll; st["nav_pitch"] = msg.nav_pitch
                st["alt_error"] = msg.alt_error; st["aspd_error"] = msg.aspd_error
                st["xtrack_error"] = msg.xtrack_error
                continue

            if mtype == "GLOBAL_POSITION_INT":
                st["lat"] = msg.lat; st["lon"] = msg.lon
                st["alt"] = msg.alt; st["alt_rel"] = msg.relative_alt
                st["vx"] = msg.vx; st["vy"] = msg.vy; st["vz"] = msg.vz
                alt_agl = msg.relative_alt / 1000.0
                max_alt = max(max_alt, alt_agl)

                if alt_agl > TAKEOFF_DET and not takeoff:
                    takeoff = True
                    guided_phase = True
                    set_mode(conn, "GUIDED")
                    time.sleep(0.3)
                    send_guided_target(conn, tgt_lat, tgt_lon, TAKEOFF_ALT)
                    _log(tag, f"takeoff @ {alt_agl:.1f}m → GUIDED")

                if guided_phase and not loiter_phase and alt_agl > LOITER_SW:
                    loiter_phase = True
                    guided_phase = False
                    set_mode(conn, "LOITER")
                    _log(tag, f"LOITER @ {alt_agl:.1f}m")

            elif mtype == "HEARTBEAT":
                st["mode"] = msg.custom_mode

        # Close state log
        try:
            state_fp.close()
        except Exception:
            pass

    except Exception as exc:
        result["fail_reasons"] = f"exception: {exc}"
        _log(tag, f"ERROR: {exc}")
        traceback.print_exc()

    finally:
        if conn:
            try:
                conn.close()
            except Exception:
                pass
        if sitl_proc:
            sitl_pid = sitl_proc.pid
            _kill_proc(sitl_proc)

            # Move sim_output CSV into case_dir
            pattern = os.path.join(str(LOGS_SIM_DIR),
                                   f"sim_output_*_pid{sitl_pid}.csv")
            for csv_file in glob.glob(pattern):
                dst = os.path.join(case_dir, os.path.basename(csv_file))
                try:
                    shutil.move(csv_file, dst)
                except Exception:
                    pass

            # Delete BIN logs
            bin_dir = os.path.join(worker_dir, "logs")
            if os.path.isdir(bin_dir):
                try:
                    shutil.rmtree(bin_dir)
                except Exception:
                    pass

    # ---- Post-process & evaluate ----
    eval_result = evaluate_case(case_dir)
    result["passed"]       = eval_result["passed"]
    result["fail_reasons"] = eval_result["fail_reasons"]
    result["flags"]        = eval_result["flags"]
    result["metrics"]      = eval_result["metrics"]

    _log(tag, f"{'PASS' if result['passed'] else 'FAIL'}"
         f"  [{result['fail_reasons']}]")

    return result


def _log(tag: str, msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"  [{ts}] {tag}: {msg}", flush=True)


# ===================================================================
#  Instance-pool helpers  (same pattern as MC runner)
# ===================================================================
_instance_queue = None


def _init_worker(q):
    global _instance_queue
    _instance_queue = q


def _pool_run_case(args):
    """Unpack args and grab an instance ID from the shared queue."""
    global _instance_queue
    param_name, pct, direction, output_dir = args
    instance_id = _instance_queue.get()
    try:
        return run_single_case(param_name, pct, direction,
                               instance_id, output_dir)
    finally:
        _instance_queue.put(instance_id)


# ===================================================================
#  Run a batch of jobs in parallel
# ===================================================================
def _run_job_batch(jobs, output_dir, inst_queue, max_workers):
    """Run a list of (param, pct, direction) jobs and return results."""
    batch_results = []
    with ProcessPoolExecutor(
            max_workers=max_workers,
            initializer=_init_worker,
            initargs=(inst_queue,)) as pool:
        futures = {}
        for j in jobs:
            p, pct, d = j
            f = pool.submit(_pool_run_case, (p, pct, d, output_dir))
            futures[f] = j

        for f in as_completed(futures):
            try:
                r = f.result()
                batch_results.append(r)
            except Exception as exc:
                j = futures[f]
                print(f"  JOB EXCEPTION: {j}: {exc}", flush=True)
                batch_results.append({
                    "param": j[0], "pct": j[1], "direction": j[2],
                    "passed": False, "fail_reasons": str(exc),
                })
    return batch_results


# ===================================================================
#  MAIN
# ===================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Find max single-parameter perturbation tolerance")
    parser.add_argument("--workers", type=int, default=10,
                        help="Max parallel SITL instances (default: 10)")
    parser.add_argument("--base-inst", type=int, default=200,
                        help="Base SITL instance ID (default: 200, avoids MC ports)")
    parser.add_argument("--coarse-pts", type=str, default="20,40,60,80,100",
                        help="Comma-separated coarse grid %% values "
                             "(default: 20,40,60,80,100)")
    parser.add_argument("--refine-step", type=float, default=10.0,
                        help="Refinement step in %% (default: 10)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output directory (auto-generated if omitted)")
    args = parser.parse_args()

    coarse_pts = sorted(set(float(x) for x in args.coarse_pts.split(",")))

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output or os.path.join(
        str(SCRIPT_DIR), f"perturbation_sweep_{ts}")
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 75)
    print("  PERTURBATION TOLERANCE SWEEP  (2-phase: coarse → refine)")
    print("=" * 75)
    print(f"  Parameters   : {len(FOCUS_PARAMS)}")
    print(f"  Directions   : +/-")
    print(f"  Coarse grid  : {coarse_pts}")
    print(f"  Refine step  : {args.refine_step}%")
    print(f"  Workers      : {args.workers}")
    print(f"  Base inst    : {args.base_inst}")
    print(f"  Output       : {output_dir}")
    print("=" * 75)

    # Create instance queue
    mgr = multiprocessing.Manager()
    inst_queue = mgr.Queue()
    for i in range(args.workers):
        inst_queue.put(args.base_inst + i)

    results = []
    t_wall_start = time.time()

    # ================================================================
    #  PHASE 1 — Coarse grid (all params × both directions × grid pts)
    # ================================================================
    coarse_jobs = []
    for p in FOCUS_PARAMS:
        for d in [+1, -1]:
            for pct in coarse_pts:
                coarse_jobs.append((p, round(pct, 1), d))

    print(f"\n  Phase 1: {len(coarse_jobs)} coarse-grid cases")

    coarse_results = _run_job_batch(coarse_jobs, output_dir, inst_queue,
                                     args.workers)
    results.extend(coarse_results)

    ph1_elapsed = time.time() - t_wall_start
    print(f"  Phase 1 done in {ph1_elapsed:.0f}s ({ph1_elapsed/60:.1f} min)")

    # ================================================================
    #  PHASE 2 — Refine around transition boundary
    # ================================================================
    refine_jobs = []
    for param in FOCUS_PARAMS:
        for d in [+1, -1]:
            cases = sorted(
                [r for r in results
                 if r["param"] == param and r["direction"] == d],
                key=lambda x: x["pct"])

            # Find transition: last PASS pct and first FAIL pct
            last_pass = 0.0
            first_fail = None
            for c in cases:
                if c["passed"]:
                    last_pass = max(last_pass, c["pct"])
                else:
                    if first_fail is None or c["pct"] < first_fail:
                        first_fail = c["pct"]

            if first_fail is None:
                # Everything passed — nothing to refine (tolerance > max grid)
                continue
            if last_pass <= 0:
                # Everything failed — refine low end [0, first_fail]
                lo, hi = 0.0, first_fail
            else:
                lo, hi = last_pass, first_fail

            # Generate refinement points between lo and hi (exclusive of
            # points we already ran)
            already = set(c["pct"] for c in cases)
            step = args.refine_step
            pct = lo + step
            while pct < hi - 0.01:
                rpct = round(pct, 1)
                if rpct not in already:
                    refine_jobs.append((param, rpct, d))
                pct += step

    if refine_jobs:
        print(f"\n  Phase 2: {len(refine_jobs)} refinement cases")
        refine_results = _run_job_batch(refine_jobs, output_dir, inst_queue,
                                         args.workers)
        results.extend(refine_results)

    wall_elapsed = time.time() - t_wall_start
    print(f"\n  All {len(results)} cases complete in {wall_elapsed:.0f}s "
          f"({wall_elapsed/60:.1f} min)")

    # ---- Analyse results: find threshold per param/direction ----
    print("\n" + "=" * 90)
    print(f"  {'Parameter':<16} {'Dir':>4} {'Max Pass%':>10} "
          f"{'First Fail%':>12} {'Flags at First Fail':<40}")
    print("-" * 90)

    summary_rows = []

    for param in FOCUS_PARAMS:
        for d in [+1, -1]:
            # Get all results for this param+direction, sorted by pct
            cases = sorted(
                [r for r in results
                 if r["param"] == param and r["direction"] == d],
                key=lambda x: x["pct"])

            max_pass_pct  = 0.0
            first_fail_pct = None
            fail_flags = ""

            for c in cases:
                if c["passed"]:
                    max_pass_pct = max(max_pass_pct, c["pct"])
                else:
                    if first_fail_pct is None or c["pct"] < first_fail_pct:
                        first_fail_pct = c["pct"]
                        fail_flags = c.get("fail_reasons", "")

            dir_str = "+" if d > 0 else "-"
            ff_str = f"{first_fail_pct:.1f}" if first_fail_pct else ">100"

            print(f"  {param:<16} {dir_str:>4} {max_pass_pct:>9.1f}% "
                  f"{ff_str:>11}% {fail_flags:<40}")

            summary_rows.append({
                "parameter": param,
                "direction": dir_str,
                "max_pass_pct": max_pass_pct,
                "first_fail_pct": first_fail_pct if first_fail_pct else -1,
                "fail_flags": fail_flags,
            })

    print("=" * 90)

    # ---- Write CSV ----
    csv_path = os.path.join(output_dir, "perturbation_tolerance_summary.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "parameter", "direction", "max_pass_pct",
            "first_fail_pct", "fail_flags"])
        w.writeheader()
        w.writerows(summary_rows)
    print(f"\n  Summary CSV: {csv_path}")

    # ---- Also write detailed per-case CSV ----
    detail_path = os.path.join(output_dir, "perturbation_sweep_detail.csv")
    with open(detail_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["parameter", "direction", "pct", "nominal_val",
                     "perturbed_val", "passed", "fail_reasons"])
        for r in sorted(results, key=lambda x: (x["param"],
                                                 x["direction"],
                                                 x["pct"])):
            w.writerow([
                r.get("param", ""),
                "+" if r.get("direction", 0) > 0 else "-",
                r.get("pct", 0),
                r.get("nominal_val", 0),
                r.get("perturbed_val", 0),
                r.get("passed", False),
                r.get("fail_reasons", ""),
            ])
    print(f"  Detail CSV : {detail_path}")
    print()


if __name__ == "__main__":
    main()
