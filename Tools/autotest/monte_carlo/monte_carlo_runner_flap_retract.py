#!/usr/bin/env python3
"""
Monte Carlo Runner — FLAP RETRACTION VARIANT
=============================================
Identical to monte_carlo_runner.py EXCEPT:
  • Flaps stay at 20° (50 %) during takeoff AND the GUIDED climb.
  • At 100 m AGL (LOITER switch) flaps smoothly retract to 0°.

Mechanism
---------
  FLAP_2_SPEED  = 45 m/s   →  keeps 50 % flap whenever IAS < 45 m/s
  FLAP_2_PERCNT = 50        →  50 % of 0–40° = 20°
  FLAP_SLEWRATE = 10        →  10 %/s  → 5-second retraction

At LOITER entry the runner zeroes FLAP_2_SPEED/FLAP_2_PERCNT via
param_set.  ArduPlane then drives auto_flap_percent → 0 %, rate-limited
by FLAP_SLEWRATE.

IMPORTANT:  The FDM must read actual SERVO12 output (input.servos[11])
instead of the hardcoded 1500 PWM.  Build with the un-hardcoded
LAT_SIM_ardu_in_2_lat.cpp before using this runner.

Usage:
    python monte_carlo_runner_flap_retract.py --runs 10 --workers 10
"""

import argparse
import csv
import glob
import json
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
#  pymavlink imports
# ---------------------------------------------------------------------------
try:
    from pymavlink import mavutil, mavwp
except ImportError:
    print("ERROR: pymavlink is required.  Install with:  pip install pymavlink")
    sys.exit(1)

# ---------------------------------------------------------------------------
#  Paths (derived from this script's location)
# ---------------------------------------------------------------------------
SCRIPT_DIR   = Path(__file__).resolve().parent            # monte_carlo/
AUTOTEST_DIR = SCRIPT_DIR.parent                          # Tools/autotest/
WORKSPACE    = AUTOTEST_DIR.parent.parent                 # repo root
BINARY       = WORKSPACE / "build" / "sitl" / "bin" / "arduplane"
DEFAULT_CFG  = SCRIPT_DIR / "monte_carlo_config_ustol_v1.json"
DEFAULTS_PARM = WORKSPACE / "Latest.parm"                    # user's tuned params
FALLBACK_PARM = AUTOTEST_DIR / "models" / "plane.parm"


# ===================================================================
#  Configuration helpers
# ===================================================================
def load_config(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def resolve_defaults_parm() -> str:
    """Return path to the ArduPilot parameter defaults file."""
    if DEFAULTS_PARM.exists():
        return str(DEFAULTS_PARM)
    if FALLBACK_PARM.exists():
        return str(FALLBACK_PARM)
    raise FileNotFoundError("Cannot find mav.parm or models/plane.parm")


# ===================================================================
#  Parameter sampling
# ===================================================================
def sample_parameters(params_cfg: dict, rng: np.random.Generator) -> dict:
    """Return dict of {param_name: perturbed_value}."""
    perturbed = {}
    for name, cfg in params_cfg.items():
        nom   = cfg["nominal"]
        s3    = cfg["sigma_3"]
        sigma = s3 / 3.0
        delta = rng.normal(0.0, sigma)
        delta = float(np.clip(delta, -s3, s3))          # clip to ±3σ
        perturbed[name] = nom + delta
    return perturbed


# ===================================================================
#  Override-file I/O
# ===================================================================
def write_override_file(filepath: str, params: dict):
    """Write key=value text file consumed by LAT_SIM_MonteCarlo.cpp."""
    with open(filepath, "w") as f:
        for k, v in params.items():
            f.write(f"{k}={v:.10f}\n")


def write_case_report(filepath: str, case_id: int,
                      perturbed: dict, nominals: dict, result: dict):
    """Write human-readable per-case report."""
    with open(filepath, "w") as f:
        f.write(f"Monte Carlo Case #{case_id:04d}  [FLAP RETRACT]\n")
        f.write("=" * 65 + "\n\n")
        f.write(f"Timestamp : {datetime.now().isoformat()}\n")
        f.write(f"Duration  : {result.get('duration_s', 0):.1f} s\n\n")

        f.write(f"{'Parameter':<20} {'Nominal':>14} {'Perturbed':>14} {'Delta%':>9}\n")
        f.write("-" * 65 + "\n")
        for name in perturbed:
            nom  = nominals[name]
            pert = perturbed[name]
            pct  = ((pert - nom) / abs(nom) * 100) if abs(nom) > 1e-12 else 0.0
            f.write(f"{name:<20} {nom:>14.6f} {pert:>14.6f} {pct:>+8.2f}%\n")

        f.write("\n" + "=" * 65 + "\n")
        f.write("Result\n")
        f.write("-" * 65 + "\n")
        f.write(f"  Takeoff Success   : {result.get('takeoff_success', False)}\n")
        f.write(f"  Mission Complete  : {result.get('mission_complete', False)}\n")
        f.write(f"  Max Altitude (m)  : {result.get('max_alt_m', 0):.1f}\n")
        f.write(f"  Flap Retracted    : {result.get('flap_retracted', False)}\n")
        f.write(f"  Exit Reason       : {result.get('exit_reason', 'unknown')}\n")


# ===================================================================
#  Mission construction
# ===================================================================
def build_mission(home_lat, home_lon, home_alt, takeoff_alt):
    """
    Build MAVLink mission items:
      WP0  – Home
      WP1  – NAV_TAKEOFF
    """
    wp = mavwp.MAVWPLoader()

    # WP 0 – Home
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1,
        0, 0, 0, 0,
        home_lat, home_lon, home_alt))

    # WP 1 – Takeoff
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 1,
        5,           # min pitch (deg)
        0, 0, 0,
        home_lat, home_lon, takeoff_alt))

    return wp


# ===================================================================
#  Guided-point & loiter helpers
# ===================================================================
def compute_offset_point(lat: float, lon: float,
                         heading_deg: float, dist_m: float):
    """Return (lat2, lon2) offset by dist_m along heading_deg from (lat, lon)."""
    bearing_rad = np.radians(heading_deg)
    dlat = dist_m * np.cos(bearing_rad) / 111320.0
    dlon = dist_m * np.sin(bearing_rad) / (111320.0 * np.cos(np.radians(lat)))
    return float(lat + dlat), float(lon + dlon)


def send_guided_target(conn, lat: float, lon: float, alt_rel_m: float):
    """Send a GUIDED-mode position target (lat/lon degrees, alt AGL m)."""
    conn.mav.set_position_target_global_int_send(
        0,                                                     # time_boot_ms
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,                                    # type_mask: pos only
        int(lat * 1e7), int(lon * 1e7), float(alt_rel_m),     # lat, lon, alt
        0, 0, 0,                                               # vx, vy, vz
        0, 0, 0,                                               # afx, afy, afz
        0, 0)                                                  # yaw, yaw_rate


# ===================================================================
#  MAVLink helpers
# ===================================================================
def connect_mavlink(port: int, retries: int = 30, delay: float = 2.0):
    """Try to connect to SITL TCP port with retries."""
    conn_str = f"tcp:127.0.0.1:{port}"
    for attempt in range(retries):
        try:
            conn = mavutil.mavlink_connection(conn_str, source_system=255)
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            if hb is not None:
                return conn
            conn.close()
        except Exception:
            pass
        time.sleep(delay)
    raise RuntimeError(f"Could not connect to {conn_str} after {retries} attempts")


def upload_mission(conn, wp_loader):
    """Upload complete mission."""
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
    if ack is None:
        raise RuntimeError("No MISSION_ACK received")
    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"Mission rejected (type={ack.type})")


def request_data_streams(conn):
    """Ask SITL to send telemetry at a reasonable rate."""
    for stream_id in [mavutil.mavlink.MAV_DATA_STREAM_ALL]:
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            stream_id, 10, 1)          # 10 Hz, enable
    time.sleep(0.5)


def set_param(conn, name: str, value: float, retries: int = 3):
    """Set a single parameter on the autopilot."""
    for _ in range(retries):
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode('utf-8'), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if ack and ack.param_id.replace('\x00','') == name:
            return True
    return False


def wait_prearm_ok(conn, timeout: float = 90):
    """Wait until SYS_STATUS reports no pre-arm errors."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        if msg is not None:
            pass
        txt = conn.recv_match(type='STATUSTEXT', blocking=False)
        if txt:
            text_str = txt.text if isinstance(txt.text, str) else txt.text.decode('utf-8', errors='ignore')
            if 'ready' in text_str.lower() or 'prearm' not in text_str.lower():
                pass
        time.sleep(0.5)


def set_mode(conn, mode_name: str, timeout: float = 15.0) -> bool:
    """Set flight mode and wait for confirmation."""
    mode_map = conn.mode_mapping()
    if mode_name not in mode_map:
        raise ValueError(f"Unknown mode '{mode_name}'; available: {list(mode_map)}")
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


def arm_vehicle(conn, timeout: float = 60.0) -> bool:
    """Arm vehicle with retries, including force-arm fallback."""
    t0 = time.time()
    attempt = 0
    while time.time() - t0 < timeout:
        attempt += 1
        if attempt <= 4:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
        else:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)

        for _ in range(5):
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        time.sleep(2.0)
    return False


# ===================================================================
#  Single-case worker
# ===================================================================
def _run_case(case_id: int, instance_id: int, config: dict,
              perturbed: dict, output_dir: str, workspace: str) -> dict:
    """
    Execute one Monte Carlo case with flap retraction at LOITER entry.
    """
    result = dict(
        case_id=case_id, takeoff_success=False, mission_complete=False,
        duration_s=0.0, max_alt_m=0.0, flap_retracted=False,
        exit_reason="unknown")

    mission_cfg = config["mission"]
    sitl_cfg    = config.get("sitl", {})

    # ---- Directories ----
    case_dir   = os.path.join(output_dir, f"case_{case_id:04d}")
    worker_dir = os.path.join(output_dir, f"worker_{instance_id}")
    os.makedirs(case_dir, exist_ok=True)
    os.makedirs(worker_dir, exist_ok=True)

    # ---- Override file ----
    override_file = os.path.join(case_dir, "overrides.txt")
    write_override_file(override_file, perturbed)

    # ---- Resolve binary & defaults ----
    binary   = os.path.join(workspace, "build", "sitl", "bin", "arduplane")
    parm     = resolve_defaults_parm()
    rwy_hdg  = mission_cfg.get("runway_heading_deg", 285)
    home_str = (f"{mission_cfg['home_lat']},{mission_cfg['home_lon']},"
                f"{mission_cfg['home_alt_m']},{rwy_hdg}")

    cmd = [
        binary,
        "-w",                           # wipe EEPROM
        "--model", "plane",
        "--defaults", parm,
        "--home", home_str,
        "--sim-address", "127.0.0.1",
        f"-I{instance_id}",
    ]

    # ---- Optional GCS output ----
    gcs_out = sitl_cfg.get("gcs_output", "")
    if gcs_out:
        cmd += ["--serial2", gcs_out]

    # ---- Environment ----
    env = os.environ.copy()
    env["LAT_MC_OVERRIDE_FILE"] = override_file
    env["DISPLAY"] = ""

    tcp_port  = 5760 + 10 * instance_id
    sitl_proc = None
    conn      = None
    t_start   = time.time()

    try:
        # ---- Launch SITL ----
        sitl_proc = subprocess.Popen(
            cmd,
            cwd=worker_dir,
            env=env,
            stdout=open(os.path.join(case_dir, "sitl_stdout.log"), "w"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        _log(case_id, f"SITL PID {sitl_proc.pid}  instance {instance_id}  "
                       f"TCP {tcp_port}")

        # ---- Connect ----
        conn = connect_mavlink(tcp_port, retries=40, delay=2.0)
        _log(case_id, "connected")

        request_data_streams(conn)

        # ---- Set parameters ----
        set_param(conn, 'ARMING_CHECK', 0)
        _log(case_id, "ARMING_CHECK disabled")

        set_param(conn, 'TKOFF_THR_MAX', 100)
        set_param(conn, 'TKOFF_THR_DELAY', 0)
        set_param(conn, 'THR_MAX', 100)

        # Takeoff flap: 50% of 0–40° = 20°
        set_param(conn, 'TKOFF_FLAP_PCNT', 50)

        # ──────────────────────────────────────────────────────────
        #  FLAP RETRACTION SETUP
        # ──────────────────────────────────────────────────────────
        #  FLAP_2_SPEED  = 45 m/s  →  hold 50% flap whenever IAS < 45
        #  FLAP_2_PERCNT = 50      →  50% of 0–40° = 20°
        #  FLAP_SLEWRATE = 10      →  10 %/s → 5 s retraction
        #
        #  During GUIDED climb (IAS ≈ 30 m/s < 45): flap stays 50%.
        #  At LOITER entry (100 m) we zero FLAP_2_SPEED/PERCNT →
        #  auto_flap → 0%, rate-limited by FLAP_SLEWRATE → smooth.
        # ──────────────────────────────────────────────────────────
        set_param(conn, 'FLAP_2_SPEED',  45)
        set_param(conn, 'FLAP_2_PERCNT', 50)
        set_param(conn, 'FLAP_SLEWRATE', 10)
        _log(case_id, "FLAP params set: TKOFF=50%, FLAP_2_SPEED=45, "
                       "FLAP_2_PERCNT=50, SLEWRATE=10")

        # Wait for EKF / GPS
        _wait_ekf_ready(conn, timeout=90)
        _log(case_id, "EKF ready")

        time.sleep(3.0)

        # ---- Upload mission ----
        wp = build_mission(
            mission_cfg["home_lat"], mission_cfg["home_lon"],
            mission_cfg["home_alt_m"],
            mission_cfg["takeoff_alt_m"])
        upload_mission(conn, wp)
        _log(case_id, "mission uploaded")

        # ---- Arm in FBWA, then AUTO ----
        if not set_mode(conn, "FBWA"):
            _log(case_id, "FBWA mode failed, trying MANUAL")
            if not set_mode(conn, "MANUAL"):
                result["exit_reason"] = "mode_set_failed"
                return result
        _log(case_id, f"mode set")
        time.sleep(1.0)

        if not arm_vehicle(conn):
            result["exit_reason"] = "arm_failed"
            return result
        _log(case_id, "armed")

        if not set_mode(conn, "AUTO"):
            result["exit_reason"] = "auto_mode_failed"
            return result
        _log(case_id, "AUTO mode set — mission running  (flaps 20°)")

        # ---- Open servo-commands CSV ----
        servo_csv_path = os.path.join(case_dir, f"case_{case_id:04d}_servo_cmds.csv")
        servo_fp = open(servo_csv_path, "w", newline="")
        servo_writer = csv.writer(servo_fp)
        servo_writer.writerow([
            "wall_time_s",
            "servo1", "servo2", "servo3", "servo4",
            "servo5", "servo6", "servo7", "servo8",
            "servo9", "servo10", "servo11", "servo12",
            "servo13", "servo14", "servo15", "servo16",
        ])

        # ---- Open state-log CSV ----
        state_csv_path = os.path.join(case_dir, f"case_{case_id:04d}_state_log.csv")
        state_fp = open(state_csv_path, "w", newline="")
        state_writer = csv.writer(state_fp)
        state_writer.writerow([
            "wall_time_s",
            "ardu_mode",
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

        # Running state dict
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

        # ---- Monitor ----
        timeout_s    = mission_cfg.get("mission_timeout_s", 600)
        thr_alt      = mission_cfg.get("takeoff_detect_alt_m", 2)
        loiter_alt   = mission_cfg.get("takeoff_alt_m", 100)
        guided_dist  = mission_cfg.get("guided_offset_m", 500)
        loiter_sw_alt = mission_cfg.get("loiter_switch_alt_m", 80)
        max_alt      = 0.0
        takeoff      = False
        guided_phase = False
        loiter_phase = False

        while (time.time() - t_start) < timeout_s:
            if sitl_proc.poll() is not None:
                result["exit_reason"] = f"sitl_crashed (rc={sitl_proc.returncode})"
                break

            msg = conn.recv_match(
                type=["GLOBAL_POSITION_INT", "HEARTBEAT",
                      "SERVO_OUTPUT_RAW", "ATTITUDE",
                      "LOCAL_POSITION_NED", "VFR_HUD",
                      "SCALED_IMU", "NAV_CONTROLLER_OUTPUT"],
                blocking=True, timeout=2)
            if msg is None:
                continue

            mtype = msg.get_type()

            if mtype == "SERVO_OUTPUT_RAW":
                servo_writer.writerow([
                    f"{time.time() - t_start:.3f}",
                    msg.servo1_raw, msg.servo2_raw,
                    msg.servo3_raw, msg.servo4_raw,
                    msg.servo5_raw, msg.servo6_raw,
                    msg.servo7_raw, msg.servo8_raw,
                    msg.servo9_raw, msg.servo10_raw,
                    msg.servo11_raw, msg.servo12_raw,
                    msg.servo13_raw, msg.servo14_raw,
                    msg.servo15_raw, msg.servo16_raw,
                ])
                continue

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
                st["lx"]  = msg.x
                st["ly"]  = msg.y
                st["lz"]  = msg.z
                st["lvx"] = msg.vx
                st["lvy"] = msg.vy
                st["lvz"] = msg.vz
                continue

            if mtype == "VFR_HUD":
                st["airspeed"]    = msg.airspeed
                st["groundspeed"] = msg.groundspeed
                st["heading"]     = msg.heading
                st["throttle"]    = msg.throttle
                st["climb"]       = msg.climb
                continue

            if mtype == "SCALED_IMU":
                st["xacc"]  = msg.xacc
                st["yacc"]  = msg.yacc
                st["zacc"]  = msg.zacc
                st["xgyro"] = msg.xgyro
                st["ygyro"] = msg.ygyro
                st["zgyro"] = msg.zgyro
                continue

            if mtype == "NAV_CONTROLLER_OUTPUT":
                st["nav_roll"]    = msg.nav_roll
                st["nav_pitch"]   = msg.nav_pitch
                st["alt_error"]   = msg.alt_error
                st["aspd_error"]  = msg.aspd_error
                st["xtrack_error"] = msg.xtrack_error
                continue

            if mtype == "GLOBAL_POSITION_INT":
                st["lat"]     = msg.lat
                st["lon"]     = msg.lon
                st["alt"]     = msg.alt
                st["alt_rel"] = msg.relative_alt
                st["vx"]      = msg.vx
                st["vy"]      = msg.vy
                st["vz"]      = msg.vz

                alt_agl = msg.relative_alt / 1000.0   # mm → m
                if alt_agl > max_alt:
                    max_alt = alt_agl

                # ---- Takeoff detection → GUIDED ----
                if alt_agl > thr_alt and not takeoff:
                    takeoff = True
                    guided_phase = True
                    result["takeoff_success"] = True

                    tgt_lat, tgt_lon = compute_offset_point(
                        mission_cfg["home_lat"], mission_cfg["home_lon"],
                        rwy_hdg, guided_dist)

                    _log(case_id, f"takeoff detected @ {alt_agl:.1f} m AGL "
                                  f"→ GUIDED (flaps still 20° via FLAP_2_SPEED)")

                    set_mode(conn, "GUIDED")
                    time.sleep(0.3)
                    send_guided_target(conn, tgt_lat, tgt_lon, loiter_alt)
                    _log(case_id, "GUIDED mode — climbing with 20° flap")

                # ---- LOITER at target altitude → RETRACT FLAPS ----
                if guided_phase and not loiter_phase and alt_agl > loiter_sw_alt:
                    loiter_phase = True
                    guided_phase = False
                    set_mode(conn, "LOITER")
                    _log(case_id, f"LOITER mode @ {alt_agl:.1f}m AGL")

                    # ── TRIGGER FLAP RETRACTION ──
                    # Zero out FLAP_2_SPEED/PERCNT → auto_flap → 0%
                    # FLAP_SLEWRATE=10 gives smooth 5-second retraction
                    set_param(conn, 'FLAP_2_SPEED',  0)
                    set_param(conn, 'FLAP_2_PERCNT', 0)
                    result["flap_retracted"] = True
                    _log(case_id, "FLAP RETRACTION triggered (FLAP_2→0, "
                                  "SLEWRATE=10 → 5s retract 20°→0°)")

            elif mtype == "HEARTBEAT":
                st["mode"] = msg.custom_mode
        else:
            if takeoff:
                result["mission_complete"] = True
                result["exit_reason"] = "timeout_loiter_complete"
                _log(case_id, "timeout reached — loiter mission complete")
            else:
                result["exit_reason"] = "timeout_no_takeoff"

        result["max_alt_m"]  = max_alt
        result["duration_s"] = time.time() - t_start

    except Exception as exc:
        result["exit_reason"] = f"exception: {exc}"
        result["duration_s"] = time.time() - t_start
        _log(case_id, f"ERROR: {exc}")
        traceback.print_exc()

    finally:
        try:
            servo_fp.close()
        except Exception:
            pass
        try:
            state_fp.close()
        except Exception:
            pass

        if conn:
            try:
                conn.close()
            except Exception:
                pass
        if sitl_proc:
            sitl_pid = sitl_proc.pid
            _kill_proc(sitl_proc)

            # Move SITL Output.cpp CSV into case dir
            log_dir = os.path.join(workspace, "Logs_Simulations")
            pattern = os.path.join(log_dir, f"sim_output_*_pid{sitl_pid}.csv")
            for csv_file in glob.glob(pattern):
                dst = os.path.join(case_dir, os.path.basename(csv_file))
                try:
                    shutil.move(csv_file, dst)
                    _log(case_id, f"moved {os.path.basename(csv_file)} → case dir")
                except Exception as e:
                    _log(case_id, f"WARNING: could not move CSV: {e}")

            # Delete .BIN dataflash logs
            bin_log_dir = os.path.join(worker_dir, "logs")
            if os.path.isdir(bin_log_dir):
                try:
                    shutil.rmtree(bin_log_dir)
                    _log(case_id, f"deleted {bin_log_dir}  (BIN logs)")
                except Exception as e:
                    _log(case_id, f"WARNING: could not delete BIN logs: {e}")

        # Per-case report
        nominals = {k: v["nominal"] for k, v in config["params"].items()}
        report   = os.path.join(case_dir, f"case_{case_id:04d}_report.txt")
        write_case_report(report, case_id, perturbed, nominals, result)

    return result


def _wait_ekf_ready(conn, timeout: float = 60):
    """Block until EKF reports healthy (or timeout)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        if msg is not None:
            if (msg.flags & 0x1F) == 0x1F:
                return


def _kill_proc(proc):
    """Kill a subprocess and its entire process group."""
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


def _log(case_id: int, msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"  [{ts}] Case {case_id:04d}: {msg}", flush=True)


# ===================================================================
#  Instance-pool wrapper
# ===================================================================
_instance_queue = None


def _init_worker(q):
    global _instance_queue
    _instance_queue = q


def _worker_entry(args):
    """Top-level function called by the executor."""
    case_id, config, perturbed, output_dir, workspace = args
    inst = _instance_queue.get()
    try:
        return _run_case(case_id, inst, config, perturbed, output_dir, workspace)
    finally:
        _instance_queue.put(inst)


# ===================================================================
#  Summary output
# ===================================================================
def write_summary_csv(path: str, results: list, all_perturbed: dict,
                      config: dict):
    """Write a single CSV summarizing all cases."""
    param_names = list(config["params"].keys())
    fieldnames = (["case_id", "takeoff_success", "mission_complete",
                   "max_alt_m", "duration_s", "flap_retracted", "exit_reason"]
                  + [f"p_{n}" for n in param_names])

    rows = []
    for r in sorted(results, key=lambda x: x["case_id"]):
        cid  = r["case_id"]
        pert = all_perturbed[cid]
        row  = {
            "case_id": cid,
            "takeoff_success":  r.get("takeoff_success", False),
            "mission_complete": r.get("mission_complete", False),
            "max_alt_m":        f"{r.get('max_alt_m', 0):.2f}",
            "duration_s":       f"{r.get('duration_s', 0):.1f}",
            "flap_retracted":   r.get("flap_retracted", False),
            "exit_reason":      r.get("exit_reason", ""),
        }
        for n in param_names:
            row[f"p_{n}"] = f"{pert[n]:.8f}"
        rows.append(row)

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


# ===================================================================
#  Main entry point
# ===================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Monte Carlo Runner — FLAP RETRACTION VARIANT")
    parser.add_argument("--config", default=str(DEFAULT_CFG),
                        help="Path to monte_carlo_config_ustol_v1.json")
    parser.add_argument("--runs", type=int, default=None,
                        help="Override number of runs")
    parser.add_argument("--workers", type=int, default=None,
                        help="Override max parallel workers")
    parser.add_argument("--seed", type=int, default=42,
                        help="RNG seed for reproducibility")
    parser.add_argument("--base-instance", type=int, default=100,
                        help="Starting SITL instance ID (ports = 5760+10*id)")
    args = parser.parse_args()

    config     = load_config(args.config)
    num_runs   = args.runs    or config.get("num_runs", 100)
    max_workers = args.workers or config.get("max_workers", 4)
    base_inst  = args.base_instance

    print("=" * 65)
    print("  LAT Monte Carlo Runner — FLAP RETRACTION VARIANT")
    print("=" * 65)
    print(f"  Runs           : {num_runs}")
    print(f"  Workers        : {max_workers}")
    print(f"  Seed           : {args.seed}")
    print(f"  Base instance  : {base_inst}  (ports {5760+10*base_inst}"
          f"–{5760+10*(base_inst+max_workers-1)})")
    print(f"  Binary         : {BINARY}")
    print(f"  Defaults       : {resolve_defaults_parm()}")
    print(f"  Config         : {args.config}")
    print()
    print("  FLAP SCHEDULE:")
    print("    Takeoff → 15 m :  TKOFF_FLAP_PCNT = 50%  (20°)")
    print("    15 m → 100 m   :  FLAP_2_SPEED=45, PERCNT=50  (20°)")
    print("    100 m (LOITER) :  FLAP_2 zeroed → retract 0°")
    print("    Retraction rate:  FLAP_SLEWRATE = 10 %/s  (5 s)")
    print("=" * 65)

    if not BINARY.exists():
        print(f"\nERROR: Binary not found: {BINARY}")
        print("       Run  ./waf plane  first to build.")
        sys.exit(1)

    # ---- Output directory ----
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = str(SCRIPT_DIR / f"mc_results_flap_retract_{ts}")
    os.makedirs(output_dir, exist_ok=True)
    print(f"\n  Output → {output_dir}\n")

    # ---- Sample all cases ----
    rng = np.random.default_rng(args.seed)
    all_perturbed = {}
    for cid in range(num_runs):
        all_perturbed[cid] = sample_parameters(config["params"], rng)

    # ---- Instance-ID pool ----
    manager = multiprocessing.Manager()
    inst_q  = manager.Queue()
    for w in range(max_workers):
        inst_q.put(base_inst + w)

    # ---- Build task list ----
    workspace = str(WORKSPACE)
    tasks = [(cid, config, all_perturbed[cid], output_dir, workspace)
             for cid in range(num_runs)]

    # ---- Run ----
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
                status = "PASS" if res["takeoff_success"] else "FAIL"
                flap   = "FLAP_OK" if res.get("flap_retracted") else "NO_FLAP"
                _log(cid, f"{status}  |  {flap}  |  {res['exit_reason']}  "
                          f"|  {res['duration_s']:.0f}s  "
                          f"|  alt {res['max_alt_m']:.0f}m")
            except Exception as exc:
                _log(cid, f"WORKER EXCEPTION: {exc}")
                results.append(dict(
                    case_id=cid, takeoff_success=False,
                    mission_complete=False, max_alt_m=0,
                    duration_s=0, flap_retracted=False,
                    exit_reason=f"worker_exception: {exc}"))

    elapsed = time.time() - t_global

    # ---- Summary ----
    summary_path = os.path.join(output_dir, "summary.csv")
    write_summary_csv(summary_path, results, all_perturbed, config)

    n_pass = sum(1 for r in results if r["takeoff_success"])
    n_full = sum(1 for r in results if r["mission_complete"])
    n_flap = sum(1 for r in results if r.get("flap_retracted"))
    print("\n" + "=" * 65)
    print("  SUMMARY — FLAP RETRACTION CAMPAIGN")
    print("=" * 65)
    print(f"  Total cases       : {len(results)}")
    print(f"  Takeoff success   : {n_pass}/{len(results)}  "
          f"({100*n_pass/max(len(results),1):.1f}%)")
    print(f"  Mission complete  : {n_full}/{len(results)}  "
          f"({100*n_full/max(len(results),1):.1f}%)")
    print(f"  Flap retracted    : {n_flap}/{len(results)}  "
          f"({100*n_flap/max(len(results),1):.1f}%)")
    print(f"  Wall-clock time   : {elapsed:.0f}s  ({elapsed/60:.1f} min)")
    print(f"  Summary CSV       : {summary_path}")
    print("=" * 65)


if __name__ == "__main__":
    main()
