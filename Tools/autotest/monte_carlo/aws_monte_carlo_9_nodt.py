#!/usr/bin/env python3
"""
Monte Carlo Runner — uSTOL CRUISE ROLL/YAW DOUBLET STABILITY TEST (no-DT) + WIND
==================================================================================
The Monte Carlo driver for the mission_ustol_10_cruise_nodt.py profile: instead of
the takeoff->cruise->loiter->landing mission of aws_monte_carlo_8.py, each case flies
an open-loop takeoff/climb to cruise and then a CRUISE DYNAMIC-STABILITY test — a
series of ROLL doublets followed by YAW (rudder) doublets — then settles back to
wings-level cruise and ends (the aircraft is left airborne; the worker kills its SITL
afterward). There is NO loiter and NO landing.

NO DIFFERENTIAL THRUST by default ("nodt"): --diff-thrust off (default) leaves the
nine ESCs on k_throttle and forces UST_ENABLE=0, so yaw is handled the stock way
(rudder surface). This is the BASELINE twin — run it against the DT variant
(--diff-thrust on) and diff the stability metrics (roll overshoot/recovery, peak yaw
rate, yaw recovery) to see what the AP_DiffThrust mixer buys you in turns. With DT off
the motor1/motor9 split stays ~0 (max_motor_diff_pwm is a sanity signal).

Per case the runner measures, from a SETTLE-gated clean start so steps don't
cross-contaminate:
  • ROLL doublets (+10/-10/+20/-20 deg bank): worst |held-cmd|, worst overshoot,
    worst recovery time back to wings-level.
  • YAW  doublets (+10/-10/+20/-20 deg rudder): peak yaw rate, worst recovery time.
  • departures: if the aircraft spirals past BANK_ABORT it runs a wings-level
    recovery and the case is flagged departed/aborted.
  • whether it re-settled to cruise at the end, and the cruise altitude band held.

A SINGLE campaign still flies a MIX of wind conditions (head/tail/cross/up/down and
combined slants) exactly like aws_monte_carlo_7/8 — edit WIND_MIX below. The wind
type/speed/direction is recorded per case in result.json/summary.csv.

REQUIRES the SIM_Plane.cpp wiring that copies wind_ef -> vehcle.wind_ned: the custom
uSTOL FDM ignores SIM_WIND_* without it.  Rebuild with `./waf plane`.

SELF-CONTAINED raw-pymavlink RC-override flight logic (no auto_flight4), headless,
N parallel SITL processes (one TCP port each), optional --speedup, Parquet sim_output
+ lean per-case output (same storage/throughput options as aws_monte_carlo_8.py).

Phases written to the per-case CSV: GROUND, CLIMB, CRUISE (settle), ROLL, YAW,
RECOVER (after a departure), RETURN (final settle).

Usage:
    # 1) edit WIND_MIX below (case + count per wind type)
    # 2) run a campaign (output dir auto-tagged mc_<ts>_mix):
    python3 aws_monte_carlo_9_nodt.py                      # no-DT baseline (default)
    python3 aws_monte_carlo_9_nodt.py --diff-thrust on     # DT variant for comparison
    python3 aws_monte_carlo_9_nodt.py --workers 30 --speedup 10
    python3 aws_monte_carlo_9_nodt.py --runs 25000 --workers 30  # rescale mix, keep ratios
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

# ===========================================================================
#  WIND MIX — EDIT THIS BEFORE RUNNING (no CLI flag)
# ===========================================================================
#  Unlike aws_monte_carlo_6 (ONE wind type for the whole campaign), v7 flies a
#  MIX of wind types in a single campaign: you give an explicit case COUNT per
#  wind type, so a 25 000-case run can be, e.g., 4000 crosswind + 4000 updraft +
#  4000 downdraft + ...  Each block sweeps its own speed levels so every type is
#  balanced across wind strengths.  Directions are relative to the runway heading
#  (config mission.runway_heading_deg) — the aircraft flies straight along it the
#  whole profile, so head/tail/cross are well-defined.
#
#  WIND_MIX is an ORDERED list of blocks.  Per block:
#     "case"   (required) one of:
#                  "head"|"tail"|"cross"|"up"|"down"|"updown"|"none"
#                  COMBINED "<horiz>+<vert>" — horiz in head|tail|cross, vert in
#                  up|down|updown — e.g. "cross+down" flies a crosswind AND a
#                  downdraft at once (a slanted wind).
#     "count"  (required) number of cases of this type
#     "speeds" (optional) m/s HORIZONTAL speed levels swept within the block
#                         (default WIND_SPEED_LEVELS); ignored for "none"/pure-vert
#     "side"   (optional) "right"|"left" for "cross" (default CROSSWIND_SIDE)
#     "vert_speed" (optional, COMBINED only) vertical component magnitude (m/s),
#                  scalar or list swept in lockstep with "speeds"
#                  (default WIND_VERT_SPEED).  SITL builds one wind vector from
#                  SIM_WIND_SPD at elevation SIM_WIND_DIR_Z, so a combined case is
#                  SPD=hypot(h,v) at elevation atan2(v,h): horiz=h along the horiz
#                  direction, vert=v (up -> rising air, down -> sinking air).
#  The campaign size = sum of counts.  (--runs, if given, proportionally rescales
#  every block to that total while preserving the ratios — handy to scale a mix
#  up/down without re-typing counts.)  Add as many blocks as you like, including
#  several "cross"/combined blocks with different sides/speeds.
WIND_MIX = [
    {"case": "none",       "count": 2000},
    {"case": "head",       "count": 500},
    {"case": "tail",       "count": 500},
    {"case": "cross",      "count": 500, "side": "right"},
    {"case": "cross",      "count": 0, "side": "left"},
    {"case": "up",         "count": 500},
    {"case": "down",       "count": 500},
    # combined horizontal + vertical (vertical magnitude = vert_speed m/s):
    {"case": "cross+down", "count": 500, "side": "right", "vert_speed": 1.0},
    {"case": "head+down",  "count": 500, "vert_speed": 1.0},
    {"case": "tail+up",    "count": 500, "vert_speed": 1.0},
]
WIND_SPEED_LEVELS     = [1.0, 1.5, 2.0] # default m/s HORIZ levels; block case i -> levels[i % len]
WIND_VERT_SPEED       = 1.0             # default vertical component (m/s) for combined cases
                                        # (e.g. "cross+down"); per-block override via "vert_speed"
CROSSWIND_SIDE        = "right"         # default cross side: "right" (hdg+90) | "left" (hdg-90)
WIND_MIX_SHUFFLE      = True            # True -> deterministically (seeded) spread types across
                                        # case ids so ANY prefix of the run is a representative
                                        # sample of the whole mix; False -> blocks in WIND_MIX order
WIND_TURBULENCE       = 0.2             # m/s RMS gusts (0 = steady wind)
WIND_PROFILE_CONSTANT = True            # True  -> SIM_WIND_T=1 (constant w/ altitude)
                                        # False -> SIM_WIND_T=0 (sqrt profile: less near ground)
# ===========================================================================


def wind_params_for_case(case: str, runway_hdg: float, speed: float,
                         side: str = "right", vert_speed: float = 0.0) -> dict:
    """Map a wind case name + speed(s) to the SITL SIM_WIND_* params (+ metadata).

    Convention (verified against SIM_Aircraft.cpp update_wind, where the wind
    vector is built as
        wind_ef = (cos(DIR)cos(DIR_Z), sin(DIR)cos(DIR_Z), sin(DIR_Z)) * SPD,
    then sign-flipped at line 990, with wind_ned := wind_ef so
    V_air = V_gnd - wind_ned):
      • SIM_WIND_DIR is the heading the wind comes FROM (compass deg).
      • DIR = runway heading  -> headwind (raises airspeed, shortens ground roll).
      • SIM_WIND_DIR_Z is the ELEVATION angle: horiz = SPD*cos(DIR_Z),
        vert = SPD*sin(DIR_Z).  DIR_Z = +90 -> updraft (rising air, reduces sink);
        DIR_Z = -90 -> downdraft (sinking air, increases sink).
    For pure-vertical cases the horizontal component is cos(90)=0, so the whole
    SIM_WIND_SPD magnitude goes vertical.

    COMBINED cases "<horiz>+<vert>" (e.g. "cross+down") set BOTH a horizontal wind
    of magnitude `speed` along the horiz direction AND a vertical wind of magnitude
    `vert_speed` (up -> +, down -> -).  Because SITL has a single SPD at elevation
    DIR_Z, the two combine as SPD = hypot(speed, vert_speed) at elevation
    atan2(±vert_speed, speed) — exactly recovering horiz=speed and vert=vert_speed.
    """
    case = case.lower()
    horiz_dir = {
        "head":  runway_hdg % 360.0,
        "tail":  (runway_hdg + 180.0) % 360.0,
        "cross": (runway_hdg + (90.0 if side.lower() == "right" else -90.0)) % 360.0,
    }
    if case == "none" or (speed <= 0.0 and vert_speed <= 0.0):
        spd, direction, dir_z = 0.0, 0.0, 0.0
    elif case in horiz_dir:                  # pure horizontal (head/tail/cross)
        spd, direction, dir_z = speed, horiz_dir[case], 0.0
    elif case == "up":                       # pure updraft — rising air
        spd, direction, dir_z = speed, runway_hdg % 360.0, 90.0
    elif case == "down":                     # pure downdraft — sinking air
        spd, direction, dir_z = speed, runway_hdg % 360.0, -90.0
    elif "+" in case:                        # combined horizontal + vertical
        horiz, vert = case.split("+", 1)
        if horiz not in horiz_dir or vert not in ("up", "down"):
            raise ValueError(f"unknown combined WIND_CASE {case!r} "
                             "(use <head|tail|cross>+<up|down>)")
        v         = vert_speed if vert == "up" else -vert_speed
        spd       = math.hypot(speed, vert_speed)
        direction = horiz_dir[horiz]
        dir_z     = math.degrees(math.atan2(v, speed))   # atan2(0,0)=0 excluded above
    else:
        raise ValueError(f"unknown WIND_CASE {case!r} (use head|tail|cross|up|"
                         "down|updown|none or <head|tail|cross>+<up|down>)")
    return {
        "SIM_WIND_SPD":   spd,
        "SIM_WIND_DIR":   direction,
        "SIM_WIND_DIR_Z": dir_z,
        # metadata recorded into result.json / summary.csv
        "wind_case":      case,
        "wind_speed_mps": spd,                                   # total magnitude
        # Decomposed components for easy post-processing (no trig needed):
        #   horiz = |horizontal wind| (along wind_dir_deg);
        #   vert  = vertical wind, SIGNED (+ updraft / - downdraft).
        "wind_horiz_mps": spd * math.cos(math.radians(dir_z)),
        "wind_vert_mps":  spd * math.sin(math.radians(dir_z)),
        "wind_dir_deg":   direction,
        "wind_dir_z_deg": dir_z,                                 # elevation angle
    }


def valid_wind_case(c: str) -> bool:
    """True if `c` is a recognised wind case: a base type or a "<horiz>+<vert>"
    combined type (horiz in head|tail|cross, vert in up|down|updown)."""
    c = str(c).lower()
    if c in {"head", "tail", "cross", "up", "down", "updown", "none"}:
        return True
    if "+" in c:
        h, _, v = c.partition("+")
        return h in {"head", "tail", "cross"} and v in {"up", "down", "updown"}
    return False


def expand_wind_mix(mix: list, default_speeds: list, default_side: str,
                    default_vert: float) -> list:
    """Expand WIND_MIX into a flat ordered list of (case, h_speed, side, v_speed) specs.

    Each block contributes `count` specs; within a block the HORIZONTAL speed
    sweeps the block's own levels (and a combined block's vert_speed sweeps in
    lockstep), so the block is balanced across wind strengths. An "updown" block
    (or a combined "<horiz>+updown" block) alternates up/down by index (exactly
    even split). "none" blocks contribute zero-speed specs. The returned list's
    length is the natural campaign size (= sum of block counts).
    """
    specs = []
    for blk in mix:
        case  = str(blk["case"]).lower()
        count = int(blk.get("count", 0))
        if count <= 0:
            continue
        speeds = blk.get("speeds") or default_speeds
        side   = str(blk.get("side", default_side)).lower()
        vraw   = blk.get("vert_speed", default_vert)
        vlist  = list(vraw) if isinstance(vraw, (list, tuple)) else [vraw]
        horiz  = vert = None
        if "+" in case:
            horiz, vert = case.split("+", 1)
        for i in range(count):
            if case == "none":
                specs.append(("none", 0.0, side, 0.0))
                continue
            if horiz is not None:            # combined "<horiz>+<vert>"
                vsub = ("up" if i % 2 == 0 else "down") if vert == "updown" else vert
                specs.append((f"{horiz}+{vsub}", speeds[i % len(speeds)], side,
                              float(vlist[i % len(vlist)])))
                continue
            sub = ("up" if i % 2 == 0 else "down") if case == "updown" else case
            specs.append((sub, speeds[i % len(speeds)], side, 0.0))
    return specs


def rescale_specs(specs: list, total: int) -> list:
    """Proportionally resize a wind-spec list to exactly `total` entries.

    Preserves each wind type's share of the campaign (largest-remainder rounding
    so the per-type counts sum to `total` exactly), resampling each type's specs
    evenly so its speed sweep is preserved. Used when --runs overrides the natural
    sum-of-counts campaign size.
    """
    n = len(specs)
    if total == n or n == 0:
        return list(specs)
    # Group specs by (case, side), preserving first-seen order.
    groups, index = [], {}
    for s in specs:
        key = (s[0], s[2])
        if key not in index:
            index[key] = len(groups)
            groups.append([])
        groups[index[key]].append(s)
    # Largest-remainder allocation of `total` across groups by their share.
    raw    = [len(g) / n * total for g in groups]
    floors = [int(math.floor(x)) for x in raw]
    rem    = total - sum(floors)
    order  = sorted(range(len(groups)), key=lambda i: raw[i] - floors[i], reverse=True)
    for j in range(rem):
        floors[order[j % len(order)]] += 1
    out = []
    for g, k in zip(groups, floors):
        for i in range(k):                       # resample group evenly to k entries
            out.append(g[i % len(g)] if k >= len(g) else g[(i * len(g)) // k])
    return out


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
    """Auto-size the parallel-SITL pool to ~85% of this machine's cores.

    Each worker is one CPU-bound `arduplane` SITL process plus a mostly-idle
    Python flight driver. We deliberately leave ~15% of the cores free as a
    safety margin (OS, MAVLink I/O, the orchestrator, and so each SITL can hold
    its requested --speedup without CPU starvation). On a 192-core box this is
    163 workers; on a 96-core box 81; on a 16-core dev box 13. Override with
    --workers to tune.
    """
    cores = os.cpu_count() or 4
    return max(1, (cores * 90) // 100)


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
#  sim_output -> Parquet (storage / file-count reduction)
# ===================================================================
# The sim_output_*.csv the FDM writes is ~85% of each case's bytes (~63 columns,
# ~750 KB/case). As Parquet it is ~3-7x smaller and far faster to load, which speeds
# up BOTH the copy off the box and the dashboard build. The columns below are the ONLY
# ones build_dashboard_data.py + failure_criteria.py actually consume (per_case + TS_COLS
# + landing_metrics + the SIGNAL_DEFS registry) — used only when --trim-cols is set, to
# narrow the file ~50%. Everything else (pwm_*, slew_*, pos_ned_*, Accel_b_*, *_dot,
# alt_msl, coeffs, Side_N, J, Cmu, MLG_NR/FLG_NR, ArduPlane_Mode, mot1_thr_cmd, ...) is
# unused downstream. NOTE: a missing consumed column silently disables a criterion (it goes
# 'na'), so this list MUST stay in sync if the dashboard/criteria add a signal.
SIM_OUTPUT_KEEP = [
    "Time_s", "plane_moving_state", "TAS_mps", "alt_agl_m", "lat", "lon",
    "phi", "theta", "p", "q", "r",
    "V_b_tas_0", "V_b_tas_2", "V_ned_gnd_0", "V_ned_gnd_1", "V_ned_gnd_2",
    "delta_e", "delta_aL", "delta_r",
    "delta_e_cmd", "delta_aL_cmd", "delta_aR_cmd", "delta_r_cmd",
    "mot0_thr_cmd", "Lift_N", "Drag_N", "total_rotor_force",
]


def convert_sim_output_to_parquet(case_dir: str, trim_cols: bool = False) -> int:
    """Convert each sim_output_*.csv in case_dir to Parquet and delete the CSV.

    Best-effort and never fatal: on ANY error the original CSV is left in place, so a
    case is never lost (and build_dashboard_data.py falls back to reading the CSV).
    With trim_cols, keep only SIM_OUTPUT_KEEP (the columns the dashboard + criteria use).
    Returns the number of files converted."""
    n = 0
    for csv_path in glob.glob(os.path.join(case_dir, "sim_output_*.csv")):
        try:
            import pandas as pd                       # lazy: parquet needs pandas+pyarrow
            df = pd.read_csv(csv_path)
            if trim_cols:
                keep = [c for c in SIM_OUTPUT_KEEP if c in df.columns]
                if keep:
                    df = df[keep]
            # zstd (lossless) compresses these high-entropy float columns far better than the
            # parquet default (snappy) — the storage win we're after. Falls back to default if
            # the pyarrow build lacks zstd.
            try:
                df.to_parquet(csv_path[:-4] + ".parquet", index=False, compression="zstd")
            except Exception:
                df.to_parquet(csv_path[:-4] + ".parquet", index=False)
            os.remove(csv_path)
            n += 1
        except Exception:
            pass                                       # leave the CSV; CSV path still works
    return n


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


def has_param(conn, name: str, timeout: float = 3.0) -> bool:
    """True if the connected firmware actually has parameter `name`.

    Used to make differential thrust OPTIONAL so this runner is portable across
    builds: a tree without the AP_DiffThrust module has no UST_* params at all, and
    blindly set_param-ing them just burns 4x3s of retry timeout per case. One PARAM_
    REQUEST_READ tells us whether to touch them. Best-effort: False on any error."""
    try:
        conn.mav.param_request_read_send(
            conn.target_system, conn.target_component, name.encode("ascii"), -1)
    except Exception:
        return False
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg and msg.param_id.replace("\x00", "") == name:
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
_PUMP_TYPES = ["GLOBAL_POSITION_INT", "VFR_HUD", "ATTITUDE", "RAW_IMU", "SERVO_OUTPUT_RAW"]
GRAVITY_MSS = 9.80665


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
            st["hdg"] = m.hdg / 100.0                       # heading (deg), for the yaw doublet
        elif t == "VFR_HUD":
            st["as"]    = m.airspeed
            st["climb"] = m.climb
            st["thr"]   = m.throttle
        elif t == "ATTITUDE":
            st["roll"]  = math.degrees(m.roll)
            st["pitch"] = math.degrees(m.pitch)
            st["yawrate"] = math.degrees(m.yawspeed)        # body yaw rate (deg/s)
        elif t == "SERVO_OUTPUT_RAW":
            # per-motor outputs: SERVO1 (k_motor1, port outer) & SERVO9 (k_motor9,
            # starboard outer). Their split is the direct signature of the
            # differential-thrust mixer working during a yaw doublet (stays ~0 with DT off).
            st["m1"] = getattr(m, "servo1_raw", st.get("m1", 0))
            st["m9"] = getattr(m, "servo9_raw", st.get("m9", 0))
        elif t == "RAW_IMU":
            # RAW_IMU accel fields are milli-g (mg) in the body frame (z down).
            # Normal load factor n_z = -a_z/g (level/at-rest = +1 g); total g is
            # the specific-force magnitude. Track both for post-processing.
            st["nz"] = -m.zacc / 1000.0
            st["g_total"] = math.sqrt(m.xacc**2 + m.yacc**2 + m.zacc**2) / 1000.0
        m = conn.recv_match(type=_PUMP_TYPES, blocking=False)


def fly_cruise_doublets(conn, m, writer, st, perf, sitl_proc, t0_flight, deadline):
    """Fly the mission_ustol_10_cruise(_nodt) profile via RC overrides: open-loop
    takeoff/climb to cruise, then a ROLL-doublet then YAW-doublet dynamic-stability
    test at cruise, then settle back to wings-level and END (the aircraft is left
    airborne; the worker kills SITL afterward). NO loiter, NO landing.

    Each doublet starts from a SETTLE-gated wings-level state so steps don't
    cross-contaminate; we measure held-vs-commanded, overshoot, peak yaw rate and the
    recovery time back to wings-level. A spiral past BANK_ABORT triggers a wings-level
    recovery and aborts the rest of the run. The stability scorecard is stashed into
    perf['stab'] for _run_case to record.

    Returns : 'settled'        test finished and re-settled to wings-level cruise
              'unsettled'      finished but never re-settled within RETURN_MAX_T
              'departed'       spiralled past BANK_ABORT during a doublet (aborted)
              'crashed_climb' | 'crashed_cruise'   SITL process died in that phase
              'climb_timeout'  never reached cruise altitude
              'deadline'       hard per-case deadline hit mid-mission
    """
    u = m.get("ustol2", {})
    # --- climb knobs (same schedule as the landing runner / mission_ustol_9) ---
    CRUISE_ALT     = u.get("cruise_alt_m", m.get("takeoff_alt_m", 100.0))
    CRUISE_AS      = u.get("cruise_as_mps", 12.0)
    CRUISE_CAP_THR = u.get("cruise_cap_thr_pwm", 1660)
    K_CAP          = u.get("k_cap", 3.0)
    LEVEL_BAND     = u.get("level_band_m", 10.0)
    LEVEL_K        = u.get("level_k", 0.5)
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

    # --- stability-test knobs (defaults match mission_ustol_10_cruise_nodt.py) ---
    HOLD_T         = u.get("doublet_hold_s", 6.0)
    ROLL_SEQ       = u.get("roll_seq_deg", [10.0, -10.0, 20.0, -20.0])
    YAW_SEQ        = u.get("yaw_seq_deg",  [10.0, -10.0, 20.0, -20.0])
    SETTLE_OK_T    = u.get("settle_ok_s", 3.0)
    ROLL_OK_DEG    = u.get("roll_ok_deg", 3.0)
    YAWRATE_OK     = u.get("yawrate_ok_dps", 5.0)
    SETTLE_MAX_T   = u.get("settle_max_s", 15.0)
    RECOVER_MAX_T  = u.get("recover_max_s", 15.0)
    RETURN_MAX_T   = u.get("return_max_s", 20.0)
    K_ALT_THR      = u.get("k_alt_thr", 6.0)
    THR_CR_MIN     = u.get("thr_cr_min_pwm", 1450)
    THR_CR_MAX     = u.get("thr_cr_max_pwm", 1850)
    BANK_THR_FREEZE= u.get("bank_thr_freeze_deg", 25.0)
    BANK_ABORT     = u.get("bank_abort_deg", 45.0)
    # Altitude floor for the cruise stability test: a doublet can upset the aircraft into a
    # WINGS-LEVEL descent, which the roll+yaw-only settle/abort logic below would otherwise call
    # 'settled' as it sinks to the ground. Treat dropping below this floor as a departure (and never
    # count a sub-floor sample as 'in band' for a settle). Default = half the cruise altitude.
    ALT_FLOOR_M    = u.get("test_alt_floor_m", 0.5 * CRUISE_ALT)
    RECOVER_PITCH  = u.get("recover_pitch_deg", -3.0)
    RECOVER_THR    = u.get("recover_thr_pwm", 1500)
    ROLL_LIMIT_DEG = u.get("roll_limit_deg", 30.0)
    RUD_LIMIT_DEG  = u.get("rud_limit_deg", 40.0)
    RC1_MIN        = u.get("rc1_min", 1000); RC1_TRIM = u.get("rc1_trim", 1500); RC1_MAX = u.get("rc1_max", 1775)
    RC4_MIN        = u.get("rc4_min", 1000); RC4_TRIM = u.get("rc4_trim", 1500); RC4_MAX = u.get("rc4_max", 2000)
    ROLL_SIGN      = u.get("roll_sign", 1)
    RUD_SIGN       = u.get("rud_sign", 1)

    class _Stop(Exception):
        def __init__(self, reason): self.reason = reason

    def _smooth(x):
        x = 0.0 if x < 0 else (1.0 if x > 1 else x)
        return x * x * (3.0 - 2.0 * x)

    def _clamp(v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)

    def theta_target(V):
        if V <= V_R:     return 0.0
        if V <= V_LO:    return TH_LO * _smooth((V - V_R) / (V_LO - V_R))
        if V <= V_CLIMB: return TH_LO + (TH_CLIMB - TH_LO) * _smooth((V - V_LO) / (V_CLIMB - V_LO))
        return TH_CLIMB

    def rate_limit(prev, target, dt, cap=RATE_LIM):
        step = cap * dt
        return max(prev - step, min(target, prev + step))

    def pitch_pwm(theta_deg):
        frac = _clamp(PITCH_SIGN * theta_deg / PTCH_LIM_MAX, -1.0, 1.0)
        return int(RC2_TRIM + frac * (RC2_MAX - RC2_TRIM))

    def roll_pwm(bank_deg):
        # honor the AP's ASYMMETRIC RC1 calibration (right half spans TRIM..MAX, left TRIM..MIN)
        frac = _clamp(ROLL_SIGN * bank_deg / ROLL_LIMIT_DEG, -1.0, 1.0)
        span = (RC1_MAX - RC1_TRIM) if frac >= 0 else (RC1_TRIM - RC1_MIN)
        return int(RC1_TRIM + frac * span)

    def rud_pwm(rud_deg):
        frac = _clamp(RUD_SIGN * rud_deg / RUD_LIMIT_DEG, -1.0, 1.0)
        return int(RC4_TRIM + frac * (RC4_MAX - RC4_TRIM))

    def send_sticks(theta_deg, roll_deg=0.0, rud_deg=0.0, thr=None):
        # AETR override: ch1 roll (bank), ch2 pitch, ch3 throttle, ch4 rudder (manual).
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component,
            roll_pwm(roll_deg), pitch_pwm(theta_deg),
            int(THR_PWM if thr is None else thr), rud_pwm(rud_deg),
            0, 0, 0, 0)

    def release_sticks():
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

    _last_csv = [0.0]

    def write_row(phase, theta, roll_cmd=0.0, rud_cmd=0.0):
        now = time.time()
        if now - _last_csv[0] < 0.2:           # ~5 Hz
            return
        _last_csv[0] = now
        writer.writerow([
            "%.2f" % (now - t0_flight), phase,
            "%.2f" % st["alt"], "%.2f" % st["as"],
            "%.2f" % st["roll"], "%.2f" % st["pitch"],
            "%.2f" % st["climb"], "%.0f" % st["thr"],
            "%.2f" % theta,
            "%.7f" % (st["lat"] or 0.0), "%.7f" % (st["lon"] or 0.0),
            "%.3f" % st["nz"], "%.3f" % st["g_total"],
            "%d" % st.get("m1", 0), "%d" % st.get("m9", 0),
            "%.2f" % st.get("yawrate", 0.0),
            "%.2f" % roll_cmd, "%.2f" % rud_cmd])

    def cruise_pitch():
        # plain speed-on-pitch about CRUISE_AS (no load-factor nose-up term, which would
        # tighten a spiral in a bank).
        return _clamp(K_CAP * (st["as"] - CRUISE_AS), -5.0, TH_CLIMB)

    def cruise_thr():
        # altitude-hold trim, FROZEN above BANK_THR_FREEZE bank so we never push power
        # into a turn/spiral.
        if abs(st["roll"]) > BANK_THR_FREEZE:
            return CRUISE_CAP_THR
        return int(_clamp(CRUISE_CAP_THR + K_ALT_THR * (CRUISE_ALT - st["alt"]),
                          THR_CR_MIN, THR_CR_MAX))

    def departed():
        # Spiral past the bank limit OR a loss of test altitude (a sinking, wings-level mush is
        # just as much a departure as an over-bank — and the roll/yaw checks alone never catch it).
        return abs(st["roll"]) > BANK_ABORT or st["alt"] < ALT_FLOOR_M

    stab = {"n_ok": 0, "n_aborted": 0, "departed": False,
            "roll_worst_held_err_deg": 0.0, "roll_worst_overshoot_deg": 0.0,
            "roll_worst_recover_s": -1.0, "yaw_peak_yawrate_dps": 0.0,
            "yaw_worst_recover_s": -1.0, "max_motor_diff_pwm": 0.0,
            "alt_min_m": None, "alt_max_m": None,
            "settled_final": False, "t_settle_final_s": None}
    perf["stab"] = stab

    def _guard():
        if sitl_proc.poll() is not None:
            raise _Stop("crashed_cruise")
        if time.time() > deadline:
            raise _Stop("deadline")

    def _track(phase, roll_cmd=0.0, rud_cmd=0.0):
        d = abs(st.get("m1", 0) - st.get("m9", 0))
        stab["max_motor_diff_pwm"] = max(stab["max_motor_diff_pwm"], d)
        a = st["alt"]
        stab["alt_min_m"] = a if stab["alt_min_m"] is None else min(stab["alt_min_m"], a)
        stab["alt_max_m"] = a if stab["alt_max_m"] is None else max(stab["alt_max_m"], a)
        write_row(phase, 0.0, roll_cmd, rud_cmd)

    def settle(phase, max_t):
        """Hold wings-level/neutral cruise (with alt-hold) until in-band continuously for
        SETTLE_OK_T s, or until max_t. Returns seconds-to-settle, or -1.0 on timeout."""
        t0 = time.time(); ok_since = None
        while time.time() - t0 < max_t:
            _guard()
            _pump(conn, st)
            send_sticks(cruise_pitch(), thr=cruise_thr())
            now = time.time()
            in_band = (abs(st["roll"]) < ROLL_OK_DEG and abs(st["yawrate"]) < YAWRATE_OK
                       and st["alt"] >= ALT_FLOOR_M)   # sinking out of the band is NOT 'settled'
            ok_since = ok_since if (in_band and ok_since is not None) else (now if in_band else None)
            _track(phase)
            if ok_since is not None and now - ok_since >= SETTLE_OK_T:
                return now - t0
            time.sleep(0.05)
        return -1.0

    def recover():
        """Break a spiral: wings level + fixed nose-down + reduced throttle until settled."""
        t0 = time.time(); ok_since = None
        while time.time() - t0 < RECOVER_MAX_T:
            _guard()
            _pump(conn, st)
            send_sticks(RECOVER_PITCH, roll_deg=0.0, rud_deg=0.0, thr=RECOVER_THR)
            now = time.time()
            in_band = (abs(st["roll"]) < ROLL_OK_DEG and abs(st["yawrate"]) < YAWRATE_OK
                       and st["alt"] >= ALT_FLOOR_M)   # sinking out of the band is NOT 'settled'
            ok_since = ok_since if (in_band and ok_since is not None) else (now if in_band else None)
            _track("RECOVER")
            if ok_since is not None and now - ok_since >= SETTLE_OK_T:
                return
            time.sleep(0.05)

    def run_doublets(label, targets, axis):
        """Run the doublet sequence on one axis. Returns True if ABORTED on a departure."""
        for sp in targets:
            settle(label, SETTLE_MAX_T)                     # (a) gate to a clean wings-level start
            if departed():
                stab["n_aborted"] += 1; stab["departed"] = True
                recover(); return True
            # (b) apply the step and HOLD
            t0 = time.time(); pk_roll = 0.0; pk_yawrate = 0.0
            while time.time() - t0 < HOLD_T:
                _guard()
                _pump(conn, st)
                if axis == "roll":
                    send_sticks(cruise_pitch(), roll_deg=sp, thr=cruise_thr())
                else:
                    send_sticks(cruise_pitch(), rud_deg=sp, thr=cruise_thr())
                if abs(st["roll"])    > abs(pk_roll):    pk_roll = st["roll"]
                if abs(st["yawrate"]) > abs(pk_yawrate): pk_yawrate = st["yawrate"]
                _track(label, roll_cmd=(sp if axis == "roll" else 0.0),
                       rud_cmd=(sp if axis == "yaw" else 0.0))
                if departed():
                    stab["n_aborted"] += 1; stab["departed"] = True
                    recover(); return True
                time.sleep(0.05)
            held_roll = st["roll"]; held_yawrate = st["yawrate"]
            # (c) RECOVERY: command 0 and time the return to wings-level
            tr = time.time(); ok_since = None; t_rec = -1.0
            while time.time() - tr < RECOVER_MAX_T:
                _guard()
                _pump(conn, st)
                send_sticks(cruise_pitch(), thr=cruise_thr())
                now = time.time()
                if departed():
                    stab["n_aborted"] += 1; stab["departed"] = True
                    recover(); return True
                in_band = (abs(st["roll"]) < ROLL_OK_DEG and abs(st["yawrate"]) < YAWRATE_OK
                       and st["alt"] >= ALT_FLOOR_M)   # sinking out of the band is NOT 'settled'
                ok_since = ok_since if (in_band and ok_since is not None) else (now if in_band else None)
                _track(label)
                if ok_since is not None and now - ok_since >= SETTLE_OK_T:
                    t_rec = ok_since - tr; break
                time.sleep(0.05)
            stab["n_ok"] += 1
            if axis == "roll":
                stab["roll_worst_held_err_deg"]  = max(stab["roll_worst_held_err_deg"],
                                                        abs(abs(held_roll) - abs(sp)))
                stab["roll_worst_overshoot_deg"] = max(stab["roll_worst_overshoot_deg"],
                                                        abs(pk_roll) - abs(held_roll))
                if t_rec >= 0:
                    stab["roll_worst_recover_s"] = max(stab["roll_worst_recover_s"], t_rec)
            else:
                stab["yaw_peak_yawrate_dps"] = max(stab["yaw_peak_yawrate_dps"], abs(pk_yawrate))
                if t_rec >= 0:
                    stab["yaw_worst_recover_s"] = max(stab["yaw_worst_recover_s"], t_rec)
        return False

    # ---- ground roll + climb to cruise (open-loop, wings level, neutral rudder) ----
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
        if perf["liftoff_time"] is None and st["alt"] >= 0.5:
            perf["liftoff_time"]  = now - t0_flight
            perf["liftoff_speed"] = st["as"]
        fpa = math.degrees(math.asin(max(-1.0, min(1.0, st["climb"] / max(st["as"], 0.1)))))
        spd_trim = K_SPD * max(0.0, V_HOLD - st["as"])
        thr_cmd = THR_PWM
        phase = "CLIMB"
        if st["alt"] >= CRUISE_ALT - LEVEL_BAND:
            if flare0 is None:
                flare0 = theta
            s = math.exp(-LEVEL_K * abs(st["alt"] - CRUISE_ALT))
            target  = flare0 * (1.0 - s)
            thr_cmd = THR_PWM + (CRUISE_CAP_THR - THR_PWM) * s
        elif v_peak < V_R:
            target = GND_PITCH
            phase = "GROUND"
        else:
            target = min(theta_target(v_peak) - spd_trim, fpa + AOA_CAP)
            target = max(0.0, target)
        theta = rate_limit(theta, target, dt)
        send_sticks(theta, thr=int(thr_cmd))
        write_row(phase, theta)
        if st["alt"] >= CRUISE_ALT - 1.0:
            break
        time.sleep(0.05)

    # ---- cruise settle -> ROLL doublets -> YAW doublets -> final settle ----
    try:
        settle("CRUISE", SETTLE_MAX_T)                      # begin from clean trim
        aborted = run_doublets("ROLL", ROLL_SEQ, "roll")
        if not aborted:
            aborted = run_doublets("YAW", YAW_SEQ, "yaw")
        t_fin = settle("RETURN", RETURN_MAX_T)
        stab["settled_final"]   = t_fin >= 0
        stab["t_settle_final_s"] = t_fin if t_fin >= 0 else None
        release_sticks()
        if aborted:
            return "departed"
        return "settled" if t_fin >= 0 else "unsettled"
    except _Stop as s:
        return s.reason


# ===================================================================
#  Per-case metrics (from the per-case telemetry CSV this runner writes)
# ===================================================================
def metrics_from_csv(csv_path, home_lat, home_lon):
    """Derive the flight ENVELOPE metrics from the per-case telemetry CSV. The
    doublet-specific stability metrics are computed live in fly_cruise_doublets and
    recorded from perf['stab']; this only covers max alt/roll/pitch, airspeed band,
    load factor and the takeoff/cruise timestamps. (home_lat/home_lon unused here —
    this profile never lands, so there is no landing point.)"""
    m = dict(max_alt_m=0.0, max_roll_deg=0.0, max_pitch_deg=0.0,
             min_airspeed=None, max_airspeed=0.0,
             max_load_factor_nz=None, min_load_factor_nz=None, max_load_factor_total=0.0,
             time_to_takeoff_s=None, time_to_cruise_s=None)

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
                if alt > 0.5 and ("load_factor_nz" in row):
                    nz = _f(row, "load_factor_nz")
                    m["max_load_factor_nz"] = nz if m["max_load_factor_nz"] is None else max(m["max_load_factor_nz"], nz)
                    m["min_load_factor_nz"] = nz if m["min_load_factor_nz"] is None else min(m["min_load_factor_nz"], nz)
                    m["max_load_factor_total"] = max(m["max_load_factor_total"], _f(row, "load_factor_total"))
                t = _f(row, "time_s")
                phase = row.get("phase", "")
                if m["time_to_takeoff_s"] is None and alt >= 0.8:
                    m["time_to_takeoff_s"] = t
                if m["time_to_cruise_s"] is None and phase == "CRUISE":
                    m["time_to_cruise_s"] = t
    except FileNotFoundError:
        return m
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
        dt_tag = "diff-thrust ON" if result.get("dt_enabled") else "diff-thrust OFF (baseline)"
        f.write(f"Monte Carlo Case #{case_id:04d}  [uSTOL cruise roll/yaw doublet stability; {dt_tag}]\n")
        f.write("=" * 70 + "\n\n")
        f.write(f"Timestamp : {datetime.now().isoformat()}\n")
        f.write(f"Duration  : {result.get('duration_s', 0):.1f} s\n")
        f.write(f"Wind      : {result.get('wind_case', 'none')}  "
                f"total {result.get('wind_speed_mps', 0):.1f} m/s  "
                f"(horiz {result.get('wind_horiz_mps', 0):.1f}, "
                f"vert {result.get('wind_vert_mps', 0):+.1f} m/s)\n\n")

        f.write(f"{'Parameter':<22} {'Nominal':>14} {'Perturbed':>14} {'Delta%':>9}\n")
        f.write("-" * 70 + "\n")
        for name in perturbed:
            nom  = nominals[name]
            pert = perturbed[name]
            pct  = ((pert - nom) / abs(nom) * 100) if abs(nom) > 1e-12 else 0.0
            f.write(f"{name:<22} {nom:>14.6f} {pert:>14.6f} {pct:>+8.2f}%\n")

        f.write("\n" + "=" * 70 + "\nResult\n" + "-" * 70 + "\n")
        f.write(f"  Test result       : {result.get('land_result', 'unknown')}\n")
        f.write(f"  Diff-thrust       : {'ON' if result.get('dt_enabled') else 'OFF (baseline)'}\n")
        f.write(f"  Takeoff success   : {result.get('takeoff_success', False)}\n")
        f.write(f"  Settled to cruise : {result.get('stab_settled')}  "
                f"(t_settle {result.get('stab_t_settle_final_s')} s)\n")
        f.write(f"  Doublets ok/abort : {result.get('stab_n_doublets_ok')}/{result.get('stab_n_aborted')}"
                f"  departed={result.get('stab_departed')}\n")
        f.write(f"  ROLL worst        : |held-cmd| {result.get('roll_worst_held_err_deg')} deg  "
                f"overshoot {result.get('roll_worst_overshoot_deg')} deg  "
                f"recover {result.get('roll_worst_recover_s')} s\n")
        f.write(f"  YAW  worst        : peak yaw-rate {result.get('yaw_peak_yawrate_dps')} deg/s  "
                f"recover {result.get('yaw_worst_recover_s')} s\n")
        f.write(f"  Motor split       : max |m1-m9| {result.get('max_motor_diff_pwm')} pwm "
                f"(~0 expected with DT off)\n")
        f.write(f"  Max altitude (m)  : {result.get('max_alt_m', 0):.1f}\n")
        f.write(f"  Load factor n_z   : {result.get('min_load_factor_nz')} .. "
                f"{result.get('max_load_factor_nz')} g  "
                f"(peak |g| {result.get('max_load_factor_total')})\n")
        f.write(f"  Liftoff Vlof (m/s): {result.get('liftoff_speed')}\n")
        f.write(f"  Exit reason       : {result.get('exit_reason', 'unknown')}\n")


# ===================================================================
#  Single-case worker
# ===================================================================
def _run_case(case_id, case_seed, instance_id, config, perturbed, output_dir,
              workspace, speedup, save_plot, hard_timeout, params_file, wind,
              dt_enabled=True, sim_parquet=True, trim_cols=False, keep_aux=False):
    """Execute one Monte Carlo case: launch SITL, set the case's wind, optionally
    enable differential thrust, fly the uSTOL cruise roll/yaw doublet stability test."""
    result = dict(
        case_id=case_id, case_seed=case_seed,
        takeoff_success=False, mission_complete=False,
        duration_s=0.0, max_alt_m=0.0, land_result="unknown",
        liftoff_speed=None, exit_reason="unknown", dt_enabled=bool(dt_enabled),
        wind_case=wind["wind_case"], wind_speed_mps=wind["wind_speed_mps"],
        wind_horiz_mps=wind["wind_horiz_mps"], wind_vert_mps=wind["wind_vert_mps"],
        wind_dir_deg=wind["wind_dir_deg"], wind_dir_z_deg=wind["wind_dir_z_deg"])

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
        # SITL stdout is the FDM's per-step debug spam ("Time:..., MLG_NR...") — a few MB of
        # serial-port noise per case that nothing downstream reads. Drop it to /dev/null by
        # default (no file, no spam); --keep-aux re-enables the sitl_stdout.log for debugging.
        sitl_log = (open(os.path.join(case_dir, "sitl_stdout.log"), "w")
                    if keep_aux else subprocess.DEVNULL)
        sitl_proc = subprocess.Popen(
            cmd, cwd=worker_dir, env=env,
            stdout=sitl_log,
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
            ("ROLL_LIMIT_DEG",   ucfg.get("roll_limit_deg", 30.0)),  # FBWA bank authority for the roll doublets
        ]:
            set_param(conn, n, v)

        # ---- DIFFERENTIAL THRUST: when enabled, map the nine ESC outputs to
        #      k_motor1..k_motor9 and turn on the AP_DiffThrust mixer (UST_*). The mixer
        #      becomes the SOLE writer of those outputs, reading the throttle demand as a
        #      base and adding an antisymmetric rudder-driven split. UST_UMAX=1.0 keeps
        #      takeoff thrust uncapped (the bench overlay's 0.4619 wire-cap would peg
        #      throttle at ~46% and prevent takeoff). With --diff-thrust off the outputs
        #      stay on k_throttle and UST_ENABLE=0 -> stock uniform throttle (the no-DT
        #      baseline; run both ways and diff the roll/yaw stability metrics). ----
        # Differential thrust is OPTIONAL and probed, so this runner is portable to a
        # build WITHOUT the AP_DiffThrust module (no UST_* params — e.g. a tree where DT
        # was never ported). With DT off we just leave the stock uniform throttle; with
        # DT on against such a build we abort the case with a clear reason instead of
        # silently remapping SERVO1..9 to motors that nothing drives (-> no takeoff).
        has_ust = has_param(conn, "UST_ENABLE")
        if dt_enabled:
            if not has_ust:
                result["exit_reason"] = "ust_unavailable"
                _log(case_id, "ERROR: --diff-thrust on but UST_ENABLE not in this firmware "
                              "(AP_DiffThrust not built); aborting case. Port AP_DiffThrust "
                              "+ ./waf plane, or run --diff-thrust off.")
                return result
            K_MOTOR_FN = [33, 34, 35, 36, 37, 38, 39, 40, 82]      # k_motor1..k_motor9 functions
            for ch, fn in enumerate(K_MOTOR_FN, start=1):
                set_param(conn, f"SERVO{ch}_FUNCTION", fn)
            for n, v in [
                ("UST_ENABLE",  1),
                ("UST_DT_VLO",  ucfg.get("ust_dt_vlo", 13.0)),
                ("UST_DT_VHI",  ucfg.get("ust_dt_vhi", 17.0)),
                ("UST_NDES_MAX", ucfg.get("ust_ndes_max", 20.0)),  # peak yaw MOMENT [N*m] (was UST_DT_KYAW gain)
                ("UST_KRUD",    ucfg.get("ust_krud", 0.11)),       # >0 = rudder-aware daisy-chain (new); 0 = legacy VLO/VHI
                ("UST_DT_RLFF", ucfg.get("ust_dt_rlff", 0.0)),
                ("UST_UMAX",    ucfg.get("ust_umax", 1.0)),        # 1.0 = uncapped (SITL takeoff)
            ]:
                set_param(conn, n, v)
            _log(case_id, "differential thrust ENABLED (SERVO1..9=k_motor1..9, UST_ENABLE=1)")
        elif has_ust:
            set_param(conn, "UST_ENABLE", 0)                       # explicit stock baseline (only if present)
        else:
            _log(case_id, "no UST_ params in firmware — stock uniform throttle (no-DT baseline)")

        # ---- WIND: set this case's SIM_WIND_* BEFORE arming so wind is active
        #      from the ground roll on. SIM_Plane.cpp copies wind_ef -> wind_ned
        #      every step, so these drive the custom uSTOL FDM. Small TC -> wind
        #      ramps to full well before liftoff. ----
        for n, v in [
            ("SIM_WIND_T",    1 if WIND_PROFILE_CONSTANT else 0),  # 1 = constant w/ alt
            ("SIM_WIND_TC",   1.0),                                # fast ramp-in (s)
            ("SIM_WIND_TURB", WIND_TURBULENCE),                    # RMS gusts (0 = steady)
            ("SIM_WIND_SPD",  wind["SIM_WIND_SPD"]),
            ("SIM_WIND_DIR",  wind["SIM_WIND_DIR"]),
            ("SIM_WIND_DIR_Z", wind["SIM_WIND_DIR_Z"]),
        ]:
            set_param(conn, n, v)
        _log(case_id, f"wind: {wind['wind_case']}  "
                      f"horiz {wind['wind_horiz_mps']:.1f}  vert {wind['wind_vert_mps']:+.1f} m/s  "
                      f"(spd {wind['SIM_WIND_SPD']:.1f}  "
                      f"dir {wind['SIM_WIND_DIR']:.0f}°  dirZ {wind['SIM_WIND_DIR_Z']:.0f}°)")

        if not set_mode(conn, "FBWA"):
            result["exit_reason"] = "fbwa_mode_failed"
            return result
        wait_ekf_ready(conn, timeout=ucfg.get("ekf_timeout_s", 60))
        if not arm_vehicle(conn):
            result["exit_reason"] = "arm_failed"
            return result
        _log(case_id, "armed — flying uSTOL climb + cruise doublet stability test")

        # Per-case telemetry CSV (drives metrics_from_csv + summary).
        csv_fp = open(case_csv, "w", newline="")
        writer = csv.writer(csv_fp)
        writer.writerow([
            "time_s", "phase", "alt_agl_m", "airspeed", "roll_deg",
            "pitch_deg", "climb_mps", "throttle_pct", "theta_cmd_deg", "lat", "lon",
            "load_factor_nz", "load_factor_total",
            "motor1_pwm", "motor9_pwm", "yaw_rate_dps", "roll_cmd_deg", "rud_cmd_deg"])

        st = {"alt": 0.0, "as": 0.0, "climb": 0.0, "lat": None, "lon": None,
              "thr": 0.0, "roll": 0.0, "pitch": 0.0, "nz": 1.0, "g_total": 1.0,
              "m1": 0, "m9": 0, "yawrate": 0.0, "hdg": 0.0}
        perf = {"liftoff_time": None, "liftoff_speed": None, "ground_roll_m": None}
        t0_flight = time.time()

        land_result = fly_cruise_doublets(conn, mission_cfg, writer, st, perf,
                                          sitl_proc, t0_flight, deadline)
        _log(case_id, f"profile returned: {land_result}")

        # ---- Outcome ----
        stab = perf.get("stab", {})
        result["land_result"]      = land_result          # stability outcome (settled/unsettled/departed/...)
        result["takeoff_success"]  = perf["liftoff_time"] is not None
        result["mission_complete"] = (land_result == "settled")   # full test = re-settled to cruise
        result["liftoff_speed"]    = perf["liftoff_speed"]
        result["stab_settled"]             = stab.get("settled_final", False)
        result["stab_t_settle_final_s"]    = stab.get("t_settle_final_s")
        result["stab_departed"]            = stab.get("departed", False)
        result["stab_n_doublets_ok"]       = stab.get("n_ok", 0)
        result["stab_n_aborted"]           = stab.get("n_aborted", 0)
        result["roll_worst_held_err_deg"]  = stab.get("roll_worst_held_err_deg")
        result["roll_worst_overshoot_deg"] = stab.get("roll_worst_overshoot_deg")
        result["roll_worst_recover_s"]     = stab.get("roll_worst_recover_s")
        result["yaw_peak_yawrate_dps"]     = stab.get("yaw_peak_yawrate_dps")
        result["yaw_worst_recover_s"]      = stab.get("yaw_worst_recover_s")
        result["test_alt_min_m"]           = stab.get("alt_min_m")
        result["test_alt_max_m"]           = stab.get("alt_max_m")
        result["max_motor_diff_pwm"]       = stab.get("max_motor_diff_pwm")
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

        # sim_output -> Parquet (the high-value reduction: sim_output is ~85% of a case's
        # bytes). Done AFTER the CSV is in case_dir; best-effort (leaves the CSV on any error,
        # which build_dashboard_data.py still reads). --sim-format csv skips this.
        if sim_parquet:
            convert_sim_output_to_parquet(case_dir, trim_cols=trim_cols)

        # Lean output (6->3 files): drop the per-case files the dashboard never reads.
        # overrides.txt is redundant (its params are in result.json); the report.txt below is
        # skipped too, and sitl_stdout.log was never created (DEVNULL). --keep-aux retains all.
        if not keep_aux:
            try:
                os.remove(override_file)
            except OSError:
                pass

        # Optional per-case PNG (headless Agg backend).
        if save_plot:
            try:
                plot_case(case_csv)
            except Exception:
                pass

        # Human-readable per-case report — the dashboard never reads it, so write it only
        # with --keep-aux (part of the 6->3 file-count reduction).
        if keep_aux:
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
     speedup, save_plot, hard_timeout, params_file, wind, dt_enabled,
     sim_parquet, trim_cols, keep_aux) = args
    inst = _instance_queue.get()
    try:
        return _run_case(case_id, case_seed, inst, config, perturbed, output_dir,
                         workspace, speedup, save_plot, hard_timeout, params_file, wind,
                         dt_enabled, sim_parquet, trim_cols, keep_aux)
    finally:
        _instance_queue.put(inst)


# ===================================================================
#  Summary output
# ===================================================================
# Frozen output schema — DO NOT add/remove columns after a campaign starts
# (re-deriving metrics from thousands of logs afterward is painful).
SUMMARY_COLUMNS = [
    "case_id", "case_seed", "dt_enabled", "takeoff_success", "mission_complete", "land_result",
    "exit_reason", "duration_s",
    "wind_case", "wind_speed_mps", "wind_horiz_mps", "wind_vert_mps",
    "wind_dir_deg", "wind_dir_z_deg",
    "liftoff_speed",
    "max_alt_m", "max_roll_deg", "max_pitch_deg", "min_airspeed", "max_airspeed",
    "max_load_factor_nz", "min_load_factor_nz", "max_load_factor_total",
    "time_to_takeoff_s", "time_to_cruise_s",
    # cruise roll/yaw doublet stability metrics:
    "stab_settled", "stab_t_settle_final_s", "stab_departed",
    "stab_n_doublets_ok", "stab_n_aborted",
    "roll_worst_held_err_deg", "roll_worst_overshoot_deg", "roll_worst_recover_s",
    "yaw_peak_yawrate_dps", "yaw_worst_recover_s",
    "test_alt_min_m", "test_alt_max_m", "max_motor_diff_pwm",
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
        description="Monte Carlo Runner — uSTOL CRUISE ROLL/YAW DOUBLET STABILITY (no-DT) + WIND "
                    "(mission_ustol_10_cruise_nodt). Edit WIND_MIX at the top to set per-type counts.")
    parser.add_argument("--config", default=str(DEFAULT_CFG))
    parser.add_argument("--runs", type=int, default=None,
                        help="number of cases. If omitted, the campaign size is "
                             "the sum of WIND_MIX counts. If given, the mix is "
                             "proportionally rescaled to this size (ratios kept).")
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
    parser.add_argument("--diff-thrust", choices=["on", "off"], default="off",
                        help="differential-thrust yaw mixer. 'off' (DEFAULT, the no-DT baseline): "
                             "stock uniform throttle (UST_ENABLE=0), yaw via rudder. 'on': map "
                             "SERVO1..9 -> k_motor1..9 + UST_ENABLE=1 (UST_UMAX=1.0). Run both and "
                             "diff the roll/yaw stability metrics to see DT's effect in turns.")
    parser.add_argument("--sim-format", choices=["parquet", "csv"], default="parquet",
                        help="per-case sim_output format. 'parquet' (default): convert the FDM "
                             "CSV to Parquet (~85%% of a case's bytes, ~3-7x smaller, faster to "
                             "load + copy). 'csv': keep the raw CSV (e.g. for plot_case_full.py).")
    parser.add_argument("--trim-cols", action="store_true",
                        help="when writing Parquet, keep only the ~26 sim_output columns the "
                             "dashboard + failure criteria use (~50%% narrower). Off by default "
                             "(keep all columns) so nothing downstream can silently break.")
    parser.add_argument("--keep-aux", action="store_true",
                        help="keep the per-case files the dashboard never reads (overrides.txt, "
                             "report.txt, sitl_stdout.log). Default: drop them (6->3 files/case) "
                             "and send SITL stdout to /dev/null (no serial-port spam).")
    args = parser.parse_args()
    dt_enabled  = (args.diff_thrust == "on")
    sim_parquet = (args.sim_format == "parquet")
    trim_cols   = args.trim_cols

    config      = load_config(args.config)
    # Param/defaults file: --params wins; else the historical auto-resolve order.
    if args.params:
        params_file = os.path.abspath(args.params)
        if not os.path.isfile(params_file):
            print(f"\nERROR: --params file not found: {params_file}")
            sys.exit(1)
    else:
        params_file = resolve_defaults_parm()
    # Wind MIX: validate every block, then expand it to a flat per-case spec list.
    for blk in WIND_MIX:
        if not valid_wind_case(blk.get("case", "")):
            print(f"\nERROR: WIND_MIX case={blk.get('case')!r} invalid; use "
                  "head|tail|cross|up|down|updown|none or "
                  "<head|tail|cross>+<up|down|updown>")
            sys.exit(1)
        if int(blk.get("count", 0)) < 0:
            print(f"\nERROR: WIND_MIX count must be >= 0 (block {blk})")
            sys.exit(1)
    runway_hdg = config["mission"].get("runway_heading_deg", 285)
    wind_specs = expand_wind_mix(WIND_MIX, WIND_SPEED_LEVELS, CROSSWIND_SIDE,
                                 WIND_VERT_SPEED)
    if not wind_specs:
        print("\nERROR: WIND_MIX expands to 0 cases — set some non-zero counts.")
        sys.exit(1)

    # Runs: --runs wins (proportionally rescaling the mix to that size); otherwise
    # the campaign size is the natural sum of WIND_MIX counts (no prompt needed).
    if args.runs is not None:
        num_runs   = args.runs
        wind_specs = rescale_specs(wind_specs, num_runs)
    else:
        num_runs   = len(wind_specs)

    # Optional deterministic (seeded) shuffle so any prefix of the campaign is a
    # representative sample of the whole mix; otherwise blocks stay in WIND_MIX order.
    if WIND_MIX_SHUFFLE and num_runs > 1:
        order = np.random.default_rng(args.seed).permutation(len(wind_specs))
        wind_specs = [wind_specs[i] for i in order]

    # Per-type breakdown for the banner / DONE.txt (cross/combined split by side).
    wind_counts = {}
    for case, _spd, side, _vspd in wind_specs:
        key = f"{case}-{side}" if "cross" in case else case
        wind_counts[key] = wind_counts.get(key, 0) + 1

    # Worker precedence: explicit --workers > auto-size to this machine.
    max_workers = args.workers or auto_workers()
    # Never spin up more SITL instances than there are cases to run.
    max_workers = max(1, min(max_workers, num_runs))
    base_inst   = args.base_instance
    hard_timeout = (args.hard_timeout
                    or config["mission"].get("hard_timeout_s", 420))

    print("=" * 70)
    print("  LAT Monte Carlo Runner — uSTOL CRUISE ROLL/YAW DOUBLET STABILITY (no-DT) + WIND (mission_ustol_10)")
    print("=" * 70)
    print(f"  Wind mix      : {num_runs} cases across {len(wind_counts)} type(s)")
    for k in sorted(wind_counts):
        print(f"      {k:<12}: {wind_counts[k]}")
    print(f"  Wind speeds   : {WIND_SPEED_LEVELS} m/s default horiz  (swept within each type)")
    print(f"  Vert speed    : {WIND_VERT_SPEED} m/s default  (combined cases)")
    print(f"  Wind shuffle  : {WIND_MIX_SHUFFLE}")
    print(f"  Wind profile  : {'constant w/ alt (SIM_WIND_T=1)' if WIND_PROFILE_CONSTANT else 'sqrt (SIM_WIND_T=0)'}"
          f"  turb {WIND_TURBULENCE} m/s  runway hdg {runway_hdg}°")
    print(f"  Runs          : {num_runs}")
    print(f"  CPU cores     : {os.cpu_count()}")
    print(f"  Workers       : {max_workers}"
          f"{'' if args.workers else '  (auto)'}")
    print(f"  Speedup       : {args.speedup}x")
    if args.speedup > 1:
        print("  NOTE          : speedup>1 + high worker count starves each "
              "SITL of CPU;\n"
              "                  validate pass-rate against a 1x baseline.")
    print(f"  Diff-thrust   : {'ON (SERVO1..9=k_motor1..9, UST_ENABLE=1, UMAX=1.0)' if dt_enabled else 'OFF (stock uniform throttle — no-DT cruise baseline)'}")
    print(f"  Output        : sim_output={'Parquet' if sim_parquet else 'CSV'}"
          f"{' (trimmed cols)' if (sim_parquet and trim_cols) else ''}; "
          f"aux files {'kept' if args.keep_aux else 'dropped (6->3/case, no SITL-stdout spam)'}")
    print(f"  Seed          : {args.seed}")
    print(f"  Hard timeout  : {hard_timeout}s/case")
    print(f"  Resume        : {args.resume}")
    print(f"  Base instance : {base_inst}  (ports {5760+10*base_inst}"
          f"–{5760+10*(base_inst+max_workers-1)})")
    print(f"  Binary        : {BINARY}")
    print(f"  Defaults      : {params_file}"
          f"{'  (--params)' if args.params else '  (auto)'}")
    print(f"  Config        : {args.config}  ({len(config['params'])} params)")
    print("  Profile       : GROUND→CLIMB→CRUISE→ROLL-DOUBLETS→YAW-DOUBLETS→SETTLE  (FBWA, no landing)")
    print("=" * 70)

    if not BINARY.exists():
        print(f"\nERROR: Binary not found: {BINARY}\n       Run  ./waf plane  first.")
        sys.exit(1)

    if args.output_dir:
        output_dir = os.path.abspath(args.output_dir)
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Tag as a mixed-wind campaign.
        output_dir = str(SCRIPT_DIR / f"mc_{ts}_mix")
    os.makedirs(output_dir, exist_ok=True)
    print(f"\n  Output → {output_dir}\n")

    # Per-case independent, standalone-reproducible RNG: case `cid` is keyed by
    # (seed, cid), so any single case reproduces via default_rng([seed, cid]).
    all_perturbed = {cid: sample_parameters(config["params"],
                                            np.random.default_rng([args.seed, cid]))
                     for cid in range(num_runs)}

    # Per-case wind: the expanded mix spec list (already rescaled to num_runs and,
    # if WIND_MIX_SHUFFLE, deterministically shuffled). Spec i -> case i's wind, so
    # the same WIND_MIX + seed + shuffle flag always yields the same wind per case
    # id (resume-safe).
    all_wind = {cid: wind_params_for_case(case, runway_hdg, spd, side, vspd)
                for cid, (case, spd, side, vspd) in enumerate(wind_specs)}

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
              workspace, args.speedup, args.plot, hard_timeout, params_file,
              all_wind[cid], dt_enabled, sim_parquet, trim_cols, args.keep_aux)
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
                ms = "SETTLED" if res["mission_complete"] else res["land_result"]
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
    print(f"  SUMMARY — uSTOL CRUISE DOUBLET STABILITY (no-DT) + WIND-MIX  ({num_runs} cases)")
    print("=" * 70)
    print(f"  Cases completed  : {n_tot}  (this invocation ran {len(tasks)})")
    print(f"  Takeoff success  : {n_tk}/{n_tot}  "
          f"({100*n_tk/max(n_tot,1):.1f}%)")
    print(f"  Settled to cruise: {n_ms}/{n_tot}  "
          f"({100*n_ms/max(n_tot,1):.1f}%)")
    print(f"  Wall-clock time  : {elapsed:.0f}s  ({elapsed/60:.1f} min)")
    print(f"  Summary CSV      : {summary_path}")

    # Completion marker — a simple text file in the campaign folder saying it's done.
    done_path = os.path.join(output_dir, "DONE.txt")
    with open(done_path, "w") as f:
        f.write("Monte Carlo campaign complete.\n")
        f.write(f"Diff-thrust     : {'ON' if dt_enabled else 'OFF (baseline)'}\n")
        f.write(f"sim_output      : {'Parquet' if sim_parquet else 'CSV'}"
                f"{' (trimmed cols)' if (sim_parquet and trim_cols) else ''}; "
                f"aux files {'kept' if args.keep_aux else 'dropped'}\n")
        f.write(f"Profile         : GROUND->CLIMB->CRUISE->ROLL-DOUBLETS->YAW-DOUBLETS->SETTLE\n")
        f.write(f"Finished        : {datetime.now().isoformat()}\n")
        f.write(f"Wind mix        : {dict(sorted(wind_counts.items()))}\n")
        f.write(f"Wind speeds     : {WIND_SPEED_LEVELS} m/s default\n")
        f.write(f"Cases completed : {n_tot}\n")
        f.write(f"Takeoff success : {n_tk}/{n_tot}\n")
        f.write(f"Settled         : {n_ms}/{n_tot}\n")
        f.write(f"Wall-clock time : {elapsed:.0f}s\n")
        f.write(f"Summary CSV     : {summary_path}\n")
    print(f"  Done marker      : {done_path}")
    print("=" * 70)


if __name__ == "__main__":
    main()
