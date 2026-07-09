#!/usr/bin/env python3
"""
Monte Carlo Runner — uSTOL ALL-FBWA + HYBRID-FLARE LANDING + DIFFERENTIAL-THRUST + EDF-FAILURE + WIND
==================================================================================
Built on aws_monte_carlo_10.py (all-FBWA takeoff→cruise→approach→HYBRID theta(h)
flare landing, NO loiter) and adds the TWO things the Engine_Failure_DT branch
needs, so a single campaign can quantify how the velocity-scheduled DIFFERENTIAL
THRUST mixer (AP_DiffThrust, UST_*) copes with an EDF ENGINE-OUT during a real
landing:

  1) DIFFERENTIAL THRUST (default ON) — each case maps the nine ESC outputs
     SERVO1..9 -> k_motor1..k_motor9 and sets UST_ENABLE=1 (+ the UST_ tune, with
     UST_UMAX=1.0 so takeoff thrust is uncapped). AP_DiffThrust then becomes the
     sole writer of those channels: it reads the stock throttle as a uniform base,
     turns the rudder demand into a THRUST-NEUTRAL antisymmetric per-motor split,
     AND reallocates thrust around any dead EDF (see below). --diff-thrust off
     leaves the outputs on k_throttle (UST_ENABLE=0, stock uniform throttle).

  2) EDF ENGINE-OUT (USTF_MASK) — a per-case FAILURE_MIX declares which EDF(s)
     are dead via the USTF_MASK bitmask (bit e-1 = EDF e failed, e=1..18, port tip
     -> starboard tip). AP_DT_EngineOut turns that into a per-channel thrust-gain
     table that AP_DiffThrust applies on top of its yaw/roll mixer. A dead
     off-centre EDF also GENERATES a yaw asymmetry the autopilot must trim with
     rudder, which is exactly what drives the DT split — so on this profile the
     engine-out itself is the yaw-demand event (no loiter needed). The failure
     scenario + mask are recorded per case in result.json / summary.csv.

     Failure scenarios (whole-channel-out; see FAILURE_SCENARIOS / channel map):
       none          all 18 EDFs alive (healthy baseline)
       outboard_port ch1 (EDF1)      dead — max port-side yaw asymmetry
       outboard_stbd ch9 (EDF18)     dead — max stbd-side yaw asymmetry
       dtch_port     ch3 (EDF4,5)    dead — a DT-split channel lost (port)
       dtch_stbd     ch7 (EDF14,15)  dead — a DT-split channel lost (stbd)
       ailch_port    ch2 (EDF2,3)    dead — aileron-blown channel lost (port)
       ailch_stbd    ch8 (EDF16,17)  dead — aileron-blown channel lost (stbd)

REQUIRES a firmware build with AP_DiffThrust + AP_DT_EngineOut (UST_*/USTF_*
params) and the SIM_Plane.cpp wind wiring. Rebuild with `./waf plane`. DT and the
failure injection are BOTH probed with has_param() so the runner still works on a
tree without those modules (DT off / no failure).

Each case also flies a MIX of wind conditions (unchanged from aws_monte_carlo_10):

    head   wind from straight ahead          (SIM_WIND_DIR = runway heading)
    tail   wind from behind                  (SIM_WIND_DIR = runway heading + 180)
    cross  wind from the side                (SIM_WIND_DIR = runway heading ± 90)
    up     vertical updraft  (rising air)    (SIM_WIND_DIR_Z = +90)
    down   vertical downdraft (sinking air)  (SIM_WIND_DIR_Z = -90)
    none   zero wind (reproduces v4)
    <horiz>+<vert>  COMBINED, e.g. "cross+down" — a horizontal wind AND a
                    vertical wind at once (slanted): SIM_WIND_SPD=hypot(h,v) at
                    elevation SIM_WIND_DIR_Z=atan2(±v,h)

Unlike aws_monte_carlo_6 (ONE wind type per campaign), v7 flies a MIX: edit the
WIND_MIX list in the config block below to give an explicit case COUNT per wind
type (e.g. 4000 cross + 4000 up + 4000 down + ...), so a single 25 000-case run
sweeps every wind type at once.  Each block sweeps its own speed levels; the wind
type/speed/direction are recorded per case in result.json and summary.csv, and
the campaign output dir is tagged "mix".

REQUIRES the SIM_Plane.cpp wiring that copies wind_ef -> vehcle.wind_ned: the
custom uSTOL FDM ignores SIM_WIND_* without it.  Rebuild with `./waf plane`.

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
    FLARE     HYBRID flare (Option B) — PITCH follows the MATLAB height-scheduled
              attitude demand theta(h); THROTTLE still arrests the sink (powered law)
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
    # 1) edit WIND_MIX and FAILURE_MIX below (case + count per type)
    # 2) run a campaign (output dir auto-tagged mc_<ts>_edf):
    python3 aws_monte_carlo_11_edf.py                       # DT ON, EDF failures ON
    python3 aws_monte_carlo_11_edf.py --workers 30 --speedup 10
    python3 aws_monte_carlo_11_edf.py --diff-thrust off     # stock-throttle baseline (no DT)
    python3 aws_monte_carlo_11_edf.py --failures off        # healthy fleet (no engine-out)
    python3 aws_monte_carlo_11_edf.py --runs 25000 --workers 30  # rescale mixes to 25000, keep ratios
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
    {"case": "none",       "count": 5},
    {"case": "head",       "count": 5},
    {"case": "tail",       "count": 5},
    {"case": "cross",      "count": 10, "side": "right"},
    {"case": "cross",      "count": 0, "side": "left"},
    {"case": "up",         "count": 5},
    {"case": "down",       "count": 5},
    # combined horizontal + vertical (vertical magnitude = vert_speed m/s):
    {"case": "cross+down", "count": 5, "side": "right", "vert_speed": 1.0},
    {"case": "head+down",  "count": 5, "vert_speed": 1.0},
    {"case": "tail+up",    "count": 5, "vert_speed": 1.0},
]
WIND_SPEED_LEVELS     = [1.0, 1.5, 2.0] # default m/s HORIZ levels; block case i -> levels[i % len]
WIND_VERT_SPEED       = 1.0             # default vertical component (m/s) for combined cases
                                        # (e.g. "cross+down"); per-block override via "vert_speed"
CROSSWIND_SIDE        = "right"         # default cross side: "right" (hdg+90) | "left" (hdg-90)
WIND_MIX_SHUFFLE      = True            # True -> deterministically (seeded) spread types across
                                        # case ids so ANY prefix of the run is a representative
                                        # sample of the whole mix; False -> blocks in WIND_MIX order
WIND_TURBULENCE       = 0.1             # m/s RMS gusts (0 = steady wind)
WIND_PROFILE_CONSTANT = True            # True  -> SIM_WIND_T=1 (constant w/ altitude)
                                        # False -> SIM_WIND_T=0 (sqrt profile: less near ground)
# ===========================================================================


# ===========================================================================
#  EDF ENGINE-OUT MIX — EDIT THIS BEFORE RUNNING (no CLI flag; --failures on/off)
# ===========================================================================
#  Analogous to WIND_MIX: an ORDERED list of blocks giving a case COUNT per EDF
#  engine-out scenario. Each scenario names one or more whole channels to kill;
#  the runner turns that into the USTF_MASK bitmask (bit e-1 = EDF e dead, e=1..18)
#  and sets it per case before arming. AP_DT_EngineOut then reallocates thrust
#  around the dead EDF(s) and AP_DiffThrust trims the resulting yaw asymmetry.
#
#  uSTOL 9-channel layout (port tip -> starboard tip), channel : EDF numbers:
#    ch1:1  ch2:2,3  ch3:4,5  ch4:6,7  ch5:8,9,10,11  ch6:12,13  ch7:14,15
#    ch8:16,17  ch9:18.   ch3/ch7 are the DT-split channels; ch2/ch8 blow over
#    the ailerons; ch1/ch9 are the single-EDF wingtip channels (max yaw arm).
#  The campaign failure size is rescaled to num_runs (crossed with the wind mix),
#  so these counts set the RATIO of each scenario, not an absolute total.
FAILURE_MIX = [
    {"case": "none",          "count": 10},    # healthy — all 18 EDFs alive
    {"case": "outboard_port", "count": 10},    # ch1 (EDF1) dead
    {"case": "outboard_stbd", "count": 10},    # ch9 (EDF18) dead
    {"case": "dtch_port",     "count": 10},    # ch3 (EDF4,5) dead — DT-split channel
    {"case": "dtch_stbd",     "count": 10},    # ch7 (EDF14,15) dead — DT-split channel
    {"case": "ailch_port",    "count": 10},    # ch2 (EDF2,3) dead — aileron-blown channel
    {"case": "ailch_stbd",    "count": 10},    # ch8 (EDF16,17) dead — aileron-blown channel
]
FAILURE_MIX_SHUFFLE = True   # deterministically spread failure scenarios across case ids,
                             # INDEPENDENTLY of the wind shuffle, so wind x failure combos
                             # are well mixed and any prefix is representative.

# EDF numbers (1-based) carried by each of the nine channels, port tip -> stbd tip.
_EDF_BY_CH = {
    1: (1,), 2: (2, 3), 3: (4, 5), 4: (6, 7), 5: (8, 9, 10, 11),
    6: (12, 13), 7: (14, 15), 8: (16, 17), 9: (18,),
}
# Failure scenario -> list of whole channels to kill. "none" kills nothing.
FAILURE_SCENARIOS = {
    "none":          [],
    "outboard_port": [1],
    "outboard_stbd": [9],
    "dtch_port":     [3],
    "dtch_stbd":     [7],
    "ailch_port":    [2],
    "ailch_stbd":    [8],
}


def edf_mask_for(scenario: str) -> int:
    """USTF_MASK bitmask for a named failure scenario (bit e-1 = EDF e dead)."""
    mask = 0
    for ch in FAILURE_SCENARIOS[scenario.lower()]:
        for e in _EDF_BY_CH[ch]:
            mask |= 1 << (e - 1)
    return mask


def valid_failure_case(c: str) -> bool:
    return str(c).lower() in FAILURE_SCENARIOS


def expand_failure_mix(mix: list) -> list:
    """Expand FAILURE_MIX into a flat ordered list of scenario-name strings
    (one per case); length = sum of block counts."""
    specs = []
    for blk in mix:
        name  = str(blk["case"]).lower()
        count = int(blk.get("count", 0))
        if count > 0:
            specs.extend([name] * count)
    return specs


def rescale_names(specs: list, total: int) -> list:
    """Proportionally resize a list of scenario-name strings to exactly `total`
    entries, preserving each scenario's share (largest-remainder rounding)."""
    n = len(specs)
    if total == n or n == 0:
        return list(specs)
    groups, index = [], {}
    for s in specs:
        if s not in index:
            index[s] = len(groups)
            groups.append([])
        groups[index[s]].append(s)
    raw    = [len(g) / n * total for g in groups]
    floors = [int(math.floor(x)) for x in raw]
    rem    = total - sum(floors)
    order  = sorted(range(len(groups)), key=lambda i: raw[i] - floors[i], reverse=True)
    for j in range(rem):
        floors[order[j % len(order)]] += 1
    out = []
    for g, k in zip(groups, floors):
        out.extend(g[0] for _ in range(k))
    return out


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
DEFAULTS_PARM = USTOL_SIMS / "params_imp_v6_cruise.param"        # ustol_sims/params_imp_v6_cruise.param uSTOL tuned params (v3)
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
    raise FileNotFoundError("Cannot find params_imp_v6_cruise.param or models/plane.parm")


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
# up BOTH the copy off the box and any dashboard build. The columns below are kept
# when --trim-cols is set, to narrow the file ~50% while keeping everything this
# runner's own post-processing needs. Same base list as aws_monte_carlo_10_cruise_dt.py
# / aws_monte_carlo_9_nodt.py's dashboard-column set, PLUS MLG_NR/FLG_NR (main/nose
# landing-gear normal reaction force) which those runners drop as "unused downstream"
# but THIS runner needs: they are the ground truth for the true L=W liftoff instant
# that recompute_ground_roll_from_fdm_csv() below uses (see its docstring). Dropping
# them would silently break that recomputation on any trimmed-Parquet case.
SIM_OUTPUT_KEEP = [
    "Time_s", "plane_moving_state", "TAS_mps", "alt_agl_m", "lat", "lon",
    "phi", "theta", "p", "q", "r",
    "MLG_NR", "FLG_NR",
    "V_b_tas_0", "V_b_tas_2", "V_ned_gnd_0", "V_ned_gnd_1", "V_ned_gnd_2",
    "delta_e", "delta_aL", "delta_r",
    "delta_e_cmd", "delta_aL_cmd", "delta_aR_cmd", "delta_r_cmd",
    "mot0_thr_cmd", "Lift_N", "Drag_N", "total_rotor_force",
]


def convert_sim_output_to_parquet(case_dir: str, trim_cols: bool = False) -> int:
    """Convert each sim_output_*.csv in case_dir to Parquet and delete the CSV.

    Best-effort and never fatal: on ANY error the original CSV is left in place, so a
    case is never lost. With trim_cols, keep only SIM_OUTPUT_KEEP. Returns the number
    of files converted."""
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
            # parquet default (snappy) - the storage win we're after. Falls back to default if
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


def recompute_ground_roll_from_fdm_csv(csv_path: str):
    """Recompute (ground_roll_m, liftoff_speed) from the FDM's own high-rate log using
    the TRUE liftoff instant, instead of the live ~10 Hz MAVLink-telemetry alt>=0.5m
    proxy fly_ustol2() uses during flight to decide when to stop treating the aircraft
    as grounded.

    That proxy is a poor stand-in for the aeronautical definition of ground roll (ends
    at L=W / wheels-off): on this airframe's climb-out the aircraft crosses 0.5m AGL
    roughly 1 SECOND after the wheels actually leave the ground, so the live estimate
    overstates ground_roll_m by ~70-85% (empirically verified against this same FDM log
    across several cases) and understates liftoff_speed by a few percent (airspeed-
    estimator filter lag during the rapid acceleration).

    MLG_NR/FLG_NR (main/nose landing-gear normal reaction force, N) are the FDM's
    ground truth for L=W: both go to exactly zero the instant the gear stops carrying
    any weight. We take the first sample where both are zero AND stay zero for a few
    consecutive samples (so a single-sample zero mid-rotation bounce can't trigger a
    false-early detection), then integrate ground speed (trapezoidal) from when the
    aircraft first starts moving up to that sample for the true ground-roll distance,
    and read TAS_mps at that same sample for the true liftoff speed.

    Returns (ground_roll_m, liftoff_speed), or (None, None) if the CSV is missing,
    unreadable, lacks the needed columns, or the aircraft never left the ground (e.g.
    climb_timeout) - callers should keep their live-telemetry estimate as a fallback.
    """
    try:
        with open(csv_path, newline="") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            return None, None

        def f_(row, key):
            return float(row[key])

        def ground_speed(row):
            return math.hypot(f_(row, "V_ned_gnd_0"), f_(row, "V_ned_gnd_1"))

        liftoff_idx = None
        for i, row in enumerate(rows):
            if f_(row, "MLG_NR") == 0.0 and f_(row, "FLG_NR") == 0.0:
                if all(f_(rows[j], "MLG_NR") == 0.0
                       for j in range(i, min(i + 5, len(rows)))):
                    liftoff_idx = i
                    break
        if liftoff_idx is None:
            return None, None

        start_idx = next((i for i, row in enumerate(rows) if ground_speed(row) > 0.1), 0)

        dist = 0.0
        for i in range(start_idx + 1, liftoff_idx + 1):
            t0, t1 = f_(rows[i - 1], "Time_s"), f_(rows[i], "Time_s")
            v0, v1 = ground_speed(rows[i - 1]), ground_speed(rows[i])
            dist += 0.5 * (v0 + v1) * (t1 - t0)

        return dist, f_(rows[liftoff_idx], "TAS_mps")
    except (KeyError, ValueError, OSError):
        return None, None


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

    Used to make differential thrust + EDF-failure injection OPTIONAL so this
    runner is portable across builds: a tree without AP_DiffThrust / AP_DT_EngineOut
    has no UST_*/USTF_* params at all, and blindly set_param-ing them just burns
    4x3s of retry timeout per case. One PARAM_REQUEST_READ tells us whether to touch
    them. Best-effort: False on any error."""
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
        elif t == "VFR_HUD":
            st["as"]    = m.airspeed
            st["climb"] = m.climb
            st["thr"]   = m.throttle
        elif t == "ATTITUDE":
            st["roll"]    = math.degrees(m.roll)
            st["pitch"]   = math.degrees(m.pitch)
            st["yawrate"] = math.degrees(m.yawspeed)
        elif t == "RAW_IMU":
            # RAW_IMU accel fields are milli-g (mg) in the body frame (z down).
            # Normal load factor n_z = -a_z/g (level/at-rest = +1 g); total g is
            # the specific-force magnitude. Track both for post-processing.
            st["nz"] = -m.zacc / 1000.0
            st["g_total"] = math.sqrt(m.xacc**2 + m.yacc**2 + m.zacc**2) / 1000.0
        elif t == "SERVO_OUTPUT_RAW":
            # ch3/ch7 are the only DT-split channels (_dt_split_ch); their PWM spread
            # is the direct signature of the mixer working. SERVO10 (k_aileron) is
            # the roll-assist engage signal (AP_DiffThrust reads k_aileron's scaled
            # output to compute ail_n). This dialect's SERVO_OUTPUT_RAW carries all
            # 16 servoN_raw fields directly (no port-group splitting).
            st["m3"] = getattr(m, "servo3_raw", st.get("m3", 0))
            st["m7"] = getattr(m, "servo7_raw", st.get("m7", 0))
            st["ail_pwm"] = getattr(m, "servo10_raw", st.get("ail_pwm", 0))
        m = conn.recv_match(type=_PUMP_TYPES, blocking=False)


# ===================================================================
#  Landing-flare pitch schedule (Option B hybrid)
# ===================================================================
# theta(h) attitude demand for the flare, exported from the MATLAB landing-flare solve
# (landing_flare_solve.m) at this profile's approach (V_app=12 m/s, gamma=-7.18 deg,
# flaps=20 deg) and SITL-validated via mission_ustol_11_landing.py (touchdown sink
# 0.70 m/s vs the 1.0 m/s critical limit). The raw curve is the OPEN-LOOP response and
# oscillates (phugoid: +3.4 deg early, dips to -2.6 deg mid-flare, settles +0.8 deg); as a
# closed-loop FBWA demand that dip would command nose-DOWN, so build_flare_table(monotone=True)
# caps it nose-up-only. (h_agl_m, theta_cmd_deg), ground -> flare-init; top = flare-init height.
FLARE_PITCH_SCHED = [
    ( 0.000,   0.819),
    ( 0.342,   0.715),
    ( 0.683,   0.544),
    ( 1.025,   0.315),
    ( 1.366,   0.043),
    ( 1.708,  -0.258),
    ( 2.050,  -0.573),
    ( 2.391,  -0.891),
    ( 2.733,  -1.199),
    ( 3.074,  -1.491),
    ( 3.416,  -1.758),
    ( 3.757,  -1.996),
    ( 4.099,  -2.199),
    ( 4.441,  -2.363),
    ( 4.782,  -2.485),
    ( 5.124,  -2.561),
    ( 5.465,  -2.587),
    ( 5.807,  -2.562),
    ( 6.149,  -2.480),
    ( 6.490,  -2.339),
    ( 6.832,  -2.135),
    ( 7.173,  -1.865),
    ( 7.515,  -1.524),
    ( 7.856,  -1.111),
    ( 8.198,  -0.624),
    ( 8.540,  -0.061),
    ( 8.881,   0.572),
    ( 9.223,   1.264),
    ( 9.564,   1.988),
    ( 9.906,   2.682),
    (10.248,   3.217),
    (10.589,   3.370),
    (10.931,   2.962),
    (11.272,   2.121),
    (11.614,   1.136),
    (11.955,   0.052),
    (12.297,  -1.164),
    (12.639,  -2.163),
    (12.980,  -2.677),
    (13.322,  -2.725),
]


def build_flare_table(sched, monotone=True):
    """Sort the (h, theta) schedule ascending in altitude; optionally cap it nose-up-only as
    the aircraft descends (strips the open-loop phugoid dip so the FBWA demand never commands
    nose-down mid-flare). Returns (H, T) parallel lists."""
    rows = sorted(sched, key=lambda r: r[0])
    H = [r[0] for r in rows]
    T = [r[1] for r in rows]
    if monotone:
        run = -1e9
        for i in range(len(T) - 1, -1, -1):
            run = max(run, T[i])
            T[i] = run
    return H, T


def interp_flare(H, T, h):
    """Linear interpolation of the flare schedule at height h (m AGL)."""
    if h <= H[0]:
        return T[0]
    if h >= H[-1]:
        return T[-1]
    for i in range(1, len(H)):
        if h <= H[i]:
            f = (h - H[i-1]) / (H[i] - H[i-1])
            return T[i-1] + f * (T[i] - T[i-1])
    return T[-1]


def fly_ustol2(conn, m, writer, st, perf, sitl_proc, t0_flight, deadline):
    """Fly the mission_ustol_7 ALL-FBWA profile (takeoff -> cruise -> approach ->
    HYBRID theta(h) flare -> touchdown -> rollout/disarm) via RC overrides. NO loiter —
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
    CRUISE_CAP_THR = u.get("cruise_cap_thr_pwm", 1700)
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
    THR_PWM        = u.get("throttle_climb_pwm", 1850)
    CLIMB_TO       = u.get("climb_timeout_s", 120.0)
    CLIMB_AIRBORNE_ALT = u.get("climb_airborne_alt_m", 2.0)   # "genuinely airborne" gate
    CLIMB_STRIKE_ALT   = u.get("climb_ground_strike_alt_m", 0.3)  # back below this = ground strike

    # --- post-cruise roll/yaw doublets (DT-effectiveness check) ---
    RC1_TRIM       = u.get("rc1_trim", 1500)
    RC1_MAX        = u.get("rc1_max", 2000)
    RC1_MIN        = u.get("rc1_min", 1000)
    RC4_TRIM       = u.get("rc4_trim", 1500)
    RC4_MAX        = u.get("rc4_max", 2000)
    RC4_MIN        = u.get("rc4_min", 1000)
    ROLL_LIMIT_DEG = u.get("roll_limit_deg", 30.0)
    RUD_LIMIT_DEG  = u.get("rud_limit_deg", 40.0)
    YAW_SEQ        = u.get("yaw_seq_deg",  [10.0, -10.0, 20.0, 0.0])
    ROLL_SEQ       = u.get("roll_seq_deg", [10.0, 0.0, -15.0, 0.0, 20.0, 0.0])
    DBL_HOLD_T     = u.get("doublet_hold_s", 4.0)
    SETTLE_MAX_T   = u.get("settle_max_s", 15.0)
    SETTLE_OK_T    = u.get("settle_ok_s", 3.0)
    RECOVER_MAX_T  = u.get("recover_max_s", 15.0)
    ROLL_OK_DEG    = u.get("roll_ok_deg", 3.0)
    YAWRATE_OK     = u.get("yawrate_ok_dps", 5.0)
    BANK_ABORT     = u.get("bank_abort_deg", 45.0)
    DBL_ALT_FLOOR  = u.get("doublet_alt_floor_m", 0.5 * CRUISE_ALT)
    RECOVER_PITCH  = u.get("recover_pitch_deg", -3.0)
    RECOVER_THR    = u.get("recover_thr_pwm", 1500)
    HARD_ABORT_ALT = u.get("doublet_hard_abort_alt_m", 30.0)
    K_ALT_THR      = u.get("doublet_k_alt_thr", 6.0)
    THR_DBL_MIN    = u.get("doublet_thr_min_pwm", 1450)
    THR_DBL_MAX    = u.get("doublet_thr_max_pwm", 1850)

    # --- landing: approach schedule + HYBRID flare (Option B). Defaults below are the SITL-
    #     VALIDATED landing model (mission_ustol_11_landing.py): theta(h) flare schedule + powered
    #     sink-on-throttle -> touchdown sink 0.70 m/s vs the 1.0 m/s critical limit. ---
    V_APP          = u.get("v_app_mps", 12.0)         # approach airspeed held on PITCH
    SINK_APP       = u.get("sink_app_mps", -1.5)      # target approach sink rate (m/s, -=down)
    SINK_TD        = u.get("sink_td_mps", -0.3)       # target touchdown sink rate (m/s)
    FLARE_ALT      = u.get("flare_alt_m", 13.0)      # begin flare below this height (m AGL) = MATLAB S.h_flare (was 8.0)
    TD_ALT         = u.get("td_alt_m", 0.25)          # touchdown when alt drops below this (m) (was 0.5)
    ROLLOUT_T      = u.get("rollout_s", 2.0)          # idle + nose-down on the ground, then disarm
    K_APP_PITCH    = u.get("k_app_pitch", 1.0)        # approach pitch-on-speed gain (deg per m/s)
    APP_PITCH_MIN  = u.get("app_pitch_min_deg", -10.0)
    APP_PITCH_MAX  = u.get("app_pitch_max_deg", 4.0)
    THR_APP_TRIM   = u.get("thr_app_trim_pwm", 1450)  # approach throttle trim (pwm, ~45%)
    K_THR_SINK     = u.get("k_thr_sink", 150.0)       # approach throttle gain: pwm per (m/s) sink err (validated 150, was 80)
    THR_APP_MIN    = u.get("thr_app_min_pwm", 1150)
    THR_APP_MAX    = u.get("thr_app_max_pwm", 1720)
    V_MIN_APP      = u.get("v_min_app_mps", 9.0)      # stall guard: no nose-up below this airspeed
    TH_FLARE_HOLD  = u.get("th_flare_hold_deg", 3.0)  # nose-up fallback used only when USE_FLARE_SCHED is off (deg)
    K_THR_FLARE    = u.get("k_thr_flare", 350.0)      # flare throttle gain: pwm per (m/s) sink err (validated 350, was 110)
    THR_FLARE_MIN  = u.get("thr_flare_min_pwm", 1450) # authority floor (~45%) until touchdown
    THR_FLARE_MAX  = u.get("thr_flare_max_pwm", 1800) # ~80%: let power arrest the sink
    IDLE_THR_PWM   = u.get("idle_thr_pwm", 1000)      # idle throttle for AFTER touchdown only
    APP_TO         = u.get("approach_timeout_s", 120.0)
    FLARE_TO       = u.get("flare_timeout_s", 30.0)
    # HYBRID flare schedule (Option B): PITCH follows the MATLAB theta(h) demand (FLARE_PITCH_SCHED,
    # module level) instead of the flat TH_FLARE_HOLD. Monotone caps the open-loop phugoid dip.
    USE_FLARE_SCHED      = u.get("use_flare_sched", True)
    FLARE_SCHED_MONOTONE = u.get("flare_sched_monotone", True)
    FLARE_RATE_LIM       = u.get("flare_rate_lim_dps", 6.0)   # pitch-rate cap in the flare (deg/s)

    def _smooth(x):
        x = 0.0 if x < 0 else (1.0 if x > 1 else x)
        return x * x * (3.0 - 2.0 * x)

    def theta_target(V):
        if V <= V_R:     return 0.0
        if V <= V_LO:    return TH_LO * _smooth((V - V_R) / (V_LO - V_R))
        if V <= V_CLIMB: return TH_LO + (TH_CLIMB - TH_LO) * _smooth((V - V_LO) / (V_CLIMB - V_LO))
        return TH_CLIMB

    def rate_limit(prev, target, dt, cap=None):
        step = (RATE_LIM if cap is None else cap) * dt
        return max(prev - step, min(target, prev + step))

    # HYBRID flare: build the theta(h) interpolation table once (optionally monotone-capped).
    _FH, _FT = build_flare_table(FLARE_PITCH_SCHED, FLARE_SCHED_MONOTONE)
    def flare_theta(h):
        return interp_flare(_FH, _FT, h)

    def pitch_pwm(theta_deg):
        frac = PITCH_SIGN * theta_deg / PTCH_LIM_MAX
        frac = -1.0 if frac < -1 else (1.0 if frac > 1 else frac)
        return int(RC2_TRIM + frac * (RC2_MAX - RC2_TRIM))

    def roll_pwm(bank_deg):
        frac = max(-1.0, min(1.0, bank_deg / ROLL_LIMIT_DEG))
        span = (RC1_MAX - RC1_TRIM) if frac >= 0 else (RC1_TRIM - RC1_MIN)
        return int(RC1_TRIM + frac * span)

    def rud_pwm(rud_deg):
        frac = max(-1.0, min(1.0, rud_deg / RUD_LIMIT_DEG))
        return int(RC4_TRIM + frac * (RC4_MAX - RC4_TRIM))

    def send_sticks(theta_deg, thr, roll_deg=0.0, rud_deg=0.0):
        # AETR override order: ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw.
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component,
            roll_pwm(roll_deg), pitch_pwm(theta_deg), int(thr), rud_pwm(rud_deg), 0, 0, 0, 0)

    def release_sticks():
        conn.mav.rc_channels_override_send(
            conn.target_system, conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

    _last_csv = [0.0]

    def write_row(phase, theta, roll_cmd=0.0, rud_cmd=0.0):
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
            "%.7f" % (st["lat"] or 0.0), "%.7f" % (st["lon"] or 0.0),
            "%.3f" % st["nz"], "%.3f" % st["g_total"],
            "%d" % st.get("m3", 0), "%d" % st.get("m7", 0),
            "%.2f" % st.get("yawrate", 0.0),
            "%.2f" % roll_cmd, "%.2f" % rud_cmd,
            "%d" % st.get("ail_pwm", 0)])

    start_lat = start_lon = None

    # ---- ground roll + climb + flare ----
    v_peak = 0.0
    theta = 0.0
    flare0 = None
    climbed_airborne = False
    t_climb = time.time()
    t_prev = t_climb
    while True:
        if sitl_proc.poll() is not None:
            return "crashed_climb"
        now = time.time()
        if now > deadline or now - t_climb > CLIMB_TO:
            return "climb_timeout"
        _pump(conn, st)
        # Ground-contact detector: a roll/yaw departure during CLIMB can dive the
        # aircraft back into the ground well before cruise. Without this check the
        # loop just carries on (no ground-contact awareness), climbs back out, and
        # the mission later reports a clean "landed" - masking a genuine crash.
        # Gate on having climbed clearly clear of the ground first, so ordinary
        # liftoff noise near 0m can't false-trigger.
        if st["alt"] >= CLIMB_AIRBORNE_ALT:
            climbed_airborne = True
        elif climbed_airborne and st["alt"] <= CLIMB_STRIKE_ALT:
            return "ground_strike_climb"
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
        if st["alt"] >= CRUISE_ALT - 3.0:
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

    # ---- ROLL/YAW DOUBLETS: DT-effectiveness check, straight after cruise ----
    def _cruise_pitch():
        return max(-5.0, min(TH_CLIMB, K_CAP * (st["as"] - CRUISE_AS)))

    def _dbl_thr():
        return int(max(THR_DBL_MIN, min(CRUISE_CAP_THR + K_ALT_THR * (CRUISE_ALT - st["alt"]), THR_DBL_MAX)))

    def _in_band():
        return (abs(st["roll"]) < ROLL_OK_DEG and abs(st["yawrate"]) < YAWRATE_OK
                and st["alt"] >= DBL_ALT_FLOOR)

    def _departed():
        return abs(st["roll"]) > BANK_ABORT or st["alt"] < DBL_ALT_FLOOR

    def _settle(phase, max_t):
        t0 = time.time(); ok_since = None
        while time.time() - t0 < max_t:
            if sitl_proc.poll() is not None:
                return "crashed_cruise"
            if time.time() > deadline:
                return "deadline"
            _pump(conn, st)
            if st["alt"] < HARD_ABORT_ALT:
                return "ABORT_LOW_ALT"
            send_sticks(_cruise_pitch(), _dbl_thr())
            write_row(phase, _cruise_pitch())
            now = time.time()
            ok_since = ok_since if (_in_band() and ok_since is not None) else (now if _in_band() else None)
            if ok_since is not None and now - ok_since >= SETTLE_OK_T:
                return None
            time.sleep(0.05)
        return None

    def _recover():
        t0 = time.time(); ok_since = None
        while time.time() - t0 < RECOVER_MAX_T:
            if sitl_proc.poll() is not None:
                return "crashed_cruise"
            if time.time() > deadline:
                return "deadline"
            _pump(conn, st)
            if st["alt"] < HARD_ABORT_ALT:
                return "ABORT_LOW_ALT"
            send_sticks(RECOVER_PITCH, RECOVER_THR)
            write_row("RECOVER", RECOVER_PITCH)
            now = time.time()
            ok_since = ok_since if (_in_band() and ok_since is not None) else (now if _in_band() else None)
            if ok_since is not None and now - ok_since >= SETTLE_OK_T:
                return None
            time.sleep(0.05)
        return None

    def _run_doublet_axis(label, targets, axis):
        for sp in targets:
            r = _settle(label, SETTLE_MAX_T)
            if r:
                return r
            if _departed():
                r = _recover()
                if r:
                    return r
            t0 = time.time()
            while time.time() - t0 < DBL_HOLD_T:
                if sitl_proc.poll() is not None:
                    return "crashed_cruise"
                if time.time() > deadline:
                    return "deadline"
                _pump(conn, st)
                if st["alt"] < HARD_ABORT_ALT:
                    return "ABORT_LOW_ALT"
                if axis == "roll":
                    send_sticks(_cruise_pitch(), _dbl_thr(), roll_deg=sp)
                    write_row(label, _cruise_pitch(), roll_cmd=sp)
                else:
                    send_sticks(_cruise_pitch(), _dbl_thr(), rud_deg=sp)
                    write_row(label, _cruise_pitch(), rud_cmd=sp)
                if _departed():
                    r = _recover()
                    if r:
                        return r
                    break
                time.sleep(0.05)
        return None

    r = _run_doublet_axis("YAW_DBL", YAW_SEQ, "yaw")
    if r and r != "ABORT_LOW_ALT":
        return r
    if r is None:
        r = _run_doublet_axis("ROLL_DBL", ROLL_SEQ, "roll")
        if r and r != "ABORT_LOW_ALT":
            return r

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

    # ---- HYBRID FLARE (Option B): PITCH follows the MATLAB height-scheduled attitude demand
    #      theta(h); THROTTLE still arrests the sink around a height-scheduled target (powered
    #      law). Throttle is cut to idle ONLY after touchdown (rollout below). ----
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
        theta_cmd = flare_theta(h) if USE_FLARE_SCHED else TH_FLARE_HOLD   # MATLAB theta(h) demand
        theta     = rate_limit(theta, theta_cmd, dt, cap=FLARE_RATE_LIM)   # follow the scheduled nose-up
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
             max_load_factor_nz=None, min_load_factor_nz=None, max_load_factor_total=0.0,
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
                # Load factor: track peak/min normal n_z and peak total-g (only once
                # airborne, alt>0.5 m, so ground-handling jolts don't dominate).
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
        dt_tag = "diff-thrust ON" if result.get("dt_enabled") else "diff-thrust OFF (baseline)"
        f.write(f"Monte Carlo Case #{case_id:04d}  [uSTOL all-FBWA + landing + EDF-fail; {dt_tag}]\n")
        f.write("=" * 70 + "\n\n")
        f.write(f"Timestamp : {datetime.now().isoformat()}\n")
        f.write(f"Duration  : {result.get('duration_s', 0):.1f} s\n")
        f.write(f"Diff-thrust: {'ON' if result.get('dt_enabled') else 'OFF (baseline)'}\n")
        f.write(f"EDF fail   : {result.get('fail_case', 'none')}  "
                f"USTF_MASK={result.get('edf_mask', 0)}\n")
        f.write(f"Wind      : {result.get('wind_case', 'none')}  "
                f"total {result.get('wind_speed_mps', 0):.1f} m/s  "
                f"(horiz {result.get('wind_horiz_mps', 0):.1f}, "
                f"vert {result.get('wind_vert_mps', 0):+.1f} m/s; "
                f"dir {result.get('wind_dir_deg', 0):.0f}°, "
                f"dirZ {result.get('wind_dir_z_deg', 0):.0f}°)\n\n")

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
        f.write(f"  Load factor n_z   : {result.get('min_load_factor_nz')} .. "
                f"{result.get('max_load_factor_nz')} g  "
                f"(peak |g| {result.get('max_load_factor_total')})\n")
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
              workspace, speedup, save_plot, hard_timeout, params_file, wind,
              dt_enabled=True, fail_case="none",
              sim_parquet=True, trim_cols=False, keep_aux=False,
              roll_assist_enabled=True):
    """Execute one Monte Carlo case: launch SITL, set the case's wind, optionally
    enable differential thrust + inject the case's EDF engine-out (USTF_MASK), fly
    the uSTOL all-FBWA + HYBRID-flare landing profile (no loiter)."""
    fail_case = str(fail_case).lower()
    edf_mask  = edf_mask_for(fail_case)
    result = dict(
        case_id=case_id, case_seed=case_seed,
        dt_enabled=bool(dt_enabled), fail_case=fail_case, edf_mask=edf_mask,
        roll_assist_enabled=bool(roll_assist_enabled),
        takeoff_success=False, mission_complete=False,
        duration_s=0.0, max_alt_m=0.0, land_result="unknown",
        ground_roll_m=None, liftoff_speed=None, landing_roll_m=None,
        exit_reason="unknown",
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
        # SITL stdout is the FDM's per-step debug spam ("Time:..., MLG_NR...") - a few MB of
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
        ]:                                                         # no WP_LOITER_RAD: this profile never loiters
            set_param(conn, n, v)

        # ---- DIFFERENTIAL THRUST: when enabled, map the nine ESC outputs to
        #      k_motor1..k_motor9 and turn on the AP_DiffThrust mixer (UST_*). The mixer
        #      becomes the SOLE writer of those outputs, reading the throttle demand as a
        #      base and allocating a THRUST-NEUTRAL antisymmetric split in thrust space
        #      (inverting the prop quadratic map). UST_UMAX=1.0 keeps takeoff thrust
        #      uncapped. With --diff-thrust off the outputs stay on k_throttle and
        #      UST_ENABLE=0 -> stock uniform throttle. DT is OPTIONAL and probed, so this
        #      runner stays portable to a build WITHOUT AP_DiffThrust (no UST_* params).
        #        UST_NDES_MAX  peak yaw MOMENT [N*m]  (replaces the old UST_DT_KYAW gain)
        #        UST_KRUD > 0  rudder-aware daisy-chain: rudder covers up to KRUD*V^2 [N*m],
        #                      DT supplies only the residual. 0 -> legacy UST_DT_VLO/VHI.
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
                ("UST_ENABLE",   1),
                ("UST_DT_VLO",   ucfg.get("ust_dt_vlo", 13.0)),
                ("UST_DT_VHI",   ucfg.get("ust_dt_vhi", 17.0)),
                ("UST_NDES_MAX", ucfg.get("ust_ndes_max", 20.0)),  # peak yaw MOMENT [N*m]
                ("UST_KRUD",     ucfg.get("ust_krud", 0.11)),      # >0 = rudder-aware daisy-chain
                ("UST_DT_RLFF",  ucfg.get("ust_dt_rlff", 0.0)),
                ("UST_UMAX",     ucfg.get("ust_umax", 1.0)),       # 1.0 = uncapped (SITL takeoff)
                # DT roll assist: when the aileron saturates (e.g. after an aileron-blowing
                # EDF channel fails), DT biases ch3/ch7 thrust to add a rolling moment via
                # blown-lift asymmetry, in the direction the roll controller is demanding.
                # --roll-assist off sets UST_RL_EN=0 (values below become inert).
                ("UST_RL_EN",    1 if roll_assist_enabled else 0),
                ("UST_RL_ENG",   ucfg.get("ust_rl_eng", 0.85)),
                ("UST_RL_MAX",   ucfg.get("ust_rl_max", 2.0)),
                ("UST_RL_VMIN",  ucfg.get("ust_rl_vmin", 8.0)),
            ]:
                set_param(conn, n, v)
            _log(case_id, "differential thrust ENABLED (SERVO1..9=k_motor1..9, UST_ENABLE=1, "
                          f"NDES_MAX={ucfg.get('ust_ndes_max', 20.0)} N*m, KRUD={ucfg.get('ust_krud', 0.11)}, "
                          f"roll-assist={'ON' if roll_assist_enabled else 'OFF'})")
        elif has_ust:
            set_param(conn, "UST_ENABLE", 0)                       # explicit stock baseline (only if present)
        else:
            _log(case_id, "no UST_ params in firmware - stock uniform throttle (no-DT baseline)")

        # ---- EDF ENGINE-OUT: declare this case's dead EDF(s) via USTF_MASK (bit e-1 =
        #      EDF e failed). AP_DT_EngineOut reallocates thrust around them; the resulting
        #      yaw asymmetry is what the DT mixer trims. USTF_OPTS bit0 permits arming with
        #      a non-zero mask (belt-and-braces alongside ARMING_CHECK=0). Optional + probed
        #      so the runner is portable to a build without AP_DT_EngineOut. mask==0 (the
        #      healthy "none" scenario) needs nothing set, but we still zero it if present so
        #      a resumed/shared SITL can never inherit a stale mask. ----
        if edf_mask != 0:
            if not has_param(conn, "USTF_MASK"):
                result["exit_reason"] = "ustf_unavailable"
                _log(case_id, f"ERROR: failure '{fail_case}' requested but USTF_MASK not in "
                              "this firmware (AP_DT_EngineOut not built); aborting case. "
                              "Port AP_DT_EngineOut + ./waf plane, or run --failures off.")
                return result
            set_param(conn, "USTF_OPTS", int(ucfg.get("ustf_opts", 1)))    # bit0: allow arm w/ mask!=0
            set_param(conn, "USTF_MASK", edf_mask)
            _log(case_id, f"EDF engine-out: {fail_case}  USTF_MASK={edf_mask} "
                          f"(chan {FAILURE_SCENARIOS.get(fail_case, [])})")
        elif has_param(conn, "USTF_MASK"):
            set_param(conn, "USTF_MASK", 0)                        # healthy: explicit no-failure

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
        _log(case_id, "armed — flying uSTOL FBWA open-loop profile")

        # Per-case telemetry CSV (drives metrics_from_csv + summary).
        csv_fp = open(case_csv, "w", newline="")
        writer = csv.writer(csv_fp)
        writer.writerow([
            "time_s", "phase", "alt_agl_m", "airspeed", "roll_deg",
            "pitch_deg", "climb_mps", "throttle_pct", "theta_cmd_deg", "lat", "lon",
            "load_factor_nz", "load_factor_total",
            "motor3_pwm", "motor7_pwm", "yaw_rate_dps", "roll_cmd_deg", "rud_cmd_deg",
            "ail_pwm"])

        st = {"alt": 0.0, "as": 0.0, "climb": 0.0, "lat": None, "lon": None,
              "thr": 0.0, "roll": 0.0, "pitch": 0.0, "nz": 1.0, "g_total": 1.0,
              "m3": 0, "m7": 0, "yawrate": 0.0, "ail_pwm": 0}
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

        # Recompute ground_roll_m/liftoff_speed from the FDM's own high-rate log (true
        # L=W liftoff via MLG_NR/FLG_NR) instead of the live ~10Hz-telemetry alt>=0.5m
        # proxy fly_ustol2() used during flight - that proxy overstates ground_roll_m by
        # ~70-85% on this airframe (verified). Must run BEFORE the Parquet conversion
        # below, which deletes the CSV. Falls back to the live perf-based estimate above
        # (already in result["ground_roll_m"]/["liftoff_speed"]) if this fails.
        sim_csvs = sorted(glob.glob(os.path.join(case_dir, "sim_output_*.csv")))
        if sim_csvs:
            gr_true, vlof_true = recompute_ground_roll_from_fdm_csv(sim_csvs[0])
            if gr_true is not None:
                result["ground_roll_m"] = gr_true
            if vlof_true is not None:
                result["liftoff_speed"] = vlof_true

        # sim_output -> Parquet (the high-value reduction: sim_output is ~85% of a case's
        # bytes). Done AFTER the CSV is in case_dir and the ground-roll recompute above has
        # read it; best-effort (leaves the CSV on any error). --sim-format csv skips this.
        if sim_parquet:
            convert_sim_output_to_parquet(case_dir, trim_cols=trim_cols)

        # Lean output (6->3 files): drop the per-case files nothing downstream reads.
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

        # Human-readable per-case report - nothing downstream reads it, so write it only
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
     speedup, save_plot, hard_timeout, params_file, wind, dt_enabled, fail_case,
     sim_parquet, trim_cols, keep_aux, roll_assist_enabled) = args
    inst = _instance_queue.get()
    try:
        return _run_case(case_id, case_seed, inst, config, perturbed, output_dir,
                         workspace, speedup, save_plot, hard_timeout, params_file, wind,
                         dt_enabled, fail_case, sim_parquet, trim_cols, keep_aux,
                         roll_assist_enabled)
    finally:
        _instance_queue.put(inst)


# ===================================================================
#  Summary output
# ===================================================================
# Frozen output schema — DO NOT add/remove columns after a campaign starts
# (re-deriving metrics from thousands of logs afterward is painful).
SUMMARY_COLUMNS = [
    "case_id", "case_seed", "dt_enabled", "fail_case", "edf_mask", "roll_assist_enabled",
    "takeoff_success", "mission_complete", "land_result",
    "exit_reason", "duration_s",
    "wind_case", "wind_speed_mps", "wind_horiz_mps", "wind_vert_mps",
    "wind_dir_deg", "wind_dir_z_deg",
    "ground_roll_m", "liftoff_speed", "landing_roll_m",
    "max_alt_m", "max_roll_deg", "max_pitch_deg", "min_airspeed", "max_airspeed",
    "max_load_factor_nz", "min_load_factor_nz", "max_load_factor_total",
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
        description="Monte Carlo Runner — uSTOL all-FBWA + landing + DT + EDF-failure + WIND. "
                    "Edit WIND_MIX at the top of this file to set the per-type case counts.")
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
    parser.add_argument("--diff-thrust", choices=["on", "off"], default="on",
                        help="'on' (default): map SERVO1..9 -> k_motor1..9 + UST_ENABLE=1 "
                             "(UST_UMAX=1.0), so AP_DiffThrust reallocates around dead EDFs "
                             "and trims the yaw. 'off': stock uniform throttle (UST_ENABLE=0) "
                             "- the no-DT baseline.")
    parser.add_argument("--failures", choices=["on", "off"], default="on",
                        help="'on' (default): inject the FAILURE_MIX EDF engine-outs per case "
                             "via USTF_MASK. 'off': healthy fleet (all cases fly 'none', mask=0).")
    parser.add_argument("--roll-assist", choices=["on", "off"], default="on",
                        help="'on' (default): UST_RL_EN=1 - when the aileron saturates (e.g. "
                             "after ailch_port/ailch_stbd), DT biases ch3/ch7 thrust to add a "
                             "rolling moment via blown-lift asymmetry. 'off': UST_RL_EN=0, the "
                             "pre-roll-assist baseline. Run both to see whether it fixes the "
                             "aileron-channel climb-phase departures.")
    parser.add_argument("--sim-format", choices=["parquet", "csv"], default="parquet",
                        help="per-case sim_output format. 'parquet' (default): convert the FDM "
                             "CSV to Parquet (~85%% of a case's bytes, ~3-7x smaller, faster to "
                             "load + copy). 'csv': keep the raw CSV (e.g. for plot_case_full.py).")
    parser.add_argument("--trim-cols", action="store_true",
                        help="when writing Parquet, keep only the SIM_OUTPUT_KEEP columns "
                             "(~50%% narrower), including MLG_NR/FLG_NR so the true-liftoff "
                             "ground-roll recomputation stays auditable from the saved Parquet. "
                             "Off by default (keep all columns) so nothing downstream can "
                             "silently break.")
    parser.add_argument("--keep-aux", action="store_true",
                        help="keep the per-case files nothing downstream reads (overrides.txt, "
                             "report.txt, sitl_stdout.log). Default: drop them (6->3 files/case) "
                             "and send SITL stdout to /dev/null (no serial-port spam).")
    args = parser.parse_args()
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
    # FAILURE MIX: validate every block (skipped entirely when --failures off).
    dt_enabled          = (args.diff_thrust == "on")
    failures_enabled    = (args.failures == "on")
    roll_assist_enabled = (args.roll_assist == "on")
    for blk in FAILURE_MIX:
        if not valid_failure_case(blk.get("case", "")):
            print(f"\nERROR: FAILURE_MIX case={blk.get('case')!r} invalid; use one of "
                  f"{sorted(FAILURE_SCENARIOS)}")
            sys.exit(1)
        if int(blk.get("count", 0)) < 0:
            print(f"\nERROR: FAILURE_MIX count must be >= 0 (block {blk})")
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

    # EDF FAILURE per case: expand FAILURE_MIX -> flat scenario list, rescale to
    # num_runs, and shuffle INDEPENDENTLY of the wind (seed+1) so wind x failure
    # combos are well mixed and resume-safe. --failures off -> every case healthy.
    if failures_enabled:
        fail_specs = rescale_names(expand_failure_mix(FAILURE_MIX), num_runs)
        if not fail_specs:
            print("\nERROR: FAILURE_MIX expands to 0 cases — set some non-zero counts "
                  "or run --failures off.")
            sys.exit(1)
        if FAILURE_MIX_SHUFFLE and num_runs > 1:
            order = np.random.default_rng(args.seed + 1).permutation(len(fail_specs))
            fail_specs = [fail_specs[i] for i in order]
    else:
        fail_specs = ["none"] * num_runs
    fail_counts = {}
    for fc in fail_specs:
        fail_counts[fc] = fail_counts.get(fc, 0) + 1

    # Worker precedence: explicit --workers > auto-size to this machine.
    max_workers = args.workers or auto_workers()
    # Never spin up more SITL instances than there are cases to run.
    max_workers = max(1, min(max_workers, num_runs))
    base_inst   = args.base_instance
    hard_timeout = (args.hard_timeout
                    or config["mission"].get("hard_timeout_s", 420))

    print("=" * 70)
    print("  LAT Monte Carlo Runner — uSTOL all-FBWA + landing + DT + EDF-FAILURE + WIND")
    print("=" * 70)
    print(f"  Diff-thrust   : {'ON (SERVO1..9=k_motor1..9, UST_ENABLE=1, UMAX=1.0)' if dt_enabled else 'OFF (stock uniform throttle - no-DT baseline)'}")
    print(f"  Roll assist   : {'ON (UST_RL_EN=1)' if roll_assist_enabled else 'OFF (UST_RL_EN=0, pre-roll-assist baseline)'}")
    print(f"  EDF failures  : {'ON (USTF_MASK per FAILURE_MIX)' if failures_enabled else 'OFF (healthy fleet, mask=0)'}")
    print(f"  Output        : sim_output={'Parquet' if sim_parquet else 'CSV'}"
          f"{' (trimmed cols)' if (sim_parquet and trim_cols) else ''}; "
          f"aux files {'kept' if args.keep_aux else 'dropped (6->3/case, no SITL-stdout spam)'}")
    for k in sorted(fail_counts):
        print(f"      {k:<14}: {fail_counts[k]}")
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
        # Tag as a mixed-wind campaign.
        output_dir = str(SCRIPT_DIR / f"mc_{ts}_edf")
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

    # Per-case EDF failure scenario name (same resume-safe indexing as wind).
    all_fail = {cid: fail_specs[cid] for cid in range(num_runs)}

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
              all_wind[cid], dt_enabled, all_fail[cid],
              sim_parquet, trim_cols, args.keep_aux, roll_assist_enabled)
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
    print(f"  SUMMARY — uSTOL ALL-FBWA + LANDING + DT + EDF-FAILURE CAMPAIGN  ({num_runs} cases)")
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
        f.write(f"Diff-thrust     : {'ON' if dt_enabled else 'OFF (baseline)'}\n")
        f.write(f"Roll assist     : {'ON' if roll_assist_enabled else 'OFF (baseline)'}\n")
        f.write(f"EDF failures    : {'ON' if failures_enabled else 'OFF (healthy)'}  "
                f"{dict(sorted(fail_counts.items()))}\n")
        f.write(f"sim_output      : {'Parquet' if sim_parquet else 'CSV'}"
                f"{' (trimmed cols)' if (sim_parquet and trim_cols) else ''}; "
                f"aux files {'kept' if args.keep_aux else 'dropped'}\n")
        f.write(f"Wind mix        : {dict(sorted(wind_counts.items()))}\n")
        f.write(f"Wind speeds     : {WIND_SPEED_LEVELS} m/s default\n")
        f.write(f"Cases completed : {n_tot}\n")
        f.write(f"Takeoff success : {n_tk}/{n_tot}\n")
        f.write(f"Landed (td)     : {n_ms}/{n_tot}\n")
        f.write(f"Wall-clock time : {elapsed:.0f}s\n")
        f.write(f"Summary CSV     : {summary_path}\n")
    print(f"  Done marker      : {done_path}")
    print("=" * 70)


if __name__ == "__main__":
    main()
