#!/usr/bin/env python3

"""
uSTOL mission driver (connects to an already-running SITL) - FULL MISSION + DIFFERENTIAL THRUST.

Single-run, interactive sibling of the aws_monte_carlo_8.py campaign: it flies the SAME full
profile for ONE aircraft against a SITL you launch yourself, so you can watch it on a map/GCS.
Everything is flown in FBWA via streamed RC overrides (the mission_ustol_2 idiom) EXCEPT the
loiter, which is a real autopilot nav mode.

Sequence (mission_ustol_8 profile + differential thrust):
  0) claim GCS sysid + headless-arm params + loiter radius, and (DIFF_THRUST) map the nine ESC
     outputs SERVO1..9 -> k_motor1..k_motor9 and enable the AP_DiffThrust mixer (UST_*).
  1) arm in FBWA
  2) open-loop takeoff + climb: airspeed-scheduled, rate-limited pitch with airspeed protection
     and an AoA guard; exponential level-off to CRUISE_ALT (arrive level at cruise throttle)
  3) CRUISE hold: open-loop level hold (speed-on-pitch + cruise throttle) for CRUISE_HOLD_T s
  4) LOITER: release the overrides, switch the autopilot to LOITER (circle at WP_LOITER_RAD)
     for LOITER_T s, then hand back to FBWA. This is the ONE navigated phase - and the phase
     where the autopilot commands rudder, so it is where DIFFERENTIAL THRUST does work: the
     mixer turns the rudder demand into an antisymmetric per-motor split (watch motor1 vs
     motor9 in the LOITER print).
  5) APPROACH (FBWA, decoupled): PITCH holds the approach speed (speed-on-pitch), THROTTLE
     holds the sink rate (sink-on-throttle), down to FLARE_ALT
  6) HYBRID FLARE (Option B): the elevator (PITCH) follows a HEIGHT-SCHEDULED attitude demand
     theta(h) lifted from the MATLAB landing-flare solve (landing_flare_solve.m), while THROTTLE
     still arrests the sink around a height-scheduled target (the v9 powered law, unchanged).
     Touchdown, then idle rollout + DISARM.

THIS FILE = mission_ustol_9 with ONLY the flare (step 6) changed to the Option B hybrid; every
other phase (takeoff/climb/cruise/loiter/approach/rollout/diff-thrust) is byte-identical to v9.

LANDING FLARE SCHEDULE (Option B hybrid): FLARE_PITCH_SCHED below is theta(h) exported from
landing_flare_solve.m, solved at THIS profile's approach (V_app=12 m/s, gamma=-7.18 deg,
flaps=20 deg). That solve also moves the flare-init height to FLARE_ALT=13.32 m (was 8 m). The
raw theta(h) is the OPEN-LOOP dynamic response and OSCILLATES (phugoid: +3.4 deg early, dips to
-2.6 deg mid-flare, settles +0.8 deg); fed as a closed-loop FBWA demand its mid-flare dip would
briefly command nose-DOWN, so by default (FLARE_SCHED_MONOTONE=True) the schedule is capped
nose-up-only. Set USE_FLARE_SCHED=False to fall back to the v9 constant-nose-up flare, or
FLARE_SCHED_MONOTONE=False to command the raw oscillating curve.

DIFFERENTIAL THRUST: requires a build with the AP_DiffThrust mixer (branch Sushanth_SITL_v2-dThrust).
When DIFF_THRUST=True the nine ESCs are driven by k_motor1..9 and the throttle stick reaches them
through the mixer's uniform base; in the straight FBWA phases the yaw demand is ~0 so the split is
inert (uniform throttle), and only the loiter turn produces a split. UST_UMAX=1.0 keeps takeoff
thrust uncapped (the validated bench overlay's 0.4619 wire-cap would peg throttle at ~46% and
prevent takeoff). Set DIFF_THRUST=False to fly the identical profile with stock uniform throttle
(UST_ENABLE=0) as a baseline.

Run SITL first (separate terminal), e.g.:
  LAT_SIM_LOG_DIR="ustol_sims/logs/" build/sitl/bin/arduplane -w --model plane \
      --defaults ustol_sims/params_imp_v3.param --home 28.559741,77.11745,237,285 -I0
and have one GCS (MAVProxy/Mission Planner) on 5760 so this script can use 5762.

The FDM writes its per-run time-series CSV to ustol_sims/logs/. Plot the newest with:
  python3 Tools/autotest/monte_carlo/plot_case_full.py ustol_sims/logs
"""

from pymavlink import mavutil
import math, time, atexit

_plotted = False
def _plot_on_exit():
    global _plotted
    if _plotted:
        return
    _plotted = True
    try:
        import plot_ustol, fft_pitch                              # same folder -> importable
        print("case plot:", plot_ustol.plot(""), flush=True)      # "" = newest CSV
        print("fft  plot:", fft_pitch.analyze(""), flush=True)
    except Exception as e:
        print("plotting failed:", e, flush=True)
atexit.register(_plot_on_exit)


# A. INPUT PARAMS
HOME_LAT, HOME_LON = 28.559741, 77.11745
RWY_HDG       = 285.0                          # informational (FBWA holds wings level, no nav)
CRUISE_ALT    = 100.0                          # level off / start cruise hold at this alt (m AGL)
CRUISE_AS     = 12.0                           # cruise airspeed after level-off
CRUISE_CAP_THR = 1660                          # FBWA throttle (pwm) for ~12 m/s LEVEL during cruise hold
K_CAP          = 3.0                           # cruise pitch-on-speed gain: theta = K_CAP*(V-CRUISE_AS)
CRUISE_HOLD_T  = 30.0                          # hold level cruise (open-loop) this many s, then LOITER
LEVEL_BAND     = 10.0                          # start the level-off this far below CRUISE_ALT (m)
LEVEL_K        = 0.5                           # exp level-off rate (1/m): blend = e^(-K*|alt-CRUISE_ALT|)

# LOITER (autopilot nav circle), after the cruise hold and before the landing:
LOITER_RAD    = 200.0                          # WP_LOITER_RAD set on the AP: circle radius (m)
LOITER_T      = 30.0                           # autopilot LOITER duration (s); ~1 lap at 150 m / 12 m/s

# Landing (open-loop FBWA: approach schedule + POWERED flare), after the loiter:
V_APP         = 12.0                           # approach airspeed held on PITCH (speed-on-pitch)
SINK_APP      = -1.5                           # target approach sink rate (m/s; negative = descending)
SINK_TD       = -0.3                           # target touchdown sink rate (m/s) at the ground
FLARE_ALT     = 13.32                          # begin the flare below this height (m AGL) = MATLAB S.h_flare (was 8.0)
TD_ALT        = 0.25                            # touchdown detected when alt drops below this (m AGL)
ROLLOUT_T     = 4.0                            # hold idle + slight nose-down on the ground, then disarm
K_APP_PITCH   = 1.0                            # approach pitch-on-speed gain (deg per m/s)
APP_PITCH_MIN, APP_PITCH_MAX = -10.0, 4.0      # approach pitch clamp (deg)
THR_APP_TRIM  = 1450                           # approach throttle trim (pwm, ~45%); below cruise so it descends
K_THR_SINK    = 150.0                           # approach throttle gain: pwm per (m/s) of sink error
THR_APP_MIN, THR_APP_MAX = 1150, 1720          # approach throttle clamp (pwm)
V_MIN_APP     = 9.0                            # stall guard: never command nose-up below this airspeed

# POWERED flare: throttle arrests the sink, elevator only sets a modest achievable nose-up.
TH_FLARE_HOLD = 3.0                            # modest nose-up held in the flare (deg)
K_THR_FLARE   = 350.0                          # flare throttle gain: pwm per (m/s) of sink error
THR_FLARE_MIN = 1450                           # AUTHORITY FLOOR (~45%): throttle never drops below this until touchdown
THR_FLARE_MAX = 1800                           # ~80%: let power arrest the sink
IDLE_THR_PWM  = 1000                           # idle throttle for AFTER touchdown (rollout) only

# HYBRID flare (Option B): height-scheduled pitch demand theta(h) from the MATLAB landing-flare
# solve (landing_flare_solve.m), exported at this profile's approach. See the module docstring.
USE_FLARE_SCHED      = True   # True: PITCH follows theta(h) below; False: v9 constant TH_FLARE_HOLD
FLARE_SCHED_MONOTONE = True   # cap the schedule nose-up-only (strip the open-loop phugoid dip)
FLARE_RATE_LIM       = 6.0    # deg/s pitch-rate cap in the flare (MATLAB peak |q| was 4.9 deg/s)
# (h_agl_m, theta_cmd_deg), ground -> flare-init. Touchdown solve: V=10.97 m/s, sink=1.17 m/s,
# gamma=-6.14 deg, alpha=6.96 deg, n_peak=1.062. Top of table = FLARE_ALT = MATLAB S.h_flare.
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

# Differential thrust (AP_DiffThrust / UST_ mixer). DIFF_THRUST=False -> stock uniform throttle.
DIFF_THRUST   = True                           # enable the velocity-scheduled diff-thrust yaw mixer
UST_DT_VLO    = 13.0                           # m/s, full differential authority at/below
UST_DT_VHI    = 17.0                           # m/s, zero authority at/above (must exceed VLO)
UST_NDES_MAX  = 20.0                           # peak yaw MOMENT [N*m] at full rudder (was UST_DT_KYAW gain)
UST_KRUD      = 0.11                            # >0 = rudder-aware daisy-chain (new); 0 = legacy VLO/VHI schedule
UST_DT_RLFF   = 0.0                            # aileron roll feedforward (0 = off)
UST_UMAX      = 1.0                            # per-motor command ceiling; 1.0 = uncapped (SITL takeoff)
K_MOTOR_FN    = [33, 34, 35, 36, 37, 38, 39, 40, 82]   # SERVO1..9 -> k_motor1..k_motor9 functions

V_R, V_LO, V_CLIMB = 6.0, 8.0, 10.0            # rotate / liftoff / climb-established speeds (m/s)
TH_LO, TH_CLIMB    = 7.0, 16.5                 # pitch at liftoff / in the climb (deg)
RATE_LIM_DPS       = 5.0                       # hard cap on commanded pitch rate q (deg/s)
FPA_TARGET         = 15.0                      # design flight-path angle (TH_CLIMB tuned to hit it)
AOA_CAP            = 7.0                       # max AoA: cmd pitch <= measured_FPA + AOA_CAP
V_HOLD             = 10.0                      # climb airspeed to protect (m/s); bleed pitch if slower
K_SPD              = 1.0                       # deg of pitch backed off per m/s below V_HOLD
GND_PITCH          = -2.0                      # nose-down hold during ground roll (V < V_R)

RC2_TRIM, RC2_MAX  = 1500, 2000                # elevator INPUT channel endpoints
PTCH_LIM_MAX       = 20.0                      # must equal PTCH_LIM_MAX_DEG set below (>= TH_CLIMB)
PITCH_SIGN         = -1                        # +cmd must pitch nose UP; flip to -1 if it dives
THR_PWM            = 1900                      # 90% throttle on RC3 (climb)
ELEV_MIN, ELEV_MAX = 1100, 1900               # SERVO11 (elevator, k_elevator) output endpoints; used for the saturation flag

# B. CONNECTING TO MAVLINK
c = mavutil.mavlink_connection("tcp:127.0.0.1:5762", source_system=250)
# Lock onto the AUTOPILOT's heartbeat (ignore any GCS heartbeat on the link) and pin
# tracking to it, so c.flightmode reads the AP's mode instead of None.
while True:
    hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if hb is None or hb.type != mavutil.mavlink.MAV_TYPE_GCS:
        break
if hb is not None:
    c.target_system = hb.get_srcSystem()
    c.sysid = hb.get_srcSystem()
c.mav.request_data_stream_send(
    c.target_system, c.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
# Best-effort: ask for AOA_SSA (true angle of attack) at 10 Hz so the console can show it.
c.mav.command_long_send(
    c.target_system, c.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 11020, 100000, 0, 0, 0, 0, 0)
print("connected", flush=True)

# C. HELPER FUNCTIONS
def set_param(name, value):
    """Set one parameter and wait for the autopilot to echo it back (confirmation)."""
    for _ in range(4):
        c.mav.param_set_send(c.target_system, c.target_component,
                             name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ack = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        if ack and ack.param_id.replace('\x00', '') == name:
            return True
    return False

def set_mode(name):
    """Switch flight mode by name (e.g. 'FBWA') and confirm via heartbeat."""
    mid = c.mode_mapping()[name]
    for _ in range(3):
        c.set_mode(mid); time.sleep(0.3)
    t = time.time()
    while time.time() - t < 12:
        hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.custom_mode == mid:
            return True
        c.set_mode(mid)
    return False

def arm():
    """Arm the vehicle; falls back to force-arm after a few tries."""
    t = time.time(); n = 0
    while time.time() - t < 40:
        n += 1
        force = 21196 if n > 4 else 0
        c.mav.command_long_send(c.target_system, c.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, force, 0, 0, 0, 0, 0)
        for _ in range(5):
            hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        time.sleep(1.5)
    return False

def disarm():
    """Disarm the vehicle after touchdown; falls back to force-disarm after a few tries."""
    t = time.time(); n = 0
    while time.time() - t < 20:
        n += 1
        force = 21196 if n > 3 else 0
        c.mav.command_long_send(c.target_system, c.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, force, 0, 0, 0, 0, 0)
        for _ in range(5):
            hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        time.sleep(1.0)
    return False

st = {"alt": 0.0, "as": 0.0, "climb": 0.0, "lat": None, "lon": None, "thr": 0.0, "thr_pwm": 0,
      "pitch": 0.0, "yawrate": 0.0, "elev": 1500, "aoa": 0.0, "m1": 0, "m9": 0}
_PUMP_TYPES = ["GLOBAL_POSITION_INT", "VFR_HUD", "RC_CHANNELS", "ATTITUDE", "SERVO_OUTPUT_RAW", "AOA_SSA"]
def pump():
    """Drain ALL buffered telemetry each call and keep the LATEST values.

    CRITICAL: read the whole buffer, not one message. The AP streams faster than this loop
    consumes, so reading a single message per call lets the socket buffer back up and every
    recv_match returns an OLDER message -> st["alt"]/as lag tens of seconds behind real time
    and the flare/altitude triggers never fire.
    """
    got = None
    m = c.recv_match(type=_PUMP_TYPES, blocking=True, timeout=2)
    while m is not None:
        got = m
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            st["alt"] = m.relative_alt / 1000.0
            st["lat"] = m.lat / 1e7
            st["lon"] = m.lon / 1e7
        elif t == "VFR_HUD":
            st["as"]    = m.airspeed
            st["climb"] = m.climb
            st["thr"]   = m.throttle
        elif t == "RC_CHANNELS":
            st["thr_pwm"] = m.chan3_raw
        elif t == "ATTITUDE":
            st["pitch"]   = math.degrees(m.pitch)       # MEASURED pitch (deg)
            st["yawrate"] = math.degrees(m.yawspeed)    # body yaw rate (deg/s)
        elif t == "SERVO_OUTPUT_RAW":
            # SERVO1 (k_motor1, port outer) & SERVO9 (k_motor9, starboard outer): their split is
            # the direct signature of differential thrust during the loiter. The elevator is
            # SERVO11 (k_elevator, fn 19); SERVO14 is the FLAP (fn 2) and sits dead at 1100 — reading
            # it as the elevator pinned ELEV_SAT True on a constant-1100 channel (false positive).
            st["m1"]   = getattr(m, "servo1_raw", st["m1"])
            st["m9"]   = getattr(m, "servo9_raw", st["m9"])
            st["elev"] = getattr(m, "servo11_raw", st["elev"])
        elif t == "AOA_SSA":
            st["aoa"] = getattr(m, "AOA", st["aoa"])    # true angle of attack (deg), if streamed
        m = c.recv_match(type=_PUMP_TYPES, blocking=False)
    return got

def wait_ekf_ready(timeout=40):
    """Block until the EKF is healthy + GPS-aided so its attitude re-alignment happens on the
    GROUND, not mid-climb."""
    need = 0x01 | 0x02 | 0x10           # EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_ABS
    t = time.time()
    while time.time() - t < timeout:
        s = c.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        if s and (s.flags & need) == need:
            return True
    return False

# Pitch related functions in FBWA
def _smooth(x):
    """smoothstep 0->1 (C1 continuous)."""
    x = 0.0 if x < 0 else (1.0 if x > 1 else x)
    return x * x * (3 - 2 * x)

def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

# --- HYBRID flare schedule (Option B): build the interp table from FLARE_PITCH_SCHED ---
_FLARE     = sorted(FLARE_PITCH_SCHED, key=lambda r: r[0])     # ascending altitude
_FLARE_H   = [r[0] for r in _FLARE]
_FLARE_TH  = [r[1] for r in _FLARE]
if FLARE_SCHED_MONOTONE:
    # Raw theta(h) is the MATLAB OPEN-LOOP response and oscillates (phugoid). As a closed-loop
    # FBWA demand its mid-flare dip would briefly command nose-DOWN, so enforce nose-up-only as
    # the aircraft descends: sweep flare-init -> ground, holding the running peak.
    _run = -1e9
    for _i in range(len(_FLARE_TH) - 1, -1, -1):
        _run = max(_run, _FLARE_TH[_i])
        _FLARE_TH[_i] = _run

def flare_theta(h):
    """MATLAB-scheduled flare pitch DEMAND (deg) at height h (m AGL); linear interp on the table."""
    if h <= _FLARE_H[0]:
        return _FLARE_TH[0]
    if h >= _FLARE_H[-1]:
        return _FLARE_TH[-1]
    for i in range(1, len(_FLARE_H)):
        if h <= _FLARE_H[i]:
            f = (h - _FLARE_H[i-1]) / (_FLARE_H[i] - _FLARE_H[i-1])
            return _FLARE_TH[i-1] + f * (_FLARE_TH[i] - _FLARE_TH[i-1])
    return _FLARE_TH[-1]

def elev_sat():
    """True when the elevator servo output is within ~20us of a stop = out of pitch authority."""
    e = st["elev"]
    return e >= ELEV_MAX - 20 or 0 < e <= ELEV_MIN + 20

def theta_target(V):
    """V-scheduled TARGET pitch (deg) for the takeoff/climb. Rate-limited before commanded."""
    if V <= V_R:     return 0.0
    if V <= V_LO:    return TH_LO * _smooth((V - V_R) / (V_LO - V_R))
    if V <= V_CLIMB: return TH_LO + (TH_CLIMB - TH_LO) * _smooth((V - V_LO) / (V_CLIMB - V_LO))
    return TH_CLIMB

def rate_limit(prev, target, dt, cap=RATE_LIM_DPS):
    """Slew-limit so |d(theta)/dt| <= cap deg/s (the commanded pitch-rate cap)."""
    step = cap * dt
    return max(prev - step, min(target, prev + step))

def pitch_pwm(theta_deg):
    """Map a demanded pitch angle (deg) to the FBWA elevator-input PWM."""
    frac = PITCH_SIGN * theta_deg / PTCH_LIM_MAX
    frac = -1.0 if frac < -1 else (1.0 if frac > 1 else frac)
    return int(RC2_TRIM + frac * (RC2_MAX - RC2_TRIM))

def send_sticks(theta_deg, thr=None):
    """Wings level + neutral rudder + throttle + commanded pitch. CALL EVERY LOOP
    (override must stream continuously or the RC-loss failsafe trips). With diff thrust on,
    the throttle channel reaches the nine k_motor ESCs through the mixer's uniform base."""
    c.mav.rc_channels_override_send(
        c.target_system, c.target_component,
        1500,
        pitch_pwm(theta_deg),
        THR_PWM if thr is None else thr,
        1500,
        0, 0, 0, 0)

def release_sticks():
    """Send 0 on all channels = release override so LOITER/TECS takes the sticks."""
    c.mav.rc_channels_override_send(c.target_system, c.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

# D. MISSION PROFILE  (FBWA takeoff + climb, CRUISE, LOITER, approach + POWERED flare; diff thrust)

# 0. Params: headless arm, pitch ceiling above the climb attitude, low-speed floor, loiter radius,
#    and (when DIFF_THRUST) the k_motor servo map + UST_ mixer tune.
gcs_ok = set_param("SYSID_MYGCS", 250) or set_param("MAV_GCS_SYSID", 250)
print(f"claim GCS sysid=250 (so RC overrides are accepted): {gcs_ok}", flush=True)
for n, v in [
    ("ARMING_CHECK",     0),          # allow headless arming
    ("AIRSPEED_MIN",     7),          # liftoff sits above the floor (not flagged)
    ("PTCH_LIM_MAX_DEG", 20),         # >= TH_CLIMB so FBWA won't clamp the climb pitch
    ("PTCH_LIM_MIN_DEG", -20),        # allow nose-down for the glide
    ("TECS_PITCH_MAX",   20),
    ("AIRSPEED_CRUISE",  CRUISE_AS),  # TECS cruise speed (used by LOITER)
    ("WP_LOITER_RAD",    LOITER_RAD), # LOITER circle radius (m)
]:
    print(f"set {n}={v}: {set_param(n, v)}", flush=True)

if DIFF_THRUST:
    # Map the nine ESC outputs to k_motor1..k_motor9 and enable the AP_DiffThrust mixer. The mixer
    # becomes the SOLE writer of those outputs (throttle reaches them via its uniform base, rudder
    # via the antisymmetric split). UST_UMAX=1.0 keeps takeoff thrust uncapped.
    for ch, fn in enumerate(K_MOTOR_FN, start=1):
        set_param(f"SERVO{ch}_FUNCTION", fn)
    for n, v in [
        ("UST_ENABLE",  1),
        ("UST_DT_VLO",  UST_DT_VLO),
        ("UST_DT_VHI",  UST_DT_VHI),
        ("UST_NDES_MAX", UST_NDES_MAX),
        ("UST_KRUD",    UST_KRUD),
        ("UST_DT_RLFF", UST_DT_RLFF),
        ("UST_UMAX",    UST_UMAX),
    ]:
        print(f"set {n}={v}: {set_param(n, v)}", flush=True)
    print("-> DIFFERENTIAL THRUST ON (SERVO1..9=k_motor1..9, UST_ENABLE=1)", flush=True)
else:
    set_param("UST_ENABLE", 0)
    print("-> DIFFERENTIAL THRUST OFF (stock uniform throttle baseline)", flush=True)

# 1. Arm in FBWA. No mission: throttle + pitch come from the RC override stream below.
print("FBWA:",  set_mode("FBWA"), flush=True)
print("EKF ready:", wait_ekf_ready(), flush=True)   # let GPS-aided alignment finish on the GROUND
print("armed:", arm(),            flush=True)

# 2. Open-loop takeoff + climb: stream wings-level + 90% throttle + airspeed-scheduled pitch.
v_peak = 0.0; theta = 0.0
flare0 = None                                       # pitch captured when the level-off begins
t = time.time(); t_prev = t; last = 0
while time.time() - t < 120:
    pump()
    now = time.time()
    dt = min(now - t_prev, 0.15); t_prev = now
    v_peak = max(v_peak, st["as"])
    fpa = math.degrees(math.asin(_clamp(st["climb"] / max(st["as"], 0.1), -1, 1)))
    spd_trim = K_SPD * max(0.0, V_HOLD - st["as"])
    thr_cmd = THR_PWM
    if st["alt"] >= CRUISE_ALT - LEVEL_BAND:
        # Exponential level-off: blend s = e^(-K*|alt-CRUISE_ALT|) holds near-full climb authority
        # until within a few metres of CRUISE_ALT, then -> 0 pitch + cruise throttle at the top.
        if flare0 is None:
            flare0 = theta
        s = math.exp(-LEVEL_K * abs(st["alt"] - CRUISE_ALT))
        target  = flare0 * (1.0 - s)
        thr_cmd = THR_PWM + (CRUISE_CAP_THR - THR_PWM) * s
    elif v_peak < V_R:
        target = GND_PITCH                          # ground roll: slight nose-down hold
    else:
        target = min(theta_target(v_peak) - spd_trim, fpa + AOA_CAP)   # schedule + AoA guard
        target = max(0.0, target)
    theta  = rate_limit(theta, target, dt)
    send_sticks(theta, thr=int(thr_cmd))
    if now - last > 2:
        print(f"CLIMB t={now-t:4.0f}s mode={c.flightmode or '?':5s} cmd={theta:5.1f} pit={st['pitch']:5.1f} "
              f"alt={st['alt']:6.1f} as={st['as']:5.1f} Vpk={v_peak:4.1f} thr={st['thr']:3.0f}% "
              f"climb={st['climb']:5.2f} FPA~{fpa:5.1f}", flush=True)
        last = now
    if st["alt"] >= CRUISE_ALT - 1:
        print(f"leveled at {st['alt']:.1f} m (theta={theta:.1f}) -> cruise hold", flush=True)
        break
    time.sleep(0.05)

# 3. LEVEL CRUISE HOLD (open-loop FBWA). Speed-on-pitch keeps ~12 m/s at the level-trim throttle.
print(f"level cruise hold for {CRUISE_HOLD_T:.0f}s ...", flush=True)
tcap = time.time(); last = 0
while time.time() - tcap < CRUISE_HOLD_T:
    pump()
    theta_cap = _clamp(K_CAP * (st["as"] - CRUISE_AS), -5.0, TH_CLIMB)
    send_sticks(theta_cap, thr=CRUISE_CAP_THR)
    now = time.time()
    if now - last > 2:
        print(f"CRUISE t={now-tcap:4.0f}s cmd={theta_cap:5.1f} pit={st['pitch']:5.1f} as={st['as']:5.1f} "
              f"climb={st['climb']:5.2f} alt={st['alt']:6.1f} thr={st['thr']:3.0f}%", flush=True)
        last = now
    time.sleep(0.05)

# 4. LOITER (autopilot nav circle, radius WP_LOITER_RAD). Release the open-loop overrides so the
#    autopilot takes the sticks, circle at the cruise altitude for LOITER_T s, then hand back to
#    FBWA. With differential thrust ON this is where it does work: the autopilot's rudder demand
#    becomes an antisymmetric motor split -> watch motor1 (port) vs motor9 (starboard) diverge.
print(f"LOITER radius={LOITER_RAD:.0f} m for {LOITER_T:.0f}s (autopilot nav) ...", flush=True)
release_sticks()
print("LOITER:", set_mode("LOITER"), flush=True)
tlo = time.time(); last = 0
while time.time() - tlo < LOITER_T:
    pump()
    now = time.time()
    if now - last > 2:
        print(f"LOITER t={now-tlo:4.0f}s mode={c.flightmode or '?':6s} as={st['as']:5.1f} "
              f"alt={st['alt']:6.1f} thr={st['thr']:3.0f}% yawrate={st['yawrate']:5.1f} "
              f"m1={st['m1']} m9={st['m9']} split={st['m1']-st['m9']:+d}", flush=True)
        last = now
    time.sleep(0.1)
print("FBWA:", set_mode("FBWA"), flush=True)        # back to open-loop FBWA for the approach

# 5. APPROACH / GLIDE (FBWA, DECOUPLED). PITCH holds the approach speed (speed-on-pitch),
#    THROTTLE holds the sink rate (sink-on-throttle around THR_APP_TRIM). Down to FLARE_ALT.
print(f"approach: hold {V_APP:.0f} m/s, sink {SINK_APP:.1f} m/s, down to {FLARE_ALT:.0f} m ...", flush=True)
theta = 0.0; thr_app = float(THR_APP_TRIM)
t = time.time(); t_prev = t; last = 0
while time.time() - t < 120:
    pump()
    now = time.time()
    dt = min(now - t_prev, 0.15); t_prev = now
    target = K_APP_PITCH * (st["as"] - V_APP)                 # speed-on-pitch (stall guard below)
    hi = APP_PITCH_MAX if st["as"] > V_MIN_APP else 0.0
    target = _clamp(target, APP_PITCH_MIN, hi)
    theta  = rate_limit(theta, target, dt)
    thr_app = _clamp(THR_APP_TRIM + K_THR_SINK * (SINK_APP - st["climb"]), THR_APP_MIN, THR_APP_MAX)
    send_sticks(theta, thr=int(thr_app))
    if now - last > 2:
        print(f"APP  t={now-t:4.0f}s cmd={theta:5.1f} pit={st['pitch']:5.1f} aoa={st['aoa']:4.1f} "
              f"as={st['as']:5.1f} sink={st['climb']:5.2f}/{SINK_APP:.1f} alt={st['alt']:6.1f} "
              f"thr={st['thr']:3.0f}%(cmd{int(thr_app)})", flush=True)
        last = now
    if st["alt"] <= FLARE_ALT:
        print(f"reached flare alt {st['alt']:.1f} m (pit={st['pitch']:.1f}, thr={int(thr_app)}) -> POWERED flare", flush=True)
        break
    time.sleep(0.05)

# 6. HYBRID FLARE (Option B): PITCH follows the MATLAB height-scheduled attitude demand theta(h);
#    THROTTLE still arrests the sink around the height-scheduled target (v9 powered law, unchanged).
#    Throttle is cut to idle ONLY after touchdown (step 7).
t = time.time(); t_prev = t; last = 0
while time.time() - t < 30:
    pump()
    now = time.time()
    dt = min(now - t_prev, 0.15); t_prev = now
    h = max(0.0, st["alt"])
    sink_cmd  = SINK_TD + (SINK_APP - SINK_TD) * _clamp(h / FLARE_ALT, 0.0, 1.0)
    theta_cmd = flare_theta(h) if USE_FLARE_SCHED else TH_FLARE_HOLD   # MATLAB theta(h) demand
    theta     = rate_limit(theta, theta_cmd, dt, cap=FLARE_RATE_LIM)   # follow the scheduled nose-up
    thr_flare = _clamp(THR_APP_TRIM + K_THR_FLARE * (sink_cmd - st["climb"]), THR_FLARE_MIN, THR_FLARE_MAX)
    send_sticks(theta, thr=int(thr_flare))
    if now - last > 1:
        print(f"FLARE t={now-t:4.0f}s cmd={theta:5.1f} tgt={theta_cmd:5.1f} pit={st['pitch']:5.1f} aoa={st['aoa']:4.1f} "
              f"sink={st['climb']:5.2f}/cmd{sink_cmd:5.2f} as={st['as']:5.1f} alt={st['alt']:5.2f} "
              f"thr={st['thr']:3.0f}%(cmd{int(thr_flare)}){' ELEV_SAT' if elev_sat() else ''}", flush=True)
        last = now
    if st["alt"] <= TD_ALT:
        print(f"touchdown at alt={st['alt']:.2f} m, as={st['as']:.1f} m/s, sink={st['climb']:.2f} m/s, "
              f"pitch={st['pitch']:.1f} aoa={st['aoa']:.1f}", flush=True)
        break
    time.sleep(0.05)

# 7. ROLLOUT + DISARM. NOW cut throttle to idle + slight nose-down while the speed bleeds, then
#    release the overrides and disarm.
print(f"rollout {ROLLOUT_T:.0f}s at idle, then disarm ...", flush=True)
t = time.time()
while time.time() - t < ROLLOUT_T:
    pump()
    send_sticks(GND_PITCH, thr=IDLE_THR_PWM)
    time.sleep(0.05)
release_sticks()
print("disarmed:", disarm(), flush=True)

print("DONE (FBWA takeoff+climb -> %.0fs cruise hold -> %.0fs LOITER R=%.0fm%s -> approach %.0fm/s -> HYBRID flare theta(h) -> touchdown)"
      % (CRUISE_HOLD_T, LOITER_T, LOITER_RAD, " +DT" if DIFF_THRUST else "", V_APP), flush=True)

# 8. Plotting handled by the atexit hook (_plot_on_exit) registered at the top.
