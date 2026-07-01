#!/usr/bin/env python3

"""
uSTOL stability driver (connects to an already-running SITL) - CRUISE ROLL/YAW DOUBLETS, NO DIFF THRUST.

The NO-DIFFERENTIAL-THRUST baseline twin of mission_ustol_10_cruise.py: byte-for-byte the same
roll/yaw doublet stability test, but with DIFF_THRUST=False so the AP_DiffThrust mixer is DISABLED
(UST_ENABLE=0) and the nine ESCs are NOT remapped to k_motor1..9. Yaw is therefore handled the
stock way (rudder surface / stock mixing), not by an antisymmetric motor split. Run this against
mission_ustol_10_cruise.py to see what the differential-thrust mixer actually buys you in turns.

Single-run, watch-it-on-a-map sibling of mission_ustol_9.py. Its ONLY purpose is to check that the
aircraft is dynamically stable when perturbed in roll and in yaw at cruise. It drops the loiter and
the landing: it flies a sequence of bank-angle and rudder doublets and ends as soon as the aircraft
has settled back to steady, wings-level cruise.

Sequence:
  0) claim GCS sysid + headless-arm params + roll/rudder limits; with DIFF_THRUST=False this also
     forces UST_ENABLE=0 (stock uniform throttle, no motor split).
  1) arm in FBWA
  2) open-loop takeoff + climb to CRUISE_ALT (airspeed-scheduled, rate-limited pitch with airspeed
     protection + AoA guard, exponential level-off) - identical to mission_ustol_9. The stability
     test proper STARTS once we are level at cruise.
  3) CRUISE SETTLE: a few seconds of wings-level open-loop hold so we begin each test from trim.
  4) ROLL DOUBLETS: command a series of bank-angle setpoints
     +10 -> 0 -> -10 -> 0 -> +20 -> 0 -> -20 -> 0 (deg), holding each for HOLD_T s. Pitch and
     throttle keep holding cruise the whole time, so ONLY roll is perturbed. We log commanded vs
     achieved bank so you can see it track the step and damp back without sustained oscillation.
  5) YAW DOUBLETS: same +10 -> 0 -> -10 -> 0 -> +20 -> 0 -> -20 -> 0 pattern, but on RUDDER
     deflection (deg of the +-40 rudder). In FBWA the rudder is manual (the AP holds no yaw angle),
     so this is an open-loop yaw kick: watch the yaw rate spike and then damp and the heading return
     toward trim. With diff thrust OFF the motor1/motor9 split stays ~0 (the m1/m9 columns are inert).
  6) RETURN TO CRUISE + STOP: hold wings-level/neutral-rudder cruise until the aircraft is settled
     (|bank| and |yaw rate| small for SETTLE_OK_T s), then release the overrides and END the run.
     The aircraft is NOT disarmed (it is airborne) - the AP is left in FBWA; stop the SITL yourself.

NOTE: this baseline yaws via the stock rudder path. If the airframe has no real rudder surface (yaw
came solely from differential thrust), expect the yaw doublets to produce little/no yaw response -
that contrast vs mission_ustol_10_cruise.py is exactly the point of running this twin.

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
CRUISE_ALT    = 100.0                          # level off / start the stability test at this alt (m AGL)
CRUISE_AS     = 12.0                           # cruise airspeed after level-off
CRUISE_CAP_THR = 1660                          # FBWA throttle (pwm) for ~12 m/s LEVEL during cruise hold
K_CAP          = 3.0                           # cruise pitch-on-speed gain: theta = K_CAP*(V-CRUISE_AS)
LEVEL_BAND     = 10.0                          # start the level-off this far below CRUISE_ALT (m)
LEVEL_K        = 0.5                           # exp level-off rate (1/m): blend = e^(-K*|alt-CRUISE_ALT|)

# Stability test (the point of this file): roll then yaw doublets at cruise. Each maneuver starts
# from a SETTLE-GATED wings-level state (no cross-contamination between steps), and we measure the
# RECOVERY time back to wings-level after each step.
HOLD_T        = 6.0                            # hold each setpoint this long (s)
ROLL_SEQ      = [10.0, -10.0, 20.0, -20.0]     # commanded bank-angle targets (deg); return-to-0 is gated
YAW_SEQ       = [10.0, -10.0, 20.0, -20.0]     # commanded rudder targets (deg of rudder)
SETTLE_OK_T   = 3.0                            # must stay within the bands below this long to count as "settled"
ROLL_OK_DEG   = 3.0                            # |bank| under this = level
YAWRATE_OK    = 5.0                            # |yaw rate| under this (deg/s) = yaw damped
SETTLE_MAX_T  = 15.0                           # cap on the pre-maneuver settle gate (s); proceed even if not settled
RECOVER_MAX_T = 15.0                           # cap on the post-maneuver recovery wait (s); this measures rollout time
RETURN_MAX_T  = 20.0                           # final settle cap before ending the run (s)

# Altitude/energy hold during the test so every doublet starts near CRUISE_ALT instead of drifting
# down. Throttle trims energy back toward altitude, but is FROZEN above BANK_THR_FREEZE so we never
# add power in a turn (adding power in a bank tightens a spiral). Pitch is plain speed-on-pitch - we
# deliberately do NOT add a load-factor nose-up term, because pulling up in a bank tightens a spiral.
K_ALT_THR     = 6.0                            # cruise throttle alt-hold gain (pwm per m of altitude error)
THR_CR_MIN, THR_CR_MAX = 1450, 1850            # cruise throttle clamp for the alt-hold
BANK_THR_FREEZE = 25.0                         # above this |bank| (deg), freeze throttle at base (no alt push)

# Departure guard: if the aircraft spirals past BANK_ABORT during a maneuver, stop commanding the
# doublet and run a wings-level RECOVERY (nose down + throttle back), then abort the rest of the run.
BANK_ABORT    = 45.0                           # |bank| (deg) over this during a step = departed -> recover + abort
RECOVER_PITCH = -3.0                           # nose-down pitch (deg) held during spiral recovery (unload)
RECOVER_THR   = 1500                           # reduced throttle (pwm) during spiral recovery

# Roll-controller tuning (FBWA). NB: retunes the AUTOPILOT itself (all FBWA/auto flight), not just
# this test. FINDING from the runs: SOFTENING the roll loop (TCONST up) made the over-bank WORSE and
# triggered a spiral departure - the over-bank is spiral divergence from missing yaw coordination, so
# a slower roll loop holds bank LESS well. If tuning at all, go TIGHTER (TCONST down, P/D up) to fight
# the divergence - but roll gains can't fix it; differential thrust is the real fix. Leave OFF.
TUNE_ROLL     = False                          # OFF: roll-gain tuning can't fix the spiral over-bank here
RLL_TCONST    = 0.15                           # was 0.2: SMALLER = tighter/faster roll loop (fight the spiral)
RLL_RATE_P_T  = 0.12                           # was 0.08: more roll-rate proportional authority
RLL_RATE_D_T  = 0.02                           # was 0.008: a little roll-rate damping

# Roll/rudder command mapping (FBWA: RC1 commands a BANK ANGLE, RC4 is a MANUAL rudder deflection).
ROLL_LIMIT_DEG = 30.0                          # set on the AP; RC1 full stick = this bank. Must exceed 20.
RUD_LIMIT_DEG  = 40.0                          # rudder travel (+-40 on this airframe); RC4 full = this
# These MUST match the AP's RCx_MIN/TRIM/MAX (params_imp_v3.param). RC1 is ASYMMETRIC
# (right half = 1775-1500 = 275us, left half = 1500-1000 = 500us), so the half-range used to map a
# +-frac bank differs by sign - a naive symmetric 1500+-500 map over-banks every RIGHT command ~2x
# (a +10 deg request decodes to ~18 deg of stick). We honor the real calibration so bank is precise.
RC1_MIN, RC1_TRIM, RC1_MAX = 1000, 1500, 1775  # aileron INPUT channel calibration (asymmetric!)
RC4_MIN, RC4_TRIM, RC4_MAX = 1000, 1500, 2000  # rudder  INPUT channel calibration (symmetric)
ROLL_SIGN     = +1                             # +cmd must bank RIGHT; flip to -1 if it banks left
RUD_SIGN      = +1                             # +cmd must yaw nose-RIGHT; flip to -1 if it yaws left

# Differential thrust (AP_DiffThrust / UST_ mixer). DIFF_THRUST=False -> stock uniform throttle.
DIFF_THRUST   = False                          # NO-DT baseline: disable the mixer (UST_ENABLE=0), stock yaw
UST_DT_VLO    = 13.0                           # m/s, full differential authority at/below
UST_DT_VHI    = 17.0                           # m/s, zero authority at/above (must exceed VLO)
UST_NDES_MAX  = 20.0                           # peak yaw MOMENT [N*m] at full rudder (was UST_DT_KYAW gain)
UST_KRUD      = 0.11                            # >0 = rudder-aware daisy-chain (new); 0 = legacy VLO/VHI schedule
UST_DT_RLFF   = 0.0                            # aileron roll feedforward (0 = off)
UST_UMAX      = 1.0                            # per-motor command ceiling; 1.0 = uncapped (SITL takeoff)
K_MOTOR_FN    = [33, 34, 35, 36, 37, 38, 39, 40, 82]   # SERVO1..9 -> k_motor1..k_motor9 functions

# Takeoff + climb to cruise (identical to mission_ustol_9; the stability test starts after this).
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

st = {"alt": 0.0, "as": 0.0, "climb": 0.0, "lat": None, "lon": None, "thr": 0.0, "thr_pwm": 0,
      "roll": 0.0, "pitch": 0.0, "yawrate": 0.0, "hdg": 0.0, "elev": 1500, "aoa": 0.0,
      "m1": 0, "m9": 0}
_PUMP_TYPES = ["GLOBAL_POSITION_INT", "VFR_HUD", "RC_CHANNELS", "ATTITUDE", "SERVO_OUTPUT_RAW", "AOA_SSA"]
def pump():
    """Drain ALL buffered telemetry each call and keep the LATEST values.

    CRITICAL: read the whole buffer, not one message. The AP streams faster than this loop
    consumes, so reading a single message per call lets the socket buffer back up and every
    recv_match returns an OLDER message -> st values lag tens of seconds behind real time and
    the settle/return triggers never fire.
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
            st["hdg"] = m.hdg / 100.0                    # heading (deg), for the yaw test
        elif t == "VFR_HUD":
            st["as"]    = m.airspeed
            st["climb"] = m.climb
            st["thr"]   = m.throttle
        elif t == "RC_CHANNELS":
            st["thr_pwm"] = m.chan3_raw
        elif t == "ATTITUDE":
            st["roll"]    = math.degrees(m.roll)         # MEASURED bank (deg)  - the roll-test signal
            st["pitch"]   = math.degrees(m.pitch)        # MEASURED pitch (deg)
            st["yawrate"] = math.degrees(m.yawspeed)     # body yaw rate (deg/s) - the yaw-test signal
        elif t == "SERVO_OUTPUT_RAW":
            # SERVO1 (k_motor1, port outer) & SERVO9 (k_motor9, starboard outer): their split is
            # the direct signature of differential thrust during a yaw command. The elevator is
            # SERVO11 (k_elevator, fn 19); SERVO14 is the FLAP (fn 2) and sits dead at 1100 — reading
            # it as the elevator pinned ELEV_SAT True on a constant-1100 channel (false positive).
            st["m1"]   = getattr(m, "servo1_raw", st["m1"])
            st["m9"]   = getattr(m, "servo9_raw", st["m9"])
            st["elev"] = getattr(m, "servo11_raw", st["elev"])
        elif t == "AOA_SSA":
            st["aoa"] = getattr(m, "AOA", st["aoa"])     # true angle of attack (deg), if streamed
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

def roll_pwm(bank_deg):
    """Map a demanded BANK angle (deg) to the FBWA aileron-input PWM (RC1), honoring the AP's
    ASYMMETRIC RC1 calibration: a positive (right) command spans TRIM..MAX (275us), a negative
    (left) command spans TRIM..MIN (500us). Using the matching half-range per sign makes the
    decoded bank precise instead of over-banking to the right."""
    frac = ROLL_SIGN * bank_deg / ROLL_LIMIT_DEG
    frac = _clamp(frac, -1.0, 1.0)
    span = (RC1_MAX - RC1_TRIM) if frac >= 0 else (RC1_TRIM - RC1_MIN)
    return int(RC1_TRIM + frac * span)

def rud_pwm(rud_deg):
    """Map a demanded RUDDER deflection (deg) to the FBWA rudder-input PWM (RC4, manual)."""
    frac = RUD_SIGN * rud_deg / RUD_LIMIT_DEG
    frac = _clamp(frac, -1.0, 1.0)
    return int(RC4_TRIM + frac * (RC4_MAX - RC4_TRIM))

def send_sticks(theta_deg, roll_deg=0.0, rud_deg=0.0, thr=None):
    """Stream pitch + (optional) bank + (optional) rudder + throttle. CALL EVERY LOOP (the override
    must stream continuously or the RC-loss failsafe trips). With diff thrust on, the throttle
    channel reaches the nine k_motor ESCs through the mixer's uniform base and the rudder command
    becomes the antisymmetric motor split."""
    c.mav.rc_channels_override_send(
        c.target_system, c.target_component,
        roll_pwm(roll_deg),
        pitch_pwm(theta_deg),
        THR_PWM if thr is None else thr,
        rud_pwm(rud_deg),
        0, 0, 0, 0)

def release_sticks():
    """Send 0 on all channels = release override so the AP's own FBWA stabilizer takes the sticks."""
    c.mav.rc_channels_override_send(c.target_system, c.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

def cruise_pitch():
    """Open-loop cruise pitch (deg): plain speed-on-pitch about CRUISE_AS. NB: deliberately NO
    load-factor nose-up term - on this airframe (especially no-DT) pulling up in a bank tightens the
    spiral; we keep pitch neutral and hold altitude with throttle instead."""
    return _clamp(K_CAP * (st["as"] - CRUISE_AS), -5.0, TH_CLIMB)

def cruise_thr():
    """Cruise throttle (pwm) with an altitude-hold trim: add power below CRUISE_ALT, ease off above.
    FROZEN at base above BANK_THR_FREEZE bank, so we never push power into a turn/spiral."""
    if abs(st["roll"]) > BANK_THR_FREEZE:
        return CRUISE_CAP_THR
    return int(_clamp(CRUISE_CAP_THR + K_ALT_THR * (CRUISE_ALT - st["alt"]), THR_CR_MIN, THR_CR_MAX))

def settle(tag, max_t=SETTLE_MAX_T):
    """Hold wings-level/neutral-rudder cruise (with alt-hold) until in-band (|roll|<ROLL_OK_DEG and
    |yawrate|<YAWRATE_OK) continuously for SETTLE_OK_T s, or until max_t. Returns seconds-to-settle,
    or -1 on timeout. Used to start every maneuver from the SAME clean trim (no cross-contamination)."""
    t0 = time.time(); last = 0; ok_since = None
    while time.time() - t0 < max_t:
        pump()
        send_sticks(cruise_pitch(), thr=cruise_thr())
        now = time.time()
        in_band = abs(st["roll"]) < ROLL_OK_DEG and abs(st["yawrate"]) < YAWRATE_OK
        ok_since = ok_since if (in_band and ok_since is not None) else (now if in_band else None)
        if now - last > 2:
            print(f"{tag} settle t={now-t0:4.1f}s roll={st['roll']:5.1f} yawrate={st['yawrate']:5.1f} "
                  f"as={st['as']:5.1f} alt={st['alt']:6.1f}", flush=True)
            last = now
        if ok_since is not None and now - ok_since >= SETTLE_OK_T:
            return now - t0
        time.sleep(0.05)
    return -1.0

def departed():
    """True when the aircraft has banked past BANK_ABORT - i.e. it has spiralled out of the test
    envelope and a doublet command is no longer meaningful."""
    return abs(st["roll"]) > BANK_ABORT

def recover_wings_level(max_t=RECOVER_MAX_T):
    """Break a spiral/departure: command wings level + a fixed nose-DOWN + reduced throttle. We do
    NOT use speed-on-pitch here - in a spiral the airspeed climbs, and speed-on-pitch would pull the
    nose UP and tighten the turn. Returns seconds-to-recover, or -1 on timeout."""
    print(f"!! DEPARTURE recovery: wings level, nose {RECOVER_PITCH:.0f} deg, throttle {RECOVER_THR} "
          f"(from roll={st['roll']:.0f}, yawrate={st['yawrate']:.0f})", flush=True)
    t0 = time.time(); last = 0; ok_since = None
    while time.time() - t0 < max_t:
        pump()
        send_sticks(RECOVER_PITCH, roll_deg=0.0, rud_deg=0.0, thr=RECOVER_THR)
        now = time.time()
        in_band = abs(st["roll"]) < ROLL_OK_DEG and abs(st["yawrate"]) < YAWRATE_OK
        ok_since = ok_since if (in_band and ok_since is not None) else (now if in_band else None)
        if now - last > 1:
            print(f"RECOVER t={now-t0:4.1f}s roll={st['roll']:6.1f} yawrate={st['yawrate']:6.1f} "
                  f"as={st['as']:5.1f} alt={st['alt']:6.1f}", flush=True)
            last = now
        if ok_since is not None and now - ok_since >= SETTLE_OK_T:
            return now - t0
        time.sleep(0.05)
    return -1.0

# D. MISSION PROFILE  (FBWA climb to cruise, then ROLL doublets, then YAW doublets, then stop)

# 0. Params: headless arm, pitch ceiling above the climb attitude, low-speed floor, roll/rudder
#    limits for the doublets, and (when DIFF_THRUST) the k_motor servo map + UST_ mixer tune.
gcs_ok = set_param("SYSID_MYGCS", 250) or set_param("MAV_GCS_SYSID", 250)
print(f"claim GCS sysid=250 (so RC overrides are accepted): {gcs_ok}", flush=True)
for n, v in [
    ("ARMING_CHECK",     0),                # allow headless arming
    ("AIRSPEED_MIN",     7),                # liftoff sits above the floor (not flagged)
    ("PTCH_LIM_MAX_DEG", 20),               # >= TH_CLIMB so FBWA won't clamp the climb pitch
    ("PTCH_LIM_MIN_DEG", -20),              # allow nose-down for the level-off
    ("TECS_PITCH_MAX",   20),
    ("AIRSPEED_CRUISE",  CRUISE_AS),        # TECS cruise speed
    ("ROLL_LIMIT_DEG",   ROLL_LIMIT_DEG),   # FBWA bank-angle authority; full RC1 stick = this bank
]:
    print(f"set {n}={v}: {set_param(n, v)}", flush=True)

if TUNE_ROLL:
    # Retune the FBWA roll controller to cut bank overshoot (TCONST/D) and the steady over-bank in a
    # turn (rate-I). Affects the autopilot itself, not just this test. Iterate from the SUMMARY.
    for n, v in [
        ("RLL2SRV_TCONST", RLL_TCONST),
        ("RLL_RATE_P",     RLL_RATE_P_T),
        ("RLL_RATE_D",     RLL_RATE_D_T),
    ]:
        print(f"set {n}={v}: {set_param(n, v)}", flush=True)
    print("-> ROLL TUNING ON (TIGHTER loop: TCONST down, P/D up; NB DT is the real fix for over-bank)", flush=True)

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

# 1. Arm in FBWA. No mission: throttle + pitch + roll + rudder come from the RC override stream.
print("FBWA:",  set_mode("FBWA"), flush=True)
print("EKF ready:", wait_ekf_ready(), flush=True)   # let GPS-aided alignment finish on the GROUND
print("armed:", arm(),            flush=True)

# 2. Open-loop takeoff + climb to CRUISE_ALT (wings level, neutral rudder). Same as mission_ustol_9.
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
    send_sticks(theta, thr=int(thr_cmd))            # wings level, neutral rudder during the climb
    if now - last > 2:
        print(f"CLIMB t={now-t:4.0f}s mode={c.flightmode or '?':5s} cmd={theta:5.1f} pit={st['pitch']:5.1f} "
              f"alt={st['alt']:6.1f} as={st['as']:5.1f} Vpk={v_peak:4.1f} thr={st['thr']:3.0f}% "
              f"climb={st['climb']:5.2f} FPA~{fpa:5.1f}", flush=True)
        last = now
    if st["alt"] >= CRUISE_ALT - 1:
        print(f"leveled at {st['alt']:.1f} m (theta={theta:.1f}) -> begin stability test", flush=True)
        break
    time.sleep(0.05)

# 3. CRUISE SETTLE: gated wings-level hold (with alt-hold) so the first doublet starts from trim.
print("cruise settle (gated: wings level + alt-hold) ...", flush=True)
print(f"  settled in {settle('CRUISE'):.1f}s", flush=True)

# 4+5. DOUBLET RUNNER (gated). For each target: (a) SETTLE-gate to a clean wings-level start so steps
#      don't contaminate each other, (b) HOLD the command for HOLD_T while logging the response + peak
#      excursion, (c) command 0 and measure the RECOVERY time back to wings-level. axis="roll" commands
#      a bank angle (RC1); axis="yaw" a manual rudder deflection (RC4). Pitch/throttle hold cruise (with
#      alt-hold) throughout, so only the axis under test is perturbed.
results = []   # per-step metrics, consumed by print_summary() at the end of the run

def _abort(axis, sp):
    """Record an aborted step (captures the departure bank BEFORE recovery), recover, return True."""
    results.append({"axis": axis, "cmd": sp, "aborted": True, "depart_roll": st["roll"]})
    recover_wings_level()
    return True

def run_doublets(axis, targets):
    """Run the doublet sequence on one axis. Returns True if the run was ABORTED on a departure."""
    label = "ROLL" if axis == "roll" else "YAW "
    for i, sp in enumerate(targets):
        if settle(f"{label}pre{i}") < 0 and departed():          # (a) couldn't even get a clean start
            print(f"{label} step{i} cmd={sp:+.1f}: DEPARTED before start (roll={st['roll']:.0f}) -> abort", flush=True)
            return _abort(axis, sp)
        # (b) apply the step and HOLD
        t0 = time.time(); last = 0.0
        pk_roll = 0.0; pk_yawrate = 0.0; hdg0 = st["hdg"]
        while time.time() - t0 < HOLD_T:
            pump()
            if axis == "roll":
                send_sticks(cruise_pitch(), roll_deg=sp, thr=cruise_thr())
            else:
                send_sticks(cruise_pitch(), rud_deg=sp, thr=cruise_thr())
            pk_roll    = st["roll"]    if abs(st["roll"])    > abs(pk_roll)    else pk_roll
            pk_yawrate = st["yawrate"] if abs(st["yawrate"]) > abs(pk_yawrate) else pk_yawrate
            now = time.time()
            if departed():                                       # spiral during the hold -> recover + abort
                print(f"{label} step{i} cmd={sp:+.1f}: DEPARTED at roll={st['roll']:.0f} deg -> abort", flush=True)
                return _abort(axis, sp)
            if now - last > 1:
                # heading change wrapped to +-180 so the yaw response reads cleanly across 0/360
                dhdg = (st["hdg"] - hdg0 + 180.0) % 360.0 - 180.0
                print(f"{label} step{i} cmd={sp:+6.1f} t={now-t0:4.1f}s roll={st['roll']:6.1f} "
                      f"yawrate={st['yawrate']:6.1f} dhdg={dhdg:6.1f} as={st['as']:5.1f} "
                      f"alt={st['alt']:6.1f} m1={st['m1']} m9={st['m9']} split={st['m1']-st['m9']:+d}",
                      flush=True)
                last = now
            time.sleep(0.05)
        held_roll = st["roll"]; held_yawrate = st["yawrate"]      # values at the end of the hold
        # (c) RECOVERY: command 0 and time the return to wings-level
        tr = time.time(); ok_since = None; t_rec = -1.0
        while time.time() - tr < RECOVER_MAX_T:
            pump()
            send_sticks(cruise_pitch(), thr=cruise_thr())
            now = time.time()
            if departed():                                       # rollout turned into a spiral -> recover + abort
                print(f"{label} step{i} cmd={sp:+.1f}: spiral during rollout (roll={st['roll']:.0f} deg) -> abort", flush=True)
                return _abort(axis, sp)
            in_band = abs(st["roll"]) < ROLL_OK_DEG and abs(st["yawrate"]) < YAWRATE_OK
            ok_since = ok_since if (in_band and ok_since is not None) else (now if in_band else None)
            if ok_since is not None and now - ok_since >= SETTLE_OK_T:
                t_rec = ok_since - tr; break
            time.sleep(0.05)
        rec_str = f"{t_rec:4.1f}s" if t_rec >= 0 else f">{RECOVER_MAX_T:.0f}s(!)"
        results.append({"axis": axis, "cmd": sp, "aborted": False, "held_roll": held_roll,
                        "held_yawrate": held_yawrate, "pk_roll": pk_roll, "pk_yawrate": pk_yawrate,
                        "t_rec": t_rec, "alt": st["alt"]})
        print(f"{label} step{i} done: cmd={sp:+6.1f}  held roll={held_roll:+6.1f} yawrate={held_yawrate:+6.1f}  "
              f"peak roll={pk_roll:+6.1f} peak yawrate={pk_yawrate:+6.1f}  recover={rec_str}", flush=True)
    return False

def print_summary(settled, t_settle, aborted):
    """One-glance end-of-run scorecard for DT-vs-no-DT and gain-tuning comparison."""
    done  = [r for r in results if not r["aborted"]]
    abr   = [r for r in results if r["aborted"]]
    rolls = [r for r in done if r["axis"] == "roll"]
    yaws  = [r for r in done if r["axis"] == "yaw"]
    print("=" * 66, flush=True)
    print(f"SUMMARY  mode={'DT' if DIFF_THRUST else 'no-DT'}  roll-tune={'on' if TUNE_ROLL else 'off'}  "
          f"steps_ok={len(done)}  aborted={aborted}", flush=True)
    if rolls:
        be = max(abs(abs(r['held_roll']) - abs(r['cmd'])) for r in rolls)     # worst steady |held|-|cmd|
        ov = max(abs(r['pk_roll']) - abs(r['held_roll'])  for r in rolls)     # worst dynamic overshoot
        rc = max((r['t_rec'] for r in rolls if r['t_rec'] >= 0), default=-1.0)
        print(f"  ROLL: worst |held-cmd| = {be:4.1f} deg   worst overshoot = {ov:4.1f} deg   "
              f"worst recover = {rc:4.1f}s", flush=True)
    if yaws:
        yr = max(abs(r['pk_yawrate']) for r in yaws)
        rc = max((r['t_rec'] for r in yaws if r['t_rec'] >= 0), default=-1.0)
        print(f"  YAW : peak |yaw rate| = {yr:4.1f} deg/s   worst recover = {rc:4.1f}s", flush=True)
    if done:
        alts = [r['alt'] for r in done]
        print(f"  ALT : {min(alts):5.1f} .. {max(alts):5.1f} m  (target {CRUISE_ALT:.0f}; "
              f"max drift {max(abs(a - CRUISE_ALT) for a in alts):.1f} m)", flush=True)
    for a in abr:
        print(f"  ABORT on {a['axis']} cmd={a['cmd']:+.0f}: departed at roll={a['depart_roll']:.0f} deg", flush=True)
    print(f"  final settle: {('%.1fs' % t_settle) if settled else 'TIMEOUT'}", flush=True)
    print("=" * 66, flush=True)

print("=== ROLL DOUBLETS:", " -> ".join(f"{x:+.0f}" for x in ROLL_SEQ), "deg ===", flush=True)
aborted = run_doublets("roll", ROLL_SEQ)
if aborted:
    print("!! ROLL sequence ABORTED on a departure - skipping YAW doublets, going to final settle.", flush=True)
else:
    print("=== YAW DOUBLETS:", " -> ".join(f"{x:+.0f}" for x in YAW_SEQ), "deg rudder ===", flush=True)
    aborted = run_doublets("yaw", YAW_SEQ)
    if aborted:
        print("!! YAW sequence ABORTED on a departure - going to final settle.", flush=True)

# 6. RETURN TO CRUISE + STOP. Final gated settle to wings-level cruise, then END.
print("return to cruise (final settle) ...", flush=True)
t_settle = settle("RETURN", max_t=RETURN_MAX_T)
settled = t_settle >= 0
if settled:
    print(f"settled back to cruise in {t_settle:.1f}s (roll={st['roll']:.1f}, yawrate={st['yawrate']:.1f}) "
          f"-> stopping run", flush=True)
else:
    print(f"WARNING: did not settle within {RETURN_MAX_T:.0f}s "
          f"(roll={st['roll']:.1f}, yawrate={st['yawrate']:.1f}) - stopping run anyway", flush=True)

# End-of-run scorecard (DT vs no-DT, and gain-tuning iteration).
print_summary(settled, t_settle, aborted)

# Release the overrides (hand the sticks back to the AP's FBWA stabilizer) and END. The aircraft is
# airborne, so it is NOT disarmed; the AP is left flying wings-level FBWA - stop the SITL yourself.
release_sticks()
print("DONE (climb to cruise -> %d roll doublets -> %d yaw doublets -> %s%s). Overrides released; "
      "aircraft left in FBWA - stop the SITL when ready."
      % (len(ROLL_SEQ), len(YAW_SEQ), "settled" if settled else "NOT settled",
         " +DT" if DIFF_THRUST else ""), flush=True)

# 7. Plotting handled by the atexit hook (_plot_on_exit) registered at the top.
