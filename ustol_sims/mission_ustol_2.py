#!/usr/bin/env python3

"""
uSTOL mission driver (connects to an already-running SITL).

Sequence - FBWA open-loop takeoff + climb, then CRUISE:
  1) set pitch ceiling (>= climb attitude) + low-speed floor + headless arming
  2) arm in FBWA
  3) stream RC overrides: wings level, ~90% throttle, and a PITCH ANGLE
     scheduled on AIRSPEED and rate-limited to <= RATE_LIM_DPS deg/s:
        V < 6   -> 0 deg       (ground roll)
        6 -> 8.5 -> 0->7 deg   (rotate; liftoff ~8.5 m/s @ 7 deg)
        8.5 -> 10 -> 7->14 deg (keep rotating to climb attitude)
        V >= 10 -> 14 deg      (hold; FPA ~10 deg, open loop)
     ...with AIRSPEED PROTECTION: below V_HOLD the commanded pitch is bled down
     (K_SPD deg per m/s of speed deficit) so the open-loop climb can't stall the
     wing, and an AoA guard caps pitch at measured_FPA + AOA_CAP.
  4) at CRUISE_ALT (100 m): release overrides, switch to CRUISE, accelerate to 12 m/s

WHY AIRSPEED (not time): the triggers are speeds (V_R, liftoff, V_climb) and the
accel changes per Monte-Carlo run, so theta(t) would desync. theta(V) self-syncs;
we latch on peak airspeed so a dip can't drop the nose. The pitch-rate cap
is enforced by a real-time slew limiter (guaranteed regardless of acceleration).

theta_climb (=TH_CLIMB=19) is FPA_target + AoA (~15 + ~4); it's an open-loop knob -
fly it, read FPA = asin(climb/airspeed) from the log, nudge TH_CLIMB to hit 15 deg.

RC override is DROPPED by ArduPilot unless the sender sysid == the AP's GCS sysid,
so step 0 claims SYSID_MYGCS (or MAV_GCS_SYSID on newer builds) = 250.

Run SITL first (separate terminal), e.g.:
  build/sitl/bin/arduplane -w --model plane \
      --defaults ustol_sims/params_ustol.parm --home 28.559741,77.11745,237,285 -I0
and have one GCS (MAVProxy/Mission Planner) on 5760 so this script can use 5762.
(If a second GCS is up, it can steal SYSID_MYGCS - close it or give it another sysid.)

VERIFY ONCE (config-dependent): PITCH_SIGN (+5 cmd -> nose UP, else -1),
1900us ~= 90% throttle (RC3_MIN/MAX), 1500us neutral roll/yaw (RCx_TRIM),
RCMAP_ROLL/PITCH/THROTTLE/YAW = 1/2/3/4 (AETR override order below).

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
CRUISE_ALT    = 100.0                          # switch to CRUISE when alt reaches this (m AGL)
CRUISE_AS     = 12.0                           # cruise airspeed after level-off
CRUISE_THR_PWM = 1333                          # RC3 throttle-STICK for ~12 m/s in CRUISE:
                                               #   demand = AIRSPEED_MIN + frac*(AIRSPEED_MAX-MIN);
                                               #   (12-7)/(22-7)=0.33 of RC3 1000..2000 -> 1333us.
                                               #   In CRUISE the throttle stick = airspeed demand,
                                               #   so this must be streamed (not released) or the
                                               #   demand collapses to AIRSPEED_MIN (~stall).
CRUISE_CAP_THR = 1660                          # FBWA throttle (direct %) for ~12 m/s LEVEL during the
                                               #   speed-capture: ~67% of RC3 1000..2000, from the
                                               #   settled-TECS cruise throttle. Tune if cruise is fast/slow.
K_CAP          = 3.0                           # capture pitch-on-speed gain: theta = K_CAP*(V-CRUISE_AS)
LEVEL_BAND    = 10.0                           # flare band: smoothstep theta->0 over the last LEVEL_BAND m up to CRUISE_ALT, so we arrive level

V_R, V_LO, V_CLIMB = 6.0, 8.0, 10.0            # rotate / liftoff / climb-established speeds (m/s)
TH_LO, TH_CLIMB    = 7.0, 16.5                 # pitch at liftoff / in the climb (deg)
RATE_LIM_DPS       = 5.0                       # hard cap on commanded pitch rate q (deg/s)
FPA_TARGET         = 15.0                      # design flight-path angle (TH_CLIMB tuned to hit it)
AOA_CAP            = 7.0                       # max AoA: cmd pitch <= measured_FPA + AOA_CAP

V_HOLD             = 10.0                      # climb airspeed to protect (m/s); bleed pitch if slower
K_SPD              = 1.0                       # deg of pitch backed off per m/s below V_HOLD
GND_PITCH          = -2.0                      # nose-down hold during ground roll (V < V_R) so the airframe can't rotate before the scheduled rotation

RC2_TRIM, RC2_MAX  = 1500, 2000                # elevator INPUT channel endpoints
PTCH_LIM_MAX       = 20.0                      # must equal PTCH_LIM_MAX_DEG set below (>= TH_CLIMB)
PITCH_SIGN         = -1                        # +cmd must pitch nose UP; flip to -1 if it dives
THR_PWM            = 1900                      # 90% throttle on RC3

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

st = {"alt": 0.0, "as": 0.0, "climb": 0.0, "lat": None, "lon": None, "thr": 0.0, "thr_pwm": 0}
_PUMP_TYPES = ["GLOBAL_POSITION_INT", "VFR_HUD", "RC_CHANNELS"]
def pump():
    """Drain ALL buffered telemetry each call and keep the LATEST values.

    CRITICAL: read the whole buffer, not one message. The AP streams faster than this loop
    consumes, so reading a single message per call lets the socket buffer back up and every
    recv_match returns an OLDER message -> st["alt"]/as lag tens of seconds behind real time
    and the flare/altitude triggers never fire (plane is at 136 m while st["alt"] reads <85).
    First read blocks briefly so we always advance; then we drain whatever else is queued.
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
        m = c.recv_match(type=_PUMP_TYPES, blocking=False)  
    return got

def wait_ekf_ready(timeout=40):
    """Block until the EKF is healthy + GPS-aided so its attitude re-alignment happens on the
    GROUND, not mid-climb. (Arming with ARMING_CHECK=0 otherwise lets the ~20 s GPS-pickup
    alignment land in the climb -> the pitch-ESTIMATE spike to ~33 deg seen in ATT.Pitch,
    while true pitch (SIM) stays ~21.)"""
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

def theta_target(V):
    """V-scheduled TARGET pitch (deg). Rate-limited before it is commanded."""
    if V <= V_R:     return 0.0
    if V <= V_LO:    return TH_LO * _smooth((V - V_R) / (V_LO - V_R))
    if V <= V_CLIMB: return TH_LO + (TH_CLIMB - TH_LO) * _smooth((V - V_LO) / (V_CLIMB - V_LO))
    return TH_CLIMB

def rate_limit(prev, target, dt):
    """Slew-limit so |d(theta)/dt| <= RATE_LIM_DPS deg/s (the commanded pitch-rate cap)."""
    step = RATE_LIM_DPS * dt
    return max(prev - step, min(target, prev + step))

def pitch_pwm(theta_deg):
    """Map a demanded pitch angle (deg) to the FBWA elevator-input PWM."""
    frac = PITCH_SIGN * theta_deg / PTCH_LIM_MAX
    frac = -1.0 if frac < -1 else (1.0 if frac > 1 else frac)
    return int(RC2_TRIM + frac * (RC2_MAX - RC2_TRIM))

def send_sticks(theta_deg, thr=None):
    """Wings level + neutral rudder + fixed throttle + commanded pitch. CALL EVERY LOOP
    (override must stream continuously or the RC-loss failsafe trips)."""
    c.mav.rc_channels_override_send(
        c.target_system, c.target_component,
        1500,                                 
        pitch_pwm(theta_deg),                  
        THR_PWM if thr is None else thr,       
        1500,                                  
        0, 0, 0, 0)                          

def release_sticks():
    """Send 0 on all channels = release override so CRUISE/TECS takes the sticks."""
    c.mav.rc_channels_override_send(c.target_system, c.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

# D. MISSION PROFILE  (FBWA open-loop takeoff + climb, then CRUISE)

# 0. Params: headless arm, pitch ceiling above the climb attitude, low-speed floor.
# CRITICAL: ArduPilot DROPS RC_CHANNELS_OVERRIDE unless the sender sysid == the AP's
# GCS sysid. This script is source_system=250, so claim it or every override is ignored.
# (param renamed SYSID_MYGCS -> MAV_GCS_SYSID on newer builds; set whichever exists.)
gcs_ok = set_param("SYSID_MYGCS", 250) or set_param("MAV_GCS_SYSID", 250)
print(f"claim GCS sysid=250 (so RC overrides are accepted): {gcs_ok}", flush=True)
for n, v in [
    ("ARMING_CHECK",     0),          # allow headless arming
    ("AIRSPEED_MIN",     7),          # 8 m/s liftoff sits above the floor (not flagged)
    ("PTCH_LIM_MAX_DEG", 20),         # >= TH_CLIMB (14) so FBWA won't clamp the climb pitch
    ("TECS_PITCH_MAX",   20),         # matched ceiling for the CRUISE phase
    ("AIRSPEED_CRUISE",  CRUISE_AS),  # TECS cruise speed; set now so the handoff isn't blocked
]: 
    print(f"set {n}={v}: {set_param(n, v)}", flush=True)
print(f"-> profile: rotate@{V_R:.0f}  liftoff {TH_LO:.0f}deg@{V_LO:.1f}  "
      f"climb {TH_CLIMB:.0f}deg@{V_CLIMB:.0f} (FPA~{FPA_TARGET:.0f}deg, hold {V_HOLD:.0f}m/s)  "
      f"q<={RATE_LIM_DPS:.0f}deg/s", flush=True)

# 1. Arm in FBWA. No mission: throttle + pitch come from the RC override stream below.
print("FBWA:",  set_mode("FBWA"), flush=True)
print("EKF ready:", wait_ekf_ready(), flush=True)   # let GPS-aided alignment finish on the GROUND
print("armed:", arm(),            flush=True)

# 2. Open-loop takeoff + climb: stream wings-level + 90% throttle + airspeed-scheduled
v_peak = 0.0; theta = 0.0
flare_theta0 = None                                 # pitch captured when the flare begins
t = time.time(); t_prev = t; last = 0
while time.time() - t < 120:
    pump()
    now = time.time()
    dt = min(now - t_prev, 0.15); t_prev = now      # measured dt, capped vs telemetry stalls
    v_peak = max(v_peak, st["as"])                  # latch -> a speed dip can't drop the nose
    fpa = math.degrees(math.asin(max(-1, min(1, st["climb"] / max(st["as"], 0.1)))))
    # Airspeed protection (open loop): when slower than V_HOLD, bleed the commanded pitch
    # down (K_SPD deg per m/s of deficit) so the climb can't drive the wing into a stall.
    spd_trim = K_SPD * max(0.0, V_HOLD - st["as"])
    thr_cmd = THR_PWM                               # full throttle through climb + ground roll
    if st["alt"] >= CRUISE_ALT - LEVEL_BAND:
        # Flare: smoothstep the commanded pitch from its climb value down to 0 AND smoothly bring
        # the throttle down from full to the cruise-trim value, BOTH over the last LEVEL_BAND
        # metres -> arrive LEVEL and already decelerating toward 12 m/s, no throttle step.
        if flare_theta0 is None:
            flare_theta0 = theta                    # latch the pitch the flare starts from
        frac = (st["alt"] - (CRUISE_ALT - LEVEL_BAND)) / LEVEL_BAND
        s = _smooth(frac)
        target  = flare_theta0 * (1.0 - s)
        thr_cmd = THR_PWM + (CRUISE_CAP_THR - THR_PWM) * s   # full -> cruise throttle, smooth
    elif v_peak < V_R:
        # Ground roll: hold a slight nose-down so the airframe can't rotate before the
        # scheduled rotation begins at V_R (FBWA-at-0 lets the nose float up on its own).
        target = GND_PITCH
    else:
        # AoA guard: also never command past (FPA + AOA_CAP) so the wing can't be driven past
        # stall before the climb is established (the schedule alone over-rotates on the runway).
        target = min(theta_target(v_peak) - spd_trim, fpa + AOA_CAP)
        target = max(0.0, target)                   # never push the nose below horizon once climbing
    theta  = rate_limit(theta, target, dt)          # <- still enforces q <= RATE_LIM_DPS
    send_sticks(theta, thr=int(thr_cmd))
    if now - last > 2:
        print(f"CLIMB t={now-t:4.0f}s mode={c.flightmode or '?':5s} theta={theta:5.1f} "
              f"strim={spd_trim:4.1f} "
              f"alt={st['alt']:6.1f} as={st['as']:5.1f} Vpk={v_peak:4.1f} "
              f"thr={st['thr']:3.0f}% ch3={st['thr_pwm']} "
              f"climb={st['climb']:5.2f} FPA~{fpa:5.1f} cap={fpa+AOA_CAP:4.1f}deg", flush=True)
        last = now
    if st["alt"] >= CRUISE_ALT - 1:
        print(f"flared level at {st['alt']:.1f} m (theta={theta:.1f}) -> handing to CRUISE", flush=True)
        break
    time.sleep(0.05)                                 # ~20 Hz override stream

# 2b. SPEED CAPTURE (still FBWA): drive to the cruise TRIM (12 m/s, ~0 deg) before handing to TECS,
#     so TECS inherits a matched state instead of an energy error. theta = K_CAP*(V-CRUISE_AS)
#     (too fast -> nose up to bleed speed; stable, -> 0 at V=12) AND throttle dropped to the
#     level-trim value so theta actually settles at 0 (not holding 12 by climbing).
print("speed capture -> 12 m/s / 0 deg ...", flush=True)
tcap = time.time(); last = 0
while time.time() - tcap < 25:
    pump()
    theta_cap = max(-5.0, min(TH_CLIMB, K_CAP * (st["as"] - CRUISE_AS)))   # clamp: no big nose-down / over-climb
    send_sticks(theta_cap, thr=CRUISE_CAP_THR)
    now = time.time()
    if now - last > 1:
        print(f"CAPT t={now-tcap:4.0f}s theta={theta_cap:5.1f} as={st['as']:5.1f} "
              f"climb={st['climb']:5.2f} alt={st['alt']:6.1f} thr={st['thr']:3.0f}%", flush=True)
        last = now
    if abs(st["as"] - CRUISE_AS) < 0.5 and abs(st["climb"]) < 0.5:
        print(f"captured trim: as={st['as']:.1f} climb={st['climb']:.2f} -> handing to CRUISE", flush=True)
        break
    time.sleep(0.05)

# 3. CRUISE: switch while STREAMING continuously, picking the throttle-stick value by the CURRENT
#    mode. The throttle stick means different things per mode (FBWA: direct %; CRUISE: airspeed
#    demand), so:  still FBWA -> CRUISE_CAP_THR (~67% direct) holds the capture trim;
#                  in CRUISE  -> CRUISE_THR_PWM (12 m/s demand).
#    This never lapses the override and never lets CRUISE read the 67% capture stick as ~17 m/s
#    (which was slamming TECS at engagement). CH2 stays neutral = hold altitude.
cid = c.mode_mapping()["CRUISE"]
t = time.time(); last = 0
while time.time() - t < 40:
    pump()
    in_cruise = (c.flightmode == "CRUISE")
    if not in_cruise:
        c.set_mode(cid)                              # keep requesting until it flips
    send_sticks(0.0, thr=(CRUISE_THR_PWM if in_cruise else CRUISE_CAP_THR))
    now = time.time()
    if now - last > 2:
        print(f"CRUISE t={now-t:4.0f}s mode={c.flightmode or '?':7s} "
              f"alt={st['alt']:6.1f} as={st['as']:5.1f} climb={st['climb']:5.2f} ch3={st['thr_pwm']}", flush=True)
        last = now
    time.sleep(0.05)                                 # keep the override alive (~20 Hz)
release_sticks()                                     # window done -> hand the sticks back

print("DONE (FBWA open-loop climb to %.0f deg at <=%.0f deg/s; cruising ~%.0f m, %.0f m/s)"
      % (TH_CLIMB, RATE_LIM_DPS, CRUISE_ALT, CRUISE_AS), flush=True)

# 4. Plotting handled by the atexit hook (_plot_on_exit) registered at the top.