#!/usr/bin/env python3

"""
uSTOL mission driver (connects to an already-running SITL).

Sequence:
  1) set the climb params (10 m/s, 17.4576 deg flight-path angle)
  2) arm in FBWA, switch to TAKEOFF
  3) hand off to GUIDED at ~level-alt (9.5 m) -> straight climb toward a far/high
     anchor placed ON the FPA ray (DO_REPOSITION): L1 stays straight, TECS stays at CLMB_MAX
  4) level off at CRUISE_ALT (100 m) and accelerate to the cruise airspeed (12 m/s)

Run SITL first (separate terminal), e.g.:
  build/sitl/bin/arduplane -w --model plane \
      --defaults ustol_sims/params_ustol.parm --home 28.559741,77.11745,237,285 -I0
and have one GCS (MAVProxy/Mission Planner) on 5760 so this script can use 5762.

The FDM writes its per-run time-series CSV to ustol_sims/logs/ (Output.cpp default,
or $LAT_SIM_LOG_DIR if set). Plot the newest with:
  python3 Tools/autotest/monte_carlo/plot_case_full.py ustol_sims/logs
"""

from pymavlink import mavutil
import math, time, atexit

# Plot the run's CSV on EVERY exit — normal end, exception, or Ctrl-C — so you
# never lose the graphs by stopping the mission early. The FDM logs the CSV
# continuously while SITL runs, so the newest CSV in logs/ is always this run's.
_plotted = False
def _plot_on_exit():
    global _plotted
    if _plotted:
        return
    _plotted = True
    try:
        import plot_ustol, fft_pitch          # same folder -> importable
        print("case plot:", plot_ustol.plot(""), flush=True)   # "" = newest CSV
        print("fft  plot:", fft_pitch.analyze(""), flush=True)
    except Exception as e:
        print("plotting failed:", e, flush=True)
atexit.register(_plot_on_exit)


# A. INPUT PARAMS
HOME_LAT, HOME_LON = 28.559741, 77.11745
RWY_HDG       = 285.0
CLIMB_AS      = 11.0                          # airspeed held during the climb (FPA denominator)
CRUISE_AS     = 12.0                          # airspeed once levelled off in cruise
FPA_DEG       = 17.4576031                    # target flight-path angle (sin gamma = 0.3)
HANDOFF_ALT   = 10.0                          # TAKEOFF -> GUIDED handoff altitude (ray origin)
CRUISE_ALT    = 100.0                         # level off here and switch to cruise
CLIMB_TGT_ALT = 250.0                         # GUIDED climb-target altitude (far/high anchor)
CLMB_MAX      = round(CLIMB_AS * math.sin(math.radians(FPA_DEG)), 2)  # -> TECS_CLMB_MAX
# Horizontal distance to the climb anchor, placed exactly ON the FPA ray from the
# handoff point:  D = (alt_target - alt_handoff) / tan(gamma)  -> 1559.69 m for these values.
# Far enough that L1 never enters loiter and TECS stays saturated at CLMB_MAX through 100 m.
FAR_DIST      = round((CLIMB_TGT_ALT - HANDOFF_ALT) / math.tan(math.radians(FPA_DEG)), 2)

# B. CONNECTING TO MAVLINK
c = mavutil.mavlink_connection("tcp:127.0.0.1:5762", source_system=250)
c.recv_match(type="HEARTBEAT", blocking=True, timeout=5)     # wait until it talks
c.mav.request_data_stream_send(                              # ask for 10 Hz telemetry
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
    """Switch flight mode by name (e.g. 'TAKEOFF') and confirm via heartbeat."""
    mid = c.mode_mapping()[name]              # mode name -> numeric id
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
        force = 21196 if n > 4 else 0         # magic value = force arm
        c.mav.command_long_send(c.target_system, c.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, force, 0, 0, 0, 0, 0)          # param1=1 -> arm
        for _ in range(5):
            hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        time.sleep(1.5)
    return False

st = {"alt": 0.0, "as": 0.0, "climb": 0.0, "lat": None, "lon": None}
def pump():
    """Read one telemetry message and update state (position, alt AGL, airspeed, climb rate)."""
    m = c.recv_match(type=["GLOBAL_POSITION_INT", "VFR_HUD"], blocking=True, timeout=2)
    if m:
        if m.get_type() == "GLOBAL_POSITION_INT":
            st["alt"] = m.relative_alt / 1000.0        # mm -> m AGL
            st["lat"] = m.lat / 1e7                     # degE7 -> deg
            st["lon"] = m.lon / 1e7
        elif m.get_type() == "VFR_HUD":
            st["as"]    = m.airspeed                    # m/s
            st["climb"] = m.climb                       # m/s (vertical speed)
    return m

def offset(lat, lon, brg_deg, dist_m):
    """Return a lat/lon offset dist_m along bearing brg_deg from (lat,lon)."""
    b = math.radians(brg_deg)
    return (lat + (dist_m * math.cos(b)) / 111320.0,
            lon + (dist_m * math.sin(b)) / (111320.0 * math.cos(math.radians(lat))))

def reposition(lat, lon, alt):
    """GUIDED 'fly to here' via MAV_CMD_DO_REPOSITION (COMMAND_INT)."""
    c.mav.command_int_send(c.target_system, c.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
        -1, 0, 0, float('nan'),               # speed=default, bitmask, radius, yaw=default
        int(lat * 1e7), int(lon * 1e7), alt)  # x=lat(degE7), y=lon(degE7), z=alt(m)

# D. MISSION PROFILE

# 0. Params: define the 10 m/s, 17.46 deg climb (+ enablers)
for n, v in [
    ("ARMING_CHECK",     0),         # allow headless arming
    ("AIRSPEED_CRUISE",  CLIMB_AS),  # speed TECS holds  (= FPA denominator)
    ("TECS_CLMB_MAX",    CLMB_MAX),  # climb-rate cap    (= FPA numerator) -> 17.46 deg
    ("PTCH_LIM_MAX_DEG", 25),        # let pitch reach theta = gamma + alpha (~22-25 deg)
    ("TECS_PITCH_MAX",   25),        # TECS's own pitch ceiling
    ("TRIM_THROTTLE",    90),        # = measured steady-climb throttle (~90%); TECS FF baseline
                                     # matches the climb so the TAKEOFF->GUIDED handoff doesn't sag
    # --- handoff-smoothing: make TAKEOFF use the SAME throttle/climb regime as GUIDED ---
    ("TKOFF_OPTIONS",    1),         # bit0: let TECS govern takeoff throttle (THR_MIN..TKOFF_THR_MAX)
                                     # instead of forcing THR_MAX -> no throttle step at handoff
    ("TKOFF_THR_MAX",    90),        # cap takeoff throttle at the steady-climb value (= TRIM_THROTTLE)
    ("TKOFF_THR_MAX_T",  2),         # shorten the forced-max-throttle window at ground roll (was 4 s)
    ("TECS_CLMB_MAX",    CLMB_MAX),  # (already set above) climb cap also bounds the takeoff climb now
]:
    print(f"set {n}={v}: {set_param(n, v)}", flush=True)
print(f"-> FPA {FPA_DEG:.4f} deg via CLMB_MAX={CLMB_MAX} m/s @ {CLIMB_AS} m/s", flush=True)

# 1. Arm in FBWA
print("FBWA:", set_mode("FBWA"), flush=True)
print("armed:", arm(), flush=True)
print("TAKEOFF -> handoff at alt >= %.1f m" % HANDOFF_ALT, flush=True)

# 2. Takeoff and hand off logic
set_mode("TAKEOFF")
t = time.time()
while time.time() - t < 60:
    pump()
    if st["alt"] >= HANDOFF_ALT:
        print(f"handoff: alt={st['alt']:.1f} as={st['as']:.1f} -> GUIDED", flush=True)
        break

# 3. GUIDED straight climb toward a far, high anchor placed ON the 17.46 deg ray.
set_mode("GUIDED")
pump()                                        # refresh current position (= ray origin)
TGT_LAT, TGT_LON = offset(st["lat"], st["lon"], RWY_HDG, FAR_DIST)
print(f"GUIDED anchor: {FAR_DIST:.1f} m ahead @ {CLIMB_TGT_ALT:.0f} m "
      f"(on the {FPA_DEG:.2f} deg ray)", flush=True)
for _ in range(3):                            # send a few times in case of packet loss
    reposition(TGT_LAT, TGT_LON, CLIMB_TGT_ALT)
    time.sleep(0.3)

t = time.time(); last = 0
while time.time() - t < 120:
    pump()
    fpa = math.degrees(math.asin(max(-1, min(1, st["climb"] / max(st["as"], 1)))))
    if time.time() - last > 3:
        print(f"CLIMB  t={time.time()-t:4.0f}s mode={c.flightmode:7s} "
              f"alt={st['alt']:6.1f} as={st['as']:5.1f} "
              f"climb={st['climb']:5.2f} FPA~{fpa:5.1f}deg", flush=True)
        last = time.time()
    if st["alt"] >= CRUISE_ALT - 1:
        print(f"reached cruise alt {st['alt']:.1f} m -> levelling off", flush=True)
        break

# 4. CRUISE: hand off to ArduPlane CRUISE mode and accelerate to CRUISE_AS.
set_param("AIRSPEED_CRUISE", CRUISE_AS)
set_mode("CRUISE")

t = time.time(); last = 0
while time.time() - t < 60:
    pump()
    if time.time() - last > 3:
        print(f"CRUISE t={time.time()-t:4.0f}s mode={c.flightmode:7s} "
              f"alt={st['alt']:6.1f} as={st['as']:5.1f} climb={st['climb']:5.2f}", flush=True)
        last = time.time()
    if abs(st["as"] - CRUISE_AS) < 1 and abs(st["climb"]) < 0.5:
        print(f"cruise established: alt={st['alt']:.1f} m  as={st['as']:.1f} m/s", flush=True)
        break

print("DONE (climb FPA target %.2f deg; cruising at ~%.0f m, %.0f m/s)"
      % (FPA_DEG, CRUISE_ALT, CRUISE_AS), flush=True)

# 5. Plotting is handled by the atexit hook (_plot_on_exit) registered at the top,
#    so the graphs are produced no matter how the script ends (incl. Ctrl-C).