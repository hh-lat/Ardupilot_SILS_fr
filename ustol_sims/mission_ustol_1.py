#!/usr/bin/env python3

"""
uSTOL mission driver (connects to an already-running SITL).

Sequence — AUTO takeoff + climb, then CRUISE:
  1) set the climb params (11 m/s, 17.4576 deg flight-path angle) + takeoff params
  2) upload a 3-item AUTO mission: home / NAV_TAKEOFF / a FAR+HIGH waypoint up on the
     17.46 deg ray (so L1 flies straight toward it and never loiters at 100 m)
  3) arm in FBWA, switch to AUTO -> takeoff then straight climb toward the anchor
  4) when alt reaches CRUISE_ALT (100 m), switch to CRUISE and accelerate to 12 m/s

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
CLIMB_AS      = 10.0                          # airspeed held during the climb (FPA denominator)
CRUISE_AS     = 12.0                          # airspeed once levelled off in cruise
FPA_DEG       = 15.0                          # target flight-path angle (sin gamma = 0.3)
HANDOFF_ALT   = 10.0                          # TAKEOFF -> GUIDED handoff altitude (ray origin)
CRUISE_ALT    = 100.0                         # switch to CRUISE when alt reaches this
CLIMB_TGT_ALT = 250.0                         # climb-WP altitude (far/high anchor, ON the FPA ray)
TKOFF_PITCH   = 8.0                           # NAV_TAKEOFF min pitch (~ your 8 deg liftoff attitude)
TKOFF_ALT     = 20.0                          # end of the takeoff segment
CLMB_MAX      = round(CLIMB_AS * math.sin(math.radians(FPA_DEG)), 2)
FAR_DIST      = round((CLIMB_TGT_ALT - TKOFF_ALT) / math.tan(math.radians(FPA_DEG)), 2)

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
            st["alt"] = m.relative_alt / 1000.0         # mm -> m AGL
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

def mission_item(seq, cmd, p1=0.0, p2=0.0, p3=0.0, p4=float('nan'),
                 lat=0.0, lon=0.0, alt=0.0, frame=None, current=0, autocont=1):
    """Build one MISSION_ITEM_INT. Default frame = relative-alt (AGL)."""
    if frame is None:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        c.target_system, c.target_component, seq, frame, cmd,
        current, autocont, p1, p2, p3, p4,
        int(lat * 1e7), int(lon * 1e7), float(alt),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

def upload_mission(items):
    """Clear, send the count, then serve each item as the autopilot requests it.

    Robust to the clear's own MISSION_ACK: we drain stale mission msgs first and
    IGNORE any ACK until we've served the last item, so we never return after 0 items.
    """
    # 1) drain any stale mission traffic from a prior attempt
    while c.recv_match(type=["MISSION_ACK", "MISSION_REQUEST", "MISSION_REQUEST_INT"],
                       blocking=False):
        pass
    # 2) clear, and explicitly consume the clear's ACK
    c.mav.mission_clear_all_send(c.target_system, c.target_component,
                                 mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    c.recv_match(type="MISSION_ACK", blocking=True, timeout=3)
    # 3) announce the count, then serve each requested item
    c.mav.mission_count_send(c.target_system, c.target_component,
                             len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent_last = False
    t = time.time()
    while time.time() - t < 20:
        m = c.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                         blocking=True, timeout=3)
        if not m:
            continue
        if m.get_type() == "MISSION_ACK":
            if sent_last:                       # only trust an ACK after the last item went out
                return m.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
            continue                            # else it's the clear's ack -> ignore
        c.mav.send(items[m.seq])                # autopilot asks for seq -> send that item
        if m.seq == len(items) - 1:
            sent_last = True
    return False

def download_mission():
    """Read back what's actually onboard so we can verify the upload (returns list of items)."""
    c.mav.mission_request_list_send(c.target_system, c.target_component)
    cnt = c.recv_match(type="MISSION_COUNT", blocking=True, timeout=3)
    n = cnt.count if cnt else 0
    out = []
    for i in range(n):
        c.mav.mission_request_int_send(c.target_system, c.target_component, i)
        it = c.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=3)
        if it:
            out.append(it)
    return out

# D. MISSION PROFILE

# 0. Params: takeoff (rotate@6) + the 11 m/s, 17.46 deg climb.
for n, v in [
    ("ARMING_CHECK",     0),         # allow headless arming
    ("TKOFF_ROTATE_SPD", 6),         # rotate at 6 m/s
    ("TKOFF_LVL_ALT",    15),        # hold wings level (no bank) to 15 m -> straight climb-out
    ("AIRSPEED_MIN",     7),         # 8 m/s liftoff sits above the floor (not flagged)
    ("AIRSPEED_CRUISE",  CLIMB_AS),  # speed TECS holds during climb (= FPA denominator)
    ("TECS_CLMB_MAX",    CLMB_MAX),  # climb-rate cap (= FPA numerator) -> sin g = 3.3/11 = 0.3
    ("PTCH_LIM_MAX_DEG", 18),        # cap near trim climb attitude (~15 deg) -> stop the over-rotation
    ("TECS_PITCH_MAX",   18),        # TECS's own pitch ceiling, matched to the cap
    ("TRIM_THROTTLE",    90),        # = measured steady-climb throttle (~90%); TECS FF baseline
]:
    print(f"set {n}={v}: {set_param(n, v)}", flush=True)
print(f"-> FPA {FPA_DEG:.4f} deg via CLMB_MAX={CLMB_MAX} m/s @ {CLIMB_AS} m/s", flush=True)

# 1. Build + upload the AUTO mission: home / takeoff / FAR+HIGH climb anchor on the ray.
clb_lat, clb_lon = offset(HOME_LAT, HOME_LON, RWY_HDG, FAR_DIST)
items = [
    mission_item(0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,            # seq 0 = home (convention)
                 lat=HOME_LAT, lon=HOME_LON, alt=0,
                 frame=mavutil.mavlink.MAV_FRAME_GLOBAL, current=1),
    mission_item(1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,            # rotate@6, climb-out >= 8 deg
                 p1=TKOFF_PITCH, lat=0, lon=0, alt=TKOFF_ALT),
    mission_item(2, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,          # FAR+HIGH anchor: straight climb,
                 lat=clb_lat, lon=clb_lon, alt=CLIMB_TGT_ALT),      # never reached before we hit 100 m
]
print(f"mission uploaded: {upload_mission(items)}  "
      f"(anchor {FAR_DIST:.0f} m ahead @ {CLIMB_TGT_ALT:.0f} m)", flush=True)

# verify what's actually onboard — if this isn't 3 items ending in the anchor WP,
# that's why it circles the takeoff point (mission ended at takeoff).
ob = download_mission()
print(f"onboard mission: {len(ob)} items", flush=True)
for it in ob:
    print(f"  seq{it.seq} cmd={it.command} frame={it.frame} "
          f"lat={it.x/1e7:.6f} lon={it.y/1e7:.6f} alt={it.z:.0f}", flush=True)
if len(ob) < 3:
    print("!! mission did NOT upload fully -> AUTO will loiter at takeoff. Fix before flying.", flush=True)

# 2. Arm in FBWA, then AUTO. Autopilot does takeoff -> straight climb toward the anchor.
print("FBWA:", set_mode("FBWA"), flush=True)
print("armed:", arm(), flush=True)
print("AUTO:",  set_mode("AUTO"), flush=True)

t = time.time(); last = 0
while time.time() - t < 120:
    pump()
    fpa = math.degrees(math.asin(max(-1, min(1, st["climb"] / max(st["as"], 1)))))
    if time.time() - last > 3:
        cur = c.recv_match(type="MISSION_CURRENT", blocking=False)
        seq = cur.seq if cur else "?"
        print(f"CLIMB  t={time.time()-t:4.0f}s mode={c.flightmode:7s} wp={seq} "
              f"alt={st['alt']:6.1f} as={st['as']:5.1f} "
              f"climb={st['climb']:5.2f} FPA~{fpa:5.1f}deg", flush=True)
        last = time.time()
    if st["alt"] >= CRUISE_ALT - 1:
        print(f"reached cruise alt {st['alt']:.1f} m -> levelling off", flush=True)
        break

# 3. CRUISE: hand off to ArduPlane CRUISE mode and accelerate to CRUISE_AS.
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

# 4. Plotting handled by the atexit hook (_plot_on_exit) registered at the top.