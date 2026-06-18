#!/usr/bin/env python3
"""
auto_flight4.py  -  minimal-input full-mission runner for the tuned uSTOL FDM.

This is a slimmed-down sibling of auto_flight3.py.  The aircraft is ALREADY tuned
and its parameters are ALREADY loaded on the autopilot, so this script:

  * does NOT re-load / restore / back up any parameters (no gains, no servo map,
    no reversals, no envelope) -- it trusts the config already on the vehicle,
  * only asks you for the things that change run-to-run:
        - the takeoff altitude (TKOFF_ALT), and
        - the durations of the CRUISE / FBWA / FBWB / LOITER phases.
    No cruise-speed, climb-rate/angle, throttle or landing prompts.
  * always logs full telemetry to a timestamped CSV at ~5 Hz, and
  * PLOTS the recorded states automatically when the mission finishes OR when you
    Ctrl-C / it aborts -- i.e. on ANY exit it draws and shows the same 5-panel
    figure that plot_flight.py produces (and saves it as a PNG next to the CSV).

The mission flown, in order:

    PHASE 1  TAKEOFF   ground roll, rotation, auto-climb to the takeoff altitude
    PHASE 2  CRUISE    CRUISE mode (auto heading + altitude + airspeed hold)
    PHASE 3  FBWA      Fly-By-Wire-A, wings level, held throttle
    PHASE 4  FBWB      Fly-By-Wire-B, altitude hold + auto throttle (TECS)
    PHASE 5  LOITER    automatic circle at the loaded loiter radius
    PHASE 6  LAND      AUTO autoland (approach + LAND) using the loaded land params

It is a CLIENT: start the simulator first, e.g.

    ./Tools/autotest/sim_vehicle.py -v ArduPlane --console --map

then in another terminal:

    ./Tools/autotest/auto_flight4.py             # interactive (5 short questions)
    ./Tools/autotest/auto_flight4.py --defaults  # no prompts, built-in durations

By default it connects to SITL's spare MAVLink TCP port 5762 (SERIAL1) so it does
NOT fight MAVProxy on 5760.  See SOP_auto_flight4.md for full run instructions.
"""

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass

from pymavlink import mavutil

# --- physical / detection constants ---------------------------------------- #
EARTH_RADIUS_M = 6371000.0
RC_NEUTRAL = 1500              # us: centred stick
RAD2DEG = 180.0 / math.pi
SCREEN_HEIGHT_M = 15.0         # ~50 ft obstacle clearance height for distances
LIFTOFF_ALT_M = 0.8           # alt (AGL) above which we consider the wheels off
TOUCHDOWN_ALT_M = 1.0         # alt (AGL) below which we consider it touched down
GROUND_STOP_SPEED = 1.0       # m/s ground speed below which we consider it stopped
LANDED_DISARM_ALT = 5.0       # disarm below this alt == real landing; above == anomaly
AIL_CH, ELEV_CH, RUD_CH = 10, 11, 12   # SERVO_OUTPUT_RAW output channels (this airframe)

# Landing geometry (fixed -- not asked for; the FLARE/airspeed/sink tuning is taken
# from whatever is already loaded on the vehicle, NOT set here).
APPROACH_DIST_M = 600.0       # m, distance of approach WP south of home
APPROACH_ALT_M = 60.0         # m, altitude at the approach WP
# Held throttle during FBWA (FBWA has no auto-throttle); fixed, not asked for.
FBWA_THROTTLE_PCT = 60


class MissionAborted(Exception):
    """Raised when the aircraft crashes/disarms in flight, to stop the mission early."""


# --------------------------------------------------------------------------- #
#  Console / geometry helpers
# --------------------------------------------------------------------------- #
def banner(text):
    line = "=" * 78
    print("\n" + line + "\n  " + text + "\n" + line)


def ask(prompt, default, cast=str):
    """Prompt with a default (Enter accepts it). Re-asks on bad input."""
    while True:
        raw = input("  %s [%s]: " % (prompt, default)).strip()
        if raw == "":
            return default
        try:
            return cast(raw)
        except (ValueError, TypeError):
            print("    not a valid value, try again")


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two lat/lon points."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def offset_latlon(lat, lon, north_m, east_m):
    """Offset a lat/lon by metres north/east (flat-earth, fine for short legs)."""
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def fmt(value, width=7, prec=1):
    """Format an optional float, printing 'n/a' when it is None."""
    if value is None:
        return "%*s" % (width, "n/a")
    return "%*.*f" % (width, prec, value)


# --------------------------------------------------------------------------- #
#  Mission parameters  (ONLY the run-to-run knobs; no tuning is set here)
# --------------------------------------------------------------------------- #
@dataclass
class MissionParams:
    connect: str = "tcp:127.0.0.1:5762"
    takeoff_alt: float = 80.0       # m, TKOFF_ALT (TAKEOFF mode completes here; CRUISE holds it)
    cruise_time: float = 30.0       # s in CRUISE mode
    fbwa_time: float = 20.0         # s in FBWA mode
    fbwb_time: float = 20.0         # s in FBWB mode
    loiter_time: float = 30.0       # s in LOITER mode


def prompt_parameters():
    """Interactively collect ONLY the takeoff altitude and the phase durations."""
    banner("MISSION SETUP  -  press Enter to accept each default")
    print("  (parameters are NOT reloaded; the tuned config already on the vehicle is used)")
    p = MissionParams()
    p.connect = ask("MAVLink connection", p.connect)
    print()
    p.takeoff_alt = ask("Takeoff altitude (m)  [CRUISE then holds this height]", p.takeoff_alt, float)
    p.cruise_time = ask("CRUISE phase duration (s)", p.cruise_time, float)
    p.fbwa_time = ask("FBWA phase duration (s)", p.fbwa_time, float)
    p.fbwb_time = ask("FBWB phase duration (s)", p.fbwb_time, float)
    p.loiter_time = ask("LOITER phase duration (s)", p.loiter_time, float)
    return p


def summarise(p, csv_path):
    banner("MISSION PLAN")
    print("  connect ............ %s" % p.connect)
    print("  takeoff alt ........ %.0f m  (TAKEOFF completion; CRUISE holds it)" % p.takeoff_alt)
    print("  CRUISE/FBWA/FBWB/LOITER  %.0f / %.0f / %.0f / %.0f s"
          % (p.cruise_time, p.fbwa_time, p.fbwb_time, p.loiter_time))
    print("  FBWA throttle ...... %d %%   (fixed)" % FBWA_THROTTLE_PCT)
    print("  landing ............ AUTO  (approach %.0f m out @ %.0f m; flare/airspeed = loaded params)"
          % (APPROACH_DIST_M, APPROACH_ALT_M))
    print("  params ............. NOT reloaded (only TKOFF_ALT is set to the value above)")
    print("  CSV log ............ %s  (states are plotted on exit)" % csv_path)


# --------------------------------------------------------------------------- #
#  Take-off / landing performance metrics
# --------------------------------------------------------------------------- #
@dataclass
class Performance:
    ground_roll_m: float = None        # standing start -> lift-off
    takeoff_dist_m: float = None       # standing start -> screen height
    liftoff_speed: float = None        # airspeed at lift-off (Vlof)
    liftoff_time: float = None
    touchdown_speed: float = None      # airspeed at touchdown (Vtd)
    touchdown_time: float = None
    landing_roll_m: float = None       # touchdown -> full stop
    landing_dist_m: float = None       # screen height -> full stop
    _ld_screen: tuple = None
    _touchdown_pos: tuple = None


# --------------------------------------------------------------------------- #
#  Plane controller  (pymavlink wrapper with cached telemetry + logging)
# --------------------------------------------------------------------------- #
class Plane:
    def __init__(self, connect, csv_path, print_interval=2.0):
        self.connect = connect
        self.m = None
        self.print_interval = print_interval

        # cached telemetry
        self.alt = None            # relative altitude (AGL), m
        self.lat = self.lon = None
        self.airspeed = self.groundspeed = 0.0
        self.heading = self.throttle = 0.0
        self.roll = self.pitch = self.yaw = 0.0           # rad
        self.p = self.q = self.r = 0.0                    # rad/s body rates
        self.vN = self.vE = self.vD = 0.0                 # m/s NED
        self.armed = False
        self.custom_mode = None
        self.crashed = False
        self.navroll = self.navpitch = 0.0                # deg, commanded attitude
        self.ail = self.elev = self.rud = 0               # us, servo outputs
        self.aoa = self.ssa = 0.0                         # deg, angle of attack / sideslip
        self._have_aoa = False                            # True once AOA_SSA is streamed
        self.airborne = False
        self.phase = ""

        self.home_lat = self.home_lon = None
        self.mode_names = {}

        # timing / logging
        self.t0 = None
        self._last_print = 0.0
        self._last_csv = 0.0
        self._last_status = ""
        self.csv_path = csv_path
        self.csv_file = None
        self.csv_writer = None
        self.rows_logged = 0

        # performance / event tracking
        self.perf = Performance()
        self.landing_active = False
        self.prev_alt = None

    # -- lifecycle ---------------------------------------------------------- #
    def open(self):
        print("Connecting to %s ..." % self.connect)
        self.m = mavutil.mavlink_connection(self.connect)
        print("Waiting for heartbeat...")
        self.m.wait_heartbeat()
        print("  heartbeat from system %u component %u"
              % (self.m.target_system, self.m.target_component))
        self.mode_names = {v: k for k, v in (self.m.mode_mapping() or {}).items()}
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        for msgid in (62, 36, 11020):   # NAV_CONTROLLER_OUTPUT, SERVO_OUTPUT_RAW, AOA_SSA
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid, 100000, 0, 0, 0, 0, 0)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "time_s", "phase", "mode", "armed", "lat", "lon", "alt_agl_m",
            "airspeed", "groundspeed", "roll_deg", "pitch_deg", "yaw_deg",
            "p_dps", "q_dps", "r_dps", "gamma_deg", "vN", "vE", "vD",
            "climb_mps", "throttle_pct",
            "navroll_deg", "navpitch_deg", "ail_out", "elev_out", "rud_out",
            "aoa_deg", "ssa_deg"])
        print("  logging telemetry to %s" % self.csv_path)

    def close(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None

    @property
    def tsys(self):
        return self.m.target_system

    @property
    def tcomp(self):
        return self.m.target_component

    def _t(self):
        return 0.0 if self.t0 is None else time.time() - self.t0

    def mode_name(self):
        return self.mode_names.get(self.custom_mode, str(self.custom_mode))

    # -- message pump ------------------------------------------------------- #
    def pump(self, seconds=0.1):
        end = time.time() + seconds
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                if time.time() >= end:
                    return
                time.sleep(0.003)
                continue
            self._ingest(msg)

    def _ingest(self, msg):
        # Only trust messages from the autopilot we connected to.  On a routed
        # SITL link a GCS/MAVProxy HEARTBEAT (mode=MANUAL=0, disarmed) otherwise
        # gets ingested and flips our cached mode/armed every other sample.
        if msg.get_srcSystem() != self.tsys:
            return
        t = msg.get_type()
        if t == "HEARTBEAT":
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return                                  # ignore ground-station heartbeats
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.custom_mode = msg.custom_mode
        elif t == "GLOBAL_POSITION_INT":
            self.alt = msg.relative_alt / 1000.0
            self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
            self.vN, self.vE, self.vD = msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0
        elif t == "VFR_HUD":
            self.airspeed = msg.airspeed
            self.groundspeed = msg.groundspeed
            self.heading = msg.heading
            self.throttle = msg.throttle
        elif t == "ATTITUDE":
            self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
            self.p, self.q, self.r = msg.rollspeed, msg.pitchspeed, msg.yawspeed
        elif t == "NAV_CONTROLLER_OUTPUT":
            self.navroll, self.navpitch = msg.nav_roll, msg.nav_pitch
        elif t == "SERVO_OUTPUT_RAW":
            self.ail = getattr(msg, "servo%d_raw" % AIL_CH, 0)
            self.elev = getattr(msg, "servo%d_raw" % ELEV_CH, 0)
            self.rud = getattr(msg, "servo%d_raw" % RUD_CH, 0)
        elif t == "AOA_SSA":
            self.aoa = getattr(msg, "AOA", 0.0)            # angle of attack, deg
            self.ssa = getattr(msg, "SSA", 0.0)            # sideslip angle, deg
            self._have_aoa = True
        elif t == "STATUSTEXT":
            self._handle_statustext(msg)

    def _handle_statustext(self, msg):
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode("ascii", "ignore")
        text = text.strip("\x00").strip()
        low = text.lower()
        if "crash" in low:
            self.crashed = True
        keys = ("prearm", "arm", "disarm", "crash", "land", "takeoff", "stall",
                "error", "fail", "fence", "ekf", "gps", "throttle", "glide")
        if any(k in low for k in keys) and text != self._last_status:
            print("    [AP] %s" % text)
            self._last_status = text

    # -- derived quantities ------------------------------------------------- #
    def _alpha_beta(self):
        """Angle of attack and sideslip (deg). Uses the autopilot's AOA_SSA estimate
        when it is being streamed; otherwise derives them from the body-frame
        velocity (exact in calm air, slightly off in wind)."""
        if self._have_aoa:
            return self.aoa, self.ssa
        phi, th, psi = self.roll, self.pitch, self.yaw
        vN, vE, vD = self.vN, self.vE, self.vD
        cphi, sphi = math.cos(phi), math.sin(phi)
        cth, sth = math.cos(th), math.sin(th)
        cpsi, spsi = math.cos(psi), math.sin(psi)
        u = cth * cpsi * vN + cth * spsi * vE - sth * vD
        v = ((sphi * sth * cpsi - cphi * spsi) * vN
             + (sphi * sth * spsi + cphi * cpsi) * vE + sphi * cth * vD)
        w = ((cphi * sth * cpsi + sphi * spsi) * vN
             + (cphi * sth * spsi - sphi * cpsi) * vE + cphi * cth * vD)
        speed = math.sqrt(u * u + v * v + w * w)
        alpha = math.degrees(math.atan2(w, u)) if abs(u) > 0.1 else 0.0
        beta = math.degrees(math.asin(max(-1.0, min(1.0, v / speed)))) if speed > 0.5 else 0.0
        return alpha, beta

    def derived(self):
        climb = -self.vD
        horiz = math.hypot(self.vN, self.vE)
        gamma = math.degrees(math.atan2(climb, horiz)) if (horiz > 0.1 or abs(climb) > 0.1) else 0.0
        aoa, ssa = self._alpha_beta()
        return {
            "roll": self.roll * RAD2DEG, "pitch": self.pitch * RAD2DEG, "yaw": self.yaw * RAD2DEG,
            "p": self.p * RAD2DEG, "q": self.q * RAD2DEG, "r": self.r * RAD2DEG,
            "gamma": gamma, "climb": climb, "aoa": aoa, "ssa": ssa,
            "vN": self.vN, "vE": self.vE, "vD": self.vD,
        }

    # -- the master tick: pump, track events, print + log on cadence -------- #
    def tick(self, phase, note=""):
        self.phase = phase
        self.pump(0.1)
        self._track_events()
        self._check_alive()
        now = time.time()
        if self.csv_writer and now - self._last_csv >= 0.2:        # ~5 Hz
            self._write_csv(phase)
            self._last_csv = now
        if now - self._last_print >= self.print_interval:
            self._print_telemetry(phase, note)
            self._last_print = now

    def _print_telemetry(self, phase, note):
        d = self.derived()
        print("  t=%6.1f %-7s alt=%6.1f AS=%5.1f | roll %6.1f (cmd %6.1f) pitch %6.1f yaw %6.1f"
              " | aoa %5.1f ssa %5.1f | pqr %5.0f/%5.0f/%5.0f | g=%5.1f | thr %3.0f%% %s"
              % (self._t(), phase, self.alt or 0.0, self.airspeed,
                 d["roll"], self.navroll, d["pitch"], d["yaw"],
                 d["aoa"], d["ssa"], d["p"], d["q"], d["r"], d["gamma"], self.throttle, note))

    def _write_csv(self, phase):
        d = self.derived()
        self.csv_writer.writerow([
            "%.2f" % self._t(), phase, self.mode_name(), int(self.armed),
            "%.7f" % (self.lat or 0.0), "%.7f" % (self.lon or 0.0), "%.2f" % (self.alt or 0.0),
            "%.2f" % self.airspeed, "%.2f" % self.groundspeed,
            "%.2f" % d["roll"], "%.2f" % d["pitch"], "%.2f" % d["yaw"],
            "%.2f" % d["p"], "%.2f" % d["q"], "%.2f" % d["r"],
            "%.2f" % d["gamma"], "%.2f" % d["vN"], "%.2f" % d["vE"], "%.2f" % d["vD"],
            "%.2f" % d["climb"], "%.0f" % self.throttle,
            "%.2f" % self.navroll, "%.2f" % self.navpitch,
            self.ail, self.elev, self.rud,
            "%.2f" % d["aoa"], "%.2f" % d["ssa"]])
        self.csv_file.flush()
        self.rows_logged += 1

    def _track_events(self):
        if self.lat is None or self.alt is None or self.home_lat is None:
            return
        dist_from_home = haversine_m(self.home_lat, self.home_lon, self.lat, self.lon)
        alt = self.alt
        if not self.landing_active:
            if self.perf.ground_roll_m is None and alt >= LIFTOFF_ALT_M:
                self.perf.ground_roll_m = dist_from_home
                self.perf.liftoff_speed = self.airspeed
                self.perf.liftoff_time = self._t()
            if self.perf.takeoff_dist_m is None and alt >= SCREEN_HEIGHT_M:
                self.perf.takeoff_dist_m = dist_from_home
        else:
            if (self.perf._ld_screen is None and self.prev_alt is not None
                    and self.prev_alt >= SCREEN_HEIGHT_M and alt < SCREEN_HEIGHT_M):
                self.perf._ld_screen = (self.lat, self.lon)
            if self.perf._touchdown_pos is None and alt <= TOUCHDOWN_ALT_M:
                self.perf._touchdown_pos = (self.lat, self.lon)
                self.perf.touchdown_speed = self.airspeed
                self.perf.touchdown_time = self._t()
            if (self.perf._touchdown_pos is not None and self.perf.landing_roll_m is None
                    and self.groundspeed < GROUND_STOP_SPEED):
                self.perf.landing_roll_m = haversine_m(
                    self.perf._touchdown_pos[0], self.perf._touchdown_pos[1],
                    self.lat, self.lon)
                if self.perf._ld_screen is not None:
                    self.perf.landing_dist_m = haversine_m(
                        self.perf._ld_screen[0], self.perf._ld_screen[1],
                        self.lat, self.lon)
        self.prev_alt = alt

    def _check_alive(self):
        """Abort the mission if the aircraft crashed/disarmed while airborne."""
        if self.alt is not None and self.alt > 8.0:
            self.airborne = True
        if self.phase == "LAND":
            return                          # a disarm during landing is expected
        if self.crashed:
            raise MissionAborted("autopilot reported a CRASH")
        if self.airborne and not self.armed:
            raise MissionAborted("disarmed in flight (crash / failsafe)")

    # -- parameters (ONLY TKOFF_ALT is ever set by this script) ------------- #
    def set_param(self, name, value):
        print("  param %-18s = %s" % (name, value))
        self.m.mav.param_set_send(
            self.tsys, self.tcomp, name.encode("ascii"),
            float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and msg.param_id.strip("\x00") == name:
                return
        print("    (no confirmation for %s -- name may not exist in this build)" % name)

    # -- modes / arming ----------------------------------------------------- #
    def set_mode(self, mode_name):
        mapping = self.m.mode_mapping() or {}
        if mode_name not in mapping:
            print("ERROR: mode %r not available. Known: %s"
                  % (mode_name, ", ".join(sorted(mapping))))
            raise SystemExit(1)
        mode_id = mapping[mode_name]
        print("Mode -> %s" % mode_name)
        self.m.set_mode(mode_id)
        deadline = time.time() + 5
        while time.time() < deadline:
            self.pump(0.2)
            if self.custom_mode == mode_id:
                return
        print("  WARNING: mode %s not confirmed" % mode_name)

    def wait_ready_and_position(self, timeout=90):
        print("Waiting for position / EKF to settle...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.pump(0.5)
            if self.lat is not None and self.alt is not None:
                print("  position acquired")
                return True
        print("  WARNING: no position fix after %ds" % timeout)
        return False

    def arm(self, timeout=30):
        print("Arming...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.m.mav.command_long_send(
                self.tsys, self.tcomp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            inner = time.time() + 3
            while time.time() < inner:
                self.pump(0.3)
                if self.armed:
                    print("  ARMED")
                    return True
        print("  could not arm (check pre-arm messages above)")
        return False

    def capture_home(self):
        self.home_lat, self.home_lon = self.lat, self.lon
        print("  home / standing-start datum: %.7f, %.7f" % (self.home_lat, self.home_lon))

    # -- RC override -------------------------------------------------------- #
    def rc(self, roll=0, pitch=0, throttle=0, yaw=0):
        self.m.mav.rc_channels_override_send(
            self.tsys, self.tcomp, roll, pitch, throttle, yaw, 0, 0, 0, 0)

    def release_rc(self):
        self.rc(0, 0, 0, 0)

    # -- altitude wait ------------------------------------------------------ #
    def wait_climb_to(self, target, phase, tol=3.0, timeout=240):
        end = time.time() + timeout
        while time.time() < end:
            self.tick(phase, "-> %.0f m" % target)
            if self.alt is not None and self.alt >= target - tol:
                print("  reached %.1f m" % self.alt)
                return True
            if self.crashed:
                print("  CRASH reported during climb -- aborting wait")
                return False
        print("  TIMEOUT before %.0f m (at %.1f m)" % (target, self.alt or 0))
        return False

    # -- fly a mode for a fixed time --------------------------------------- #
    def fly_mode(self, mode, duration, phase, hold=None):
        """hold=None -> automatic mode, RC released;  hold={..} -> held sticks."""
        self.set_mode(mode)
        if hold is None:
            self.release_rc()
        end = time.time() + duration
        while time.time() < end:
            if hold is not None:
                self.rc(**hold)
            self.tick(phase, "%4.0fs left" % (end - time.time()))
        if hold is not None:
            self.release_rc()

    # -- mission upload (AUTO autoland) ------------------------------------ #
    def upload_mission(self, items):
        n = len(items)
        print("  uploading %d-item landing mission..." % n)
        self.m.mav.mission_count_send(
            self.tsys, self.tcomp, n, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent = set()
        deadline = time.time() + 20
        while len(sent) < n and time.time() < deadline:
            msg = self.m.recv_match(
                type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ACK":
                break
            it = items[msg.seq]
            self.m.mav.mission_item_int_send(
                self.tsys, self.tcomp, msg.seq, it["frame"], it["cmd"],
                it.get("current", 0), it.get("autocontinue", 1),
                it.get("p1", 0), it.get("p2", 0), it.get("p3", 0), it.get("p4", 0),
                int(it["lat"] * 1e7), int(it["lon"] * 1e7), float(it["alt"]),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            sent.add(msg.seq)
        self.m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
        print("  mission uploaded")

    def set_current_wp(self, seq):
        self.m.mav.mission_set_current_send(self.tsys, self.tcomp, seq)

    def monitor_landing(self, timeout=300):
        """Watch the descent.  Distinguishes a real touchdown from a disarm at
        altitude (crash / failsafe)."""
        print("Monitoring descent to touchdown...")
        self.landing_active = True
        end = time.time() + timeout
        while time.time() < end:
            self.tick("LAND")
            if self.crashed:
                print("  RESULT: CRASH reported by autopilot at %.1f m." % (self.alt or 0))
                return "crash"
            if not self.armed:
                if (self.alt or 99.0) <= LANDED_DISARM_ALT:
                    print("  RESULT: LANDED (disarmed at %.1f m)." % (self.alt or 0))
                    return "landed"
                print("  RESULT: ANOMALY -- disarmed at %.1f m (NOT a landing; "
                      "likely crash/failsafe)." % (self.alt or 0))
                return "anomaly"
            if self.perf.landing_roll_m is not None:
                print("  RESULT: LANDED and stopped on the runway.")
                return "landed"
        print("  RESULT: TIMEOUT -- still airborne at %.1f m." % (self.alt or 0))
        return "timeout"

    # -- performance report ------------------------------------------------- #
    def print_performance(self, land_result):
        pf = self.perf
        lines = [
            "TAKEOFF",
            "  ground roll (start -> lift-off) .. %s m" % fmt(pf.ground_roll_m),
            "  takeoff distance (-> %4.0f m screen) %s m" % (SCREEN_HEIGHT_M, fmt(pf.takeoff_dist_m)),
            "  lift-off speed (Vlof) ............ %s m/s" % fmt(pf.liftoff_speed),
        ]
        if pf.liftoff_time is not None:
            lines.append("  time to lift-off ................. %s s" % fmt(pf.liftoff_time))
        lines += [
            "LANDING  (result: %s)" % land_result.upper(),
            "  touchdown speed (Vtd) ............ %s m/s" % fmt(pf.touchdown_speed),
            "  landing ground roll (td -> stop) . %s m" % fmt(pf.landing_roll_m),
            "  landing distance (%4.0f m -> stop) . %s m" % (SCREEN_HEIGHT_M, fmt(pf.landing_dist_m)),
        ]
        incomplete = ("anomaly", "crash", "timeout", "crashed-earlier", "incomplete", "interrupted")
        if land_result in incomplete or land_result.startswith("aborted"):
            lines.append("  (landing metrics incomplete -- no normal touchdown-and-stop)")

        banner("PERFORMANCE SUMMARY")
        for line in lines:
            print("  " + line)
        try:
            with open(self.csv_path + ".summary.txt", "w") as f:
                f.write("PERFORMANCE SUMMARY\n" + "\n".join(lines) + "\n")
            print("  (summary also saved to %s.summary.txt)" % self.csv_path)
        except OSError:
            pass


# --------------------------------------------------------------------------- #
#  Flight orchestration
# --------------------------------------------------------------------------- #
def do_landing(plane):
    banner("PHASE 6/6  LAND  (AUTO)")
    if not plane.armed:
        print("  aircraft is already DISARMED entering the landing phase -- it crashed")
        print("  or hit a failsafe in an earlier phase. Skipping landing.")
        return "crashed-earlier"
    if plane.home_lat is None:
        print("  no home position; falling back to RTL")
        plane.set_mode("RTL")
        return "rtl"
    # Approach waypoint placed APPROACH_DIST_M south of home; LAND at home.
    appr_lat, appr_lon = offset_latlon(plane.home_lat, plane.home_lon, -APPROACH_DIST_M, 0.0)
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    items = [
        dict(frame=frame, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   # seq 0 = home
             lat=plane.home_lat, lon=plane.home_lon, alt=0),
        dict(frame=frame, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   # seq 1 = approach
             lat=appr_lat, lon=appr_lon, alt=APPROACH_ALT_M),
        dict(frame=frame, cmd=mavutil.mavlink.MAV_CMD_NAV_LAND,       # seq 2 = land
             lat=plane.home_lat, lon=plane.home_lon, alt=0),
    ]
    plane.upload_mission(items)
    plane.set_current_wp(1)
    plane.set_mode("AUTO")
    return plane.monitor_landing()


def fly_mission(plane, p):
    plane.t0 = time.time()
    result = "incomplete"
    try:
        banner("PHASE 1/6  TAKEOFF  (climb-out to %.0f m)" % p.takeoff_alt)
        plane.set_mode("TAKEOFF")
        plane.wait_climb_to(p.takeoff_alt, "TAKEOFF")

        banner("PHASE 2/6  CRUISE for %.0f s (holds takeoff altitude)" % p.cruise_time)
        plane.fly_mode("CRUISE", p.cruise_time, "CRUISE", hold=None)

        banner("PHASE 3/6  FBWA for %.0f s (wings level, %d%% throttle)"
               % (p.fbwa_time, FBWA_THROTTLE_PCT))
        thr_pwm = int(1000 + 10 * max(0, min(100, FBWA_THROTTLE_PCT)))
        plane.fly_mode("FBWA", p.fbwa_time, "FBWA",
                       hold=dict(roll=RC_NEUTRAL, pitch=RC_NEUTRAL, throttle=thr_pwm, yaw=RC_NEUTRAL))

        banner("PHASE 4/6  FBWB for %.0f s (altitude hold, auto throttle)" % p.fbwb_time)
        plane.fly_mode("FBWB", p.fbwb_time, "FBWB",
                       hold=dict(roll=RC_NEUTRAL, pitch=RC_NEUTRAL, throttle=0, yaw=RC_NEUTRAL))

        banner("PHASE 5/6  LOITER for %.0f s" % p.loiter_time)
        plane.fly_mode("LOITER", p.loiter_time, "LOITER", hold=None)

        result = do_landing(plane)
    except MissionAborted as exc:
        print("\n  !! MISSION ABORTED during %s phase: %s" % (plane.phase, exc))
        print("  The remaining phases are skipped (the aircraft is no longer flying).")
        result = "aborted-in-%s" % plane.phase
    finally:
        plane.print_performance(result)
    return result


# --------------------------------------------------------------------------- #
#  Plotting  (embedded -- same 5-panel figure plot_flight.py draws)
# --------------------------------------------------------------------------- #
PHASE_COLORS = ["tab:blue", "tab:green", "tab:orange", "tab:red",
                "tab:purple", "tab:brown", "tab:gray", "tab:olive"]


def _load_rows(path):
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []


def _column(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(key, "")))
        except (ValueError, TypeError):
            out.append(float("nan"))
    return out


def _phase_segments(rows, t):
    segments, current, start = [], None, None
    for i, r in enumerate(rows):
        ph = r.get("phase", "")
        if ph != current:
            if current is not None:
                segments.append((current, start, t[i]))
            current, start = ph, t[i]
    if current is not None:
        segments.append((current, start, t[-1]))
    return segments


def _shade(ax, segments):
    for i, (_, t0, t1) in enumerate(segments):
        ax.axvspan(t0, t1, color=PHASE_COLORS[i % len(PHASE_COLORS)], alpha=0.07)


def _combined_legend(primary, secondary, **kw):
    lines = primary.get_lines() + secondary.get_lines()
    primary.legend(lines, [ln.get_label() for ln in lines], **kw)


def plot_states(csv_path, show=True):
    """Read the telemetry CSV and draw the 5-panel state figure (and save a PNG).
    Called on EVERY exit -- normal completion, abort, or Ctrl-C."""
    rows = _load_rows(csv_path)
    if not rows:
        print("  no telemetry rows recorded -- nothing to plot.")
        return
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib not installed -- skipping plot (pip install matplotlib).")
        print("  CSV is saved at %s; plot later with plot_flight.py" % csv_path)
        return

    t = _column(rows, "time_s")
    segments = _phase_segments(rows, t)

    fig, axes = plt.subplots(5, 1, figsize=(13, 13.5), sharex=True)
    fig.suptitle("flight telemetry  -  %s" % csv_path, fontsize=13)

    # 1. altitude + speeds
    ax = axes[0]
    _shade(ax, segments)
    ax.plot(t, _column(rows, "alt_agl_m"), color="black", label="alt AGL (m)")
    ax.set_ylabel("altitude (m)")
    ax.grid(True, alpha=0.3)
    axspd = ax.twinx()
    axspd.plot(t, _column(rows, "airspeed"), color="tab:red", label="airspeed (m/s)")
    axspd.plot(t, _column(rows, "groundspeed"), color="tab:orange", ls="--", label="ground speed (m/s)")
    axspd.set_ylabel("speed (m/s)")
    _combined_legend(ax, axspd, loc="upper left", fontsize=8)
    top = ax.get_ylim()[1]
    for ph, t0, t1 in segments:
        ax.text((t0 + t1) / 2.0, top, ph, ha="center", va="bottom", fontsize=7)

    # 2. Euler angles
    ax = axes[1]
    _shade(ax, segments)
    ax.plot(t, _column(rows, "roll_deg"), label="roll")
    ax.plot(t, _column(rows, "pitch_deg"), label="pitch")
    ax.plot(t, _column(rows, "yaw_deg"), label="yaw")
    ax.plot(t, _column(rows, "navroll_deg"), label="roll cmd", ls="--", alpha=0.6)
    ax.plot(t, _column(rows, "navpitch_deg"), label="pitch cmd", ls=":", alpha=0.6)
    ax.set_ylabel("Euler (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # 3. body rates
    ax = axes[2]
    _shade(ax, segments)
    ax.plot(t, _column(rows, "p_dps"), label="p (roll rate)")
    ax.plot(t, _column(rows, "q_dps"), label="q (pitch rate)")
    ax.plot(t, _column(rows, "r_dps"), label="r (yaw rate)")
    ax.set_ylabel("body rates (deg/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # 4. flight-path angle + climb + throttle
    ax = axes[3]
    _shade(ax, segments)
    ax.plot(t, _column(rows, "gamma_deg"), color="tab:purple", label="flight-path angle (deg)")
    ax.plot(t, _column(rows, "climb_mps"), color="tab:green", ls="--", label="climb (m/s)")
    ax.set_ylabel("gamma (deg) / climb (m/s)")
    ax.grid(True, alpha=0.3)
    axthr = ax.twinx()
    axthr.plot(t, _column(rows, "throttle_pct"), color="tab:gray", alpha=0.6, label="throttle (%)")
    axthr.set_ylabel("throttle (%)")
    axthr.set_ylim(0, 100)
    _combined_legend(ax, axthr, loc="upper left", fontsize=8)

    # 5. angle of attack + sideslip
    ax = axes[4]
    _shade(ax, segments)
    ax.plot(t, _column(rows, "aoa_deg"), color="tab:red", label="angle of attack alpha (deg)")
    ax.plot(t, _column(rows, "ssa_deg"), color="tab:blue", label="sideslip beta (deg)")
    ax.axhline(0, color="k", lw=0.6, alpha=0.4)
    ax.set_ylabel("alpha / beta (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_xlabel("time (s)")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    png_path = csv_path.rsplit(".", 1)[0] + ".png"
    try:
        fig.savefig(png_path, dpi=120)
        print("  saved plot to %s" % png_path)
    except OSError as exc:
        print("  could not save plot: %s" % exc)
    if show:
        print("  showing plot window (close it to finish)...")
        plt.show()


# --------------------------------------------------------------------------- #
#  Entry point
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description="Minimal-input full ArduPlane SITL mission for the already-tuned uSTOL FDM "
                    "(no param reload; plots states on exit).")
    ap.add_argument("--defaults", action="store_true",
                    help="skip the prompts and fly with the built-in durations")
    ap.add_argument("--log", metavar="CSV", default=None,
                    help="telemetry CSV path (default: flight4_<timestamp>.csv)")
    ap.add_argument("--print-interval", type=float, default=2.0,
                    help="seconds between printed telemetry lines (default: 2.0)")
    ap.add_argument("--no-plot", action="store_true",
                    help="do not open the plot window on exit (the PNG is still saved)")
    args = ap.parse_args()

    p = MissionParams() if args.defaults else prompt_parameters()
    csv_path = args.log or ("flight4_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))

    summarise(p, csv_path)
    if not args.defaults:
        try:
            input("\n  Press Enter to begin the mission (Ctrl-C to abort)... ")
        except (EOFError, KeyboardInterrupt):
            print("\nAborted before connecting.")
            return

    plane = Plane(p.connect, csv_path=csv_path, print_interval=args.print_interval)
    try:
        plane.open()
        # NOTE: params are NOT reloaded -- the tuned config already on the vehicle
        # is used as-is.  The ONLY param we set is the takeoff altitude.
        banner("SETTING TAKEOFF ALTITUDE (only param this script touches)")
        plane.set_param("TKOFF_ALT", p.takeoff_alt)

        banner("ARMING")
        plane.wait_ready_and_position()
        if not plane.arm():
            print("Auto-arm failed -- aborting (check pre-arm messages above).")
            return
        plane.capture_home()
        print("Holding 1 s before takeoff...")
        time.sleep(1)

        fly_mission(plane, p)
        banner("MISSION COMPLETE")

    except KeyboardInterrupt:
        print("\n\n  INTERRUPTED - releasing sticks and switching to RTL.")
        try:
            plane.release_rc()
            plane.set_mode("RTL")
            plane.print_performance("interrupted")
        except Exception as exc:                       # best-effort failsafe
            print("  failsafe error: %s" % exc)
    finally:
        plane.close()
        # Plot the recorded states on EVERY exit (complete / abort / Ctrl-C).
        banner("PLOTTING RECORDED STATES")
        try:
            plot_states(csv_path, show=not args.no_plot)
        except Exception as exc:
            print("  plotting failed: %s" % exc)


if __name__ == "__main__":
    main()
