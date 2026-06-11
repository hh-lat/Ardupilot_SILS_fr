#!/usr/bin/env python3
"""
auto_flight3.py  -  auto_flight2 + a baked-in CUSTOM CONFIG block.

Same full mission profile, telemetry and STOL performance metrics as auto_flight2,
PLUS on every run it first restores the known-good aircraft configuration -- so you
can keep the autopilot reset to firmware defaults between sessions:

  * custom control gains (roll/pitch rate PID-FF, attitude TCONST/RMAX, limits),
  * the correct servo output map + reversals and RC input map (RCMAP, FLTMODE_CH).

You never have to re-enter mapping / reversals / gains by hand.  It also asks for
the desired climb rate and climb angle.  You are asked for the mission parameters
at the start; the script then flies, in order:

    PHASE 1  TAKEOFF   ground roll, rotation, auto-climb to the climb-out altitude
    PHASE 2  CLIMB     GUIDED climb to the cruise altitude
    PHASE 3  CRUISE    CRUISE mode (auto heading + altitude + airspeed hold)
    PHASE 4  FBWA      Fly-By-Wire-A, wings level, manual (held) throttle
    PHASE 5  FBWB      Fly-By-Wire-B, altitude hold + auto throttle (TECS)
    PHASE 6  LOITER    automatic circle at the loiter radius
    PHASE 7  LAND      AUTO autoland with flare (approach + LAND), or RTL/QRTL/QLAND

Throughout the flight it prints, and (with --log) records to CSV at ~5 Hz:
    body rates (p,q,r), Euler angles (roll,pitch,yaw), flight-path angle (gamma),
    NED velocities (vN,vE,vD), climb rate, airspeed, ground speed, throttle.

At the end it prints a PERFORMANCE SUMMARY:
    ground-roll distance, takeoff distance (to a 15 m screen height), lift-off
    speed; touchdown speed, landing ground-roll, landing distance (from 15 m).

It is a CLIENT: start the simulator first, e.g.

    ./Tools/autotest/sim_vehicle.py -v ArduPlane --console --map

then in another terminal:

    ./Tools/autotest/auto_flight3.py                 # interactive
    ./Tools/autotest/auto_flight3.py --log flight.csv # also record CSV
    ./Tools/autotest/auto_flight3.py --defaults       # all defaults, no prompts

By default it connects to SITL's spare MAVLink TCP port 5762 (SERIAL1) so it does
NOT fight MAVProxy on 5760.
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
LIFTOFF_ALT_M = 0.8            # alt (AGL) above which we consider the wheels off
TOUCHDOWN_ALT_M = 1.0          # alt (AGL) below which we consider it touched down
GROUND_STOP_SPEED = 1.0        # m/s ground speed below which we consider it stopped
LANDED_DISARM_ALT = 5.0        # disarm below this alt == real landing; above == anomaly
AIL_CH, ELEV_CH, RUD_CH = 10, 11, 12   # SERVO_OUTPUT_RAW output channels (this airframe)
LAND_METHODS = ("AUTO", "RTL", "QRTL", "QLAND")


class MissionAborted(Exception):
    """Raised when the aircraft crashes/disarms in flight, to stop the mission early."""


# --------------------------------------------------------------------------- #
#  CUSTOM AIRCRAFT CONFIG  (restored on every run; edit here after a re-tune)
#
#  Captured from the current working config (mav.parm).  Keep the autopilot reset
#  to firmware defaults between sessions -- running this script restores all of it.
# --------------------------------------------------------------------------- #
CUSTOM_GAINS = {
    # --- roll rate loop ---
    "RLL_RATE_P": 0.08, "RLL_RATE_I": 0.08, "RLL_RATE_D": 0.005, "RLL_RATE_FF": 0.12,
    "RLL_RATE_FLTT": 3.0, "RLL_RATE_FLTD": 12.0, "RLL_RATE_IMAX": 0.666, "RLL_RATE_SMAX": 150,
    # --- pitch rate loop ---
    "PTCH_RATE_P": 0.08, "PTCH_RATE_I": 0.08, "PTCH_RATE_D": 0.005, "PTCH_RATE_FF": 0.12,
    "PTCH_RATE_FLTT": 3.0, "PTCH_RATE_FLTD": 12.0, "PTCH_RATE_IMAX": 0.666, "PTCH_RATE_SMAX": 150,
    # --- attitude (angle -> rate) loop ---
    "RLL2SRV_TCONST": 0.25, "PTCH2SRV_TCONST": 0.25,
    "RLL2SRV_RMAX": 90, "PTCH2SRV_RMAX_UP": 90, "PTCH2SRV_RMAX_DN": 90, "PTCH2SRV_RLL": 1.0,
    # --- limits + speed scaling + envelope ---
    "ROLL_LIMIT_DEG": 65, "LEVEL_ROLL_LIMIT": 5, "PTCH_LIM_MIN_DEG": -20, "SCALING_SPEED": 12,
    "AIRSPEED_MIN": 9, "AIRSPEED_MAX": 22, "TECS_SINK_MIN": 2.0, "TECS_SINK_MAX": 5.0,
    # --- yaw / turn coordination ---
    "KFF_RDDRMIX": 0.5, "YAW2SRV_DAMP": 0.0, "YAW2SRV_RLL": 1.0, "YAW2SRV_INT": 0.0, "YAW2SRV_SLIP": 0.0,
    # --- TECS (pitch / throttle energy loop) -- omitting these reverts pitch
    #     damping to 0 on a defaults reset and the takeoff over-rotates ---
    "TECS_PTCH_DAMP": 0.3, "TECS_INTEG_GAIN": 0.3, "TECS_THR_DAMP": 0.5,
    "TECS_PTCH_FF_V0": 12.0, "TECS_TIME_CONST": 5.0, "TECS_VERT_ACC": 7.0, "TECS_SPDWEIGHT": 1.0,
    # --- takeoff pitch shaping ---
    "TKOFF_GND_PITCH": 5.0, "TKOFF_LVL_ALT": 10, "TKOFF_LVL_PITCH": 15,
    "TKOFF_PLIM_SEC": 2.0, "STAB_PITCH_DOWN": 2.0,
    # --- navigation (L1) ---
    "NAVL1_PERIOD": 15, "NAVL1_DAMPING": 0.75,
}

# servo OUTPUT function per channel (ArduPlane SRV_Channel function ids):
#   70 = throttle, 4 = aileron, 19 = elevator, 21 = rudder, 3 = flap_auto, 2 = flap
SERVO_FUNCTIONS = {1: 70, 2: 70, 3: 70, 4: 70, 5: 70, 6: 70, 7: 70, 8: 70, 9: 70,
                   10: 4, 11: 19, 12: 21, 13: 3, 14: 2}
SERVO_REVERSED = (1, 10, 11, 12)             # channels that must be reversed for this FDM
# control-surface output ranges (min, trim, max) us -- default 1000/1500/2000 gives
# ~25% more throw, enough to worsen the takeoff over-rotation, so restore them:
SERVO_RANGES = {ch: (1100, 1500, 1900) for ch in (10, 11, 12, 13, 14)}
RC_INPUT_MAP = {"RCMAP_ROLL": 1, "RCMAP_PITCH": 2, "RCMAP_THROTTLE": 3, "RCMAP_YAW": 4}
FLTMODE_CHANNEL = 8


# --------------------------------------------------------------------------- #
#  Console / geometry helpers
# --------------------------------------------------------------------------- #
def banner(text):
    line = "=" * 78
    print("\n" + line + "\n  " + text + "\n" + line)


def ask(prompt, default, cast=str, choices=None):
    """Prompt with a default (Enter accepts it). Re-asks on bad input."""
    suffix = "/".join(choices) + ", " if choices else ""
    while True:
        raw = input("  %s [%s%s]: " % (prompt, suffix, default)).strip()
        if raw == "":
            return default
        try:
            value = cast(raw)
        except (ValueError, TypeError):
            print("    not a valid value, try again")
            continue
        if choices and value not in choices:
            print("    choose one of: %s" % ", ".join(choices))
            continue
        return value


def ask_yes_no(prompt, default=True):
    raw = input("  %s [%s]: " % (prompt, "Y/n" if default else "y/N")).strip().lower()
    return default if raw == "" else raw in ("y", "yes", "1", "true")


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
#  Mission parameters
# --------------------------------------------------------------------------- #
@dataclass
class MissionParams:
    connect: str = "tcp:127.0.0.1:5762"
    # The TAKEOFF mode needs a target altitude at which the auto take-off
    # sequence is declared complete; that is what `climbout_alt` is.  Set it
    # equal to cruise_alt to skip the separate GUIDED climb phase.
    climbout_alt: float = 80.0      # m, TAKEOFF mode completes here
    cruise_alt: float = 150.0       # m, mission altitude (GUIDED climb target)
    cruise_airspeed: float = 12.0   # m/s, AIRSPEED_CRUISE (this airframe cruises ~12)
    rotate_speed: float = 9.0       # m/s, TKOFF_ROTATE_SPD (below cruise, above stall)
    climb_rate: float = 5.0         # m/s, TECS_CLMB_MAX (max climb rate)
    climb_angle: float = 12.0       # deg, climb pitch limit (PTCH_LIM_MAX_DEG)
    use_flaps: bool = False         # deploy flaps (FLAP ON = 20deg) for takeoff/climb-out
    flap_percent: int = 100         # flap % when ON (100 = full throw = 20deg on this aircraft)
    cruise_time: float = 30.0       # s in CRUISE mode
    fbwa_time: float = 20.0         # s in FBWA mode
    fbwa_throttle: int = 60         # %, held throttle during FBWA
    thr_min: int = 0                # %, THR_MIN (autopilot minimum throttle)
    thr_max: int = 90               # %, THR_MAX (autopilot maximum throttle)
    fbwb_time: float = 20.0         # s in FBWB mode
    loiter_time: float = 30.0       # s in LOITER mode
    loiter_radius: float = 120.0    # m, WP_LOITER_RAD
    land_method: str = "AUTO"       # AUTO | RTL | QRTL | QLAND
    approach_dist: float = 600.0    # m, distance of approach WP from home (AUTO)
    approach_alt: float = 60.0      # m, altitude at the approach WP (AUTO)
    land_airspeed: float = 0.0      # m/s, TECS_LAND_ARSPD (0 -> 0.85 x cruise)
    flare_alt: float = 3.0          # m, LAND_FLARE_ALT
    flare_sec: float = 2.0          # s, LAND_FLARE_SEC
    land_sink: float = 0.30         # m/s, TECS_LAND_SINK
    auto_arm: bool = True           # arm from the script vs wait for manual arm
    configure_flare: bool = True    # set takeoff-rotation + landing-flare params
    wind_speed: float = 0.0         # m/s, SIM_WIND_SPD (0 = calm)
    wind_dir: float = 0.0           # deg, SIM_WIND_DIR (direction the wind comes FROM)
    wind_turb: float = 0.0          # SIM_WIND_TURB turbulence intensity (0 = steady)

    def resolved_land_airspeed(self):
        return self.land_airspeed if self.land_airspeed > 0 else round(0.85 * self.cruise_airspeed, 1)


def prompt_parameters():
    """Interactively collect the mission parameters."""
    banner("MISSION SETUP  -  press Enter to accept each default")
    p = MissionParams()

    p.connect = ask("MAVLink connection", p.connect)
    print()
    p.climbout_alt = ask(
        "Takeoff climb-out altitude (m)  [TAKEOFF finishes here; = cruise alt to skip climb]",
        p.climbout_alt, float)
    p.rotate_speed = ask("Takeoff rotation speed (m/s)", p.rotate_speed, float)
    p.use_flaps = ask_yes_no("Deploy flaps (20deg) for takeoff/climb-out?", p.use_flaps)
    p.cruise_alt = ask("Cruise altitude (m)", p.cruise_alt, float)
    p.cruise_airspeed = ask("Cruise airspeed (m/s)", p.cruise_airspeed, float)
    p.climb_rate = ask("Climb rate (m/s)", p.climb_rate, float)
    p.climb_angle = ask("Climb angle / pitch limit (deg)", p.climb_angle, float)
    print()
    p.cruise_time = ask("CRUISE phase duration (s)", p.cruise_time, float)
    p.fbwa_time = ask("FBWA phase duration (s)", p.fbwa_time, float)
    p.fbwa_throttle = ask("FBWA held throttle (%)", p.fbwa_throttle, int)
    p.thr_max = ask("Max throttle limit THR_MAX (%)", p.thr_max, int)
    p.thr_min = ask("Min throttle limit THR_MIN (%)", p.thr_min, int)
    p.fbwb_time = ask("FBWB phase duration (s)", p.fbwb_time, float)
    p.loiter_time = ask("LOITER phase duration (s)", p.loiter_time, float)
    p.loiter_radius = ask("Loiter radius (m)", p.loiter_radius, float)
    print()
    p.land_method = ask("Landing method", p.land_method, str.upper, LAND_METHODS)
    if p.land_method == "AUTO":
        p.approach_dist = ask("  approach waypoint distance (m)", p.approach_dist, float)
        p.approach_alt = ask("  approach waypoint altitude (m)", p.approach_alt, float)
        p.land_airspeed = ask("  landing approach airspeed (m/s)",
                              round(0.85 * p.cruise_airspeed, 1), float)
        p.flare_alt = ask("  flare altitude (m AGL)", p.flare_alt, float)
        p.flare_sec = ask("  flare time-to-touchdown (s)", p.flare_sec, float)
        p.land_sink = ask("  desired touchdown sink rate (m/s)", p.land_sink, float)
    print()
    p.configure_flare = ask_yes_no("Configure takeoff-rotation + landing-flare params?",
                                   p.configure_flare)
    p.auto_arm = ask_yes_no("Arm automatically from the script?", p.auto_arm)
    print()
    p.wind_speed = ask("Wind speed (m/s, 0 = calm)", p.wind_speed, float)
    if p.wind_speed > 0:
        p.wind_dir = ask("  wind direction (deg, FROM)", p.wind_dir, float)
        p.wind_turb = ask("  wind turbulence (0 = steady, ~0.1-0.5 typical)", p.wind_turb, float)
    return p


def summarise(p, csv_path):
    banner("MISSION PLAN")
    print("  connect ............ %s" % p.connect)
    print("  climb-out alt ...... %.0f m  (TAKEOFF completion altitude)" % p.climbout_alt)
    print("  cruise alt ......... %.0f m" % p.cruise_alt)
    print("  cruise airspeed .... %.1f m/s   (rotate at %.1f m/s)"
          % (p.cruise_airspeed, p.rotate_speed))
    print("  climb .............. %.1f m/s,  pitch limit %.0f deg" % (p.climb_rate, p.climb_angle))
    print("  flaps .............. %s" % (
        "ON (%d%% = 20deg) for takeoff/climb-out" % p.flap_percent if p.use_flaps else "off"))
    print("  CRUISE/FBWA/FBWB/LOITER  %.0f / %.0f / %.0f / %.0f s"
          % (p.cruise_time, p.fbwa_time, p.fbwb_time, p.loiter_time))
    print("  FBWA throttle ...... %d %%   (THR_MIN/MAX %d / %d %%)"
          % (p.fbwa_throttle, p.thr_min, p.thr_max))
    print("  loiter radius ...... %.0f m" % p.loiter_radius)
    print("  landing ............ %s%s" % (
        p.land_method,
        ("  (approach %.0f m out @ %.0f m, app AS %.1f, flare %.1f m/%.1f s, sink %.2f)"
         % (p.approach_dist, p.approach_alt, p.resolved_land_airspeed(),
            p.flare_alt, p.flare_sec, p.land_sink)) if p.land_method == "AUTO" else ""))
    print("  flare config ....... %s" % ("yes" if p.configure_flare else "no"))
    print("  auto-arm ........... %s" % ("yes" if p.auto_arm else "no (arm manually)"))
    print("  wind ............... %s" % (
        "calm" if p.wind_speed <= 0 else
        "%.1f m/s from %.0f deg, turb %.2f" % (p.wind_speed, p.wind_dir, p.wind_turb)))
    print("  CSV log ............ %s" % (csv_path or "(none)"))


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
    # internal recorded positions
    _ld_screen: tuple = None
    _touchdown_pos: tuple = None


# --------------------------------------------------------------------------- #
#  Plane controller  (pymavlink wrapper with cached telemetry + logging)
# --------------------------------------------------------------------------- #
class Plane:
    def __init__(self, connect, csv_path=None, print_interval=2.0):
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
        if self.csv_path:
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
        # rotate the NED velocity into the body frame (u fwd, v right, w down)
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

    def _track_events(self):
        if self.lat is None or self.alt is None or self.home_lat is None:
            return
        dist_from_home = haversine_m(self.home_lat, self.home_lon, self.lat, self.lon)
        alt = self.alt
        if not self.landing_active:
            # take-off phase metrics (home == standing-start datum)
            if self.perf.ground_roll_m is None and alt >= LIFTOFF_ALT_M:
                self.perf.ground_roll_m = dist_from_home
                self.perf.liftoff_speed = self.airspeed
                self.perf.liftoff_time = self._t()
            if self.perf.takeoff_dist_m is None and alt >= SCREEN_HEIGHT_M:
                self.perf.takeoff_dist_m = dist_from_home
        else:
            # landing-phase metrics
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

    # -- parameters --------------------------------------------------------- #
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

    def get_param(self, name, timeout=3):
        """Read a single parameter value from the vehicle (None on timeout)."""
        self.m.mav.param_request_read_send(
            self.tsys, self.tcomp, name.encode("ascii"), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and msg.param_id.strip("\x00") == name:
                return msg.param_value
        return None

    def flaps_set(self, percent):
        """Deploy/retract flaps via the auto-flap percentage. FLAP_1_SPEED is held
        high (set in apply_setup_params) so the percentage always governs:
        100 = full (20 deg = FLAP ON), 0 = retracted (FLAP OFF)."""
        self.set_param("FLAP_1_PERCNT", percent)

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

    def wait_until_armed(self):
        print("Waiting for you to ARM the aircraft (e.g. in Mission Planner)...")
        while not self.armed:
            self.pump(0.5)

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

    # -- GUIDED climb ------------------------------------------------------- #
    def guided_goto_alt(self, target_alt, timeout=240):
        self.set_mode("GUIDED")
        self.pump(0.5)
        if self.lat is None:
            print("  no position; cannot command GUIDED altitude")
            return False
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        self.m.mav.set_position_target_global_int_send(
            0, self.tsys, self.tcomp,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
            int(self.lat * 1e7), int(self.lon * 1e7), float(target_alt),
            0, 0, 0, 0, 0, 0, 0, 0)
        return self.wait_climb_to(target_alt, "CLIMB", timeout=timeout)

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
        altitude (crash / failsafe), which the previous version wrongly called
        'landed'."""
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
        if land_result in incomplete:
            lines.append("  (landing metrics incomplete -- no normal touchdown-and-stop)")

        banner("PERFORMANCE SUMMARY")
        for line in lines:
            print("  " + line)
        if self.csv_path:
            try:
                with open(self.csv_path + ".summary.txt", "w") as f:
                    f.write("PERFORMANCE SUMMARY\n" + "\n".join(lines) + "\n")
                print("  (summary also saved to %s.summary.txt)" % self.csv_path)
            except OSError:
                pass


# --------------------------------------------------------------------------- #
#  Flight orchestration
# --------------------------------------------------------------------------- #
def check_speed_envelope(plane, p):
    """Read the vehicle's speed envelope and warn if the requested cruise is
    inconsistent with it -- catches the 'tuned at the wrong speed' mistake."""
    banner("SPEED ENVELOPE CHECK")
    scaling = plane.get_param("SCALING_SPEED")
    amin = plane.get_param("AIRSPEED_MIN")
    amax = plane.get_param("AIRSPEED_MAX")

    def show(v):
        return "%.1f" % v if v is not None else "n/a"

    print("  requested cruise airspeed .. %.1f m/s" % p.cruise_airspeed)
    print("  vehicle SCALING_SPEED ...... %s m/s" % show(scaling))
    print("  vehicle AIRSPEED_MIN / MAX . %s / %s m/s" % (show(amin), show(amax)))

    warnings = []
    if amin is not None and p.cruise_airspeed < amin:
        warnings.append("cruise (%.1f) is BELOW AIRSPEED_MIN (%.1f) -- TECS will fight to "
                        "hold >= MIN and stall-prevention may trigger"
                        % (p.cruise_airspeed, amin))
    if amax is not None and p.cruise_airspeed > amax:
        warnings.append("cruise (%.1f) is ABOVE AIRSPEED_MAX (%.1f)"
                        % (p.cruise_airspeed, amax))
    if scaling is not None and scaling > 0 and abs(scaling - p.cruise_airspeed) / scaling > 0.2:
        factor = (scaling / p.cruise_airspeed) ** 2
        warnings.append("SCALING_SPEED (%.1f) is far from cruise (%.1f): the attitude gains are "
                        "gain-scheduled for the wrong speed -- about (%.1f/%.1f)^2 = %.1fx gain "
                        "error at cruise. Re-anchor SCALING_SPEED to cruise and RE-TUNE."
                        % (scaling, p.cruise_airspeed, scaling, p.cruise_airspeed, factor))
    if warnings:
        bar = "  " + "!" * 72
        print(bar)
        for w in warnings:
            print("  WARNING: %s" % w)
        print(bar)
    else:
        print("  envelope is consistent with the requested cruise speed.")


def load_param_file(plane, path):
    """Apply every 'NAME VALUE' / 'NAME,VALUE' line from a .parm file (the reliable
    way to restore a full config). Fire-and-forget with periodic draining. Returns
    the number of parameters sent."""
    sent = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line[0] in "#/":
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 2:
                continue
            name = parts[0].strip().upper()
            try:
                val = float(parts[1])
            except ValueError:
                continue
            plane.m.mav.param_set_send(plane.tsys, plane.tcomp, name.encode("ascii"),
                                       val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            sent += 1
            if sent % 30 == 0:
                plane.pump(0.05)        # drain acks periodically to avoid flooding the link
    plane.pump(1.0)
    return sent


def backup_params(plane, path):
    """Read EVERY parameter from the vehicle and save it to `path`, so the config
    this script is about to overwrite can be put back later with --restore <path>.
    Returns the number of parameters saved."""
    plane.m.mav.param_request_list_send(plane.tsys, plane.tcomp)
    params, count, last = {}, None, time.time()
    while time.time() - last < 6:                       # stop when the stream goes quiet
        msg = plane.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            continue
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode("ascii", "ignore")
        params[name.strip("\x00")] = msg.param_value
        count = msg.param_count
        last = time.time()
        if count and len(params) >= count:
            break
    with open(path, "w") as f:
        for k in sorted(params):
            f.write("%s,%s\n" % (k, params[k]))
    return len(params)


def apply_custom_config(plane, restore_file=None):
    """Restore the known-good aircraft config so the autopilot can be kept at
    firmware defaults between sessions.

    If `restore_file` is given, load the COMPLETE parameter set from it -- this is
    the reliable option, because a hand-curated subset always risks omitting a
    tuned parameter (omitting the takeoff-pitch / TECS-pitch-damping / servo-range
    params is exactly what over-rotated the takeoff once before).  Otherwise apply
    the baked-in CUSTOM_GAINS + servo/RC mapping below (now expanded to cover the
    flight-critical params, but still not the full 1400-param config)."""
    if restore_file:
        banner("RESTORING FULL CONFIG from %s" % restore_file)
        try:
            n = load_param_file(plane, restore_file)
            print("  restored %d parameters" % n)
        except OSError as exc:
            print("  ERROR reading %s: %s -- falling back to baked-in config" % (restore_file, exc))
            restore_file = None
    if restore_file:
        return
    banner("APPLYING CUSTOM CONFIG  (baked-in gains + servo/RC mapping + reversals)")
    # RC input mapping + flight-mode channel
    for name, ch in RC_INPUT_MAP.items():
        plane.set_param(name, ch)
    plane.set_param("FLTMODE_CH", FLTMODE_CHANNEL)
    # servo output functions
    for ch, fn in SERVO_FUNCTIONS.items():
        plane.set_param("SERVO%d_FUNCTION" % ch, fn)
    # servo reversals (explicit 0/1 so it is deterministic from a defaults reset)
    for ch in SERVO_FUNCTIONS:
        plane.set_param("SERVO%d_REVERSED" % ch, 1 if ch in SERVO_REVERSED else 0)
    # control-surface output ranges
    for ch, (mn, tr, mx) in SERVO_RANGES.items():
        plane.set_param("SERVO%d_MIN" % ch, mn)
        plane.set_param("SERVO%d_TRIM" % ch, tr)
        plane.set_param("SERVO%d_MAX" % ch, mx)
    # control gains + tuning + envelope
    for name, val in CUSTOM_GAINS.items():
        plane.set_param(name, val)


def apply_setup_params(plane, p):
    banner("APPLYING PARAMETERS")
    plane.set_param("TKOFF_ALT", p.climbout_alt)
    plane.set_param("AIRSPEED_CRUISE", p.cruise_airspeed)
    plane.set_param("WP_LOITER_RAD", p.loiter_radius)
    plane.set_param("THR_MIN", p.thr_min)
    plane.set_param("THR_MAX", p.thr_max)
    # climb performance: rate -> TECS_CLMB_MAX, angle -> climb pitch limit
    plane.set_param("TECS_CLMB_MAX", p.climb_rate)
    plane.set_param("PTCH_LIM_MAX_DEG", p.climb_angle)

    if p.use_flaps:
        # Hold FLAP_1_SPEED high so the flap percentage always applies; the script
        # then toggles FLAP_1_PERCNT (100 -> deployed, 0 -> retracted) at the events.
        plane.set_param("FLAP_1_SPEED", 100)
        plane.set_param("FLAP_1_PERCNT", 0)     # start retracted; deployed at takeoff

    if p.configure_flare:
        # Takeoff rotation: speed at which the nose rotates up on an auto take-off.
        plane.set_param("TKOFF_ROTATE_SPD", p.rotate_speed)
        # Landing flare: ArduPlane rounds out near the ground for a soft touchdown.
        plane.set_param("LAND_FLARE_ALT", p.flare_alt)
        plane.set_param("LAND_FLARE_SEC", p.flare_sec)
        plane.set_param("TECS_LAND_ARSPD", p.resolved_land_airspeed())
        plane.set_param("TECS_LAND_SINK", p.land_sink)

    # Servo reversals + mapping are restored by apply_custom_config / --restore,
    # so we no longer re-set them here.

    # SITL wind: only set it when the user actually asks for wind, so we never
    # clobber the wind a restored config (e.g. good_flight.parm) already carries.
    if p.wind_speed > 0:
        plane.set_param("SIM_WIND_SPD", p.wind_speed)
        plane.set_param("SIM_WIND_DIR", p.wind_dir)
        plane.set_param("SIM_WIND_TURB", p.wind_turb)


def do_landing(plane, p):
    banner("PHASE 7/7  LAND  (%s)" % p.land_method)
    if not plane.armed:
        print("  aircraft is already DISARMED entering the landing phase -- it crashed")
        print("  or hit a failsafe in an earlier phase. Skipping landing.")
        return "crashed-earlier"
    if p.land_method == "AUTO":
        if plane.home_lat is None:
            print("  no home position; falling back to RTL")
            plane.set_mode("RTL")
            return "rtl"
        # Approach waypoint placed `approach_dist` south of home; LAND at home.
        # For real wind, point the approach leg into wind instead.
        appr_lat, appr_lon = offset_latlon(
            plane.home_lat, plane.home_lon, -p.approach_dist, 0.0)
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        items = [
            dict(frame=frame, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   # seq 0 = home
                 lat=plane.home_lat, lon=plane.home_lon, alt=0),
            dict(frame=frame, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   # seq 1 = approach
                 lat=appr_lat, lon=appr_lon, alt=p.approach_alt),
            dict(frame=frame, cmd=mavutil.mavlink.MAV_CMD_NAV_LAND,       # seq 2 = land
                 lat=plane.home_lat, lon=plane.home_lon, alt=0),
        ]
        plane.upload_mission(items)
        plane.set_current_wp(1)
        plane.set_mode("AUTO")
        return plane.monitor_landing()
    else:
        plane.set_mode(p.land_method)
        if p.land_method == "RTL":
            print("  RTL: returning to home and loitering.")
            print("  (For a fixed-wing touchdown use landing method AUTO, or set "
                  "RTL_AUTOLAND with a DO_LAND_START in the mission.)")
            return "rtl"
        return plane.monitor_landing()


def fly_mission(plane, p):
    plane.t0 = time.time()
    result = "incomplete"
    try:
        banner("PHASE 1/7  TAKEOFF  (rotate %.0f m/s, climb-out %.0f m)"
               % (p.rotate_speed, p.climbout_alt))
        if p.use_flaps:
            print("  flaps -> ON (%d%% = 20deg) for takeoff" % p.flap_percent)
            plane.flaps_set(p.flap_percent)
        plane.set_mode("TAKEOFF")
        plane.wait_climb_to(p.climbout_alt, "TAKEOFF")
        if p.use_flaps:
            print("  flaps -> OFF (takeoff height %.0f m reached)" % p.climbout_alt)
            plane.flaps_set(0)

        banner("PHASE 2/7  CLIMB to %.0f m" % p.cruise_alt)
        if p.cruise_alt > p.climbout_alt + 1:
            plane.guided_goto_alt(p.cruise_alt)
        else:
            print("  cruise altitude <= climb-out altitude; skipping separate climb")

        banner("PHASE 3/7  CRUISE for %.0f s" % p.cruise_time)
        plane.fly_mode("CRUISE", p.cruise_time, "CRUISE", hold=None)

        banner("PHASE 4/7  FBWA for %.0f s (wings level, %d%% throttle)"
               % (p.fbwa_time, p.fbwa_throttle))
        thr_pwm = int(1000 + 10 * max(0, min(100, p.fbwa_throttle)))
        plane.fly_mode("FBWA", p.fbwa_time, "FBWA",
                       hold=dict(roll=RC_NEUTRAL, pitch=RC_NEUTRAL, throttle=thr_pwm, yaw=RC_NEUTRAL))

        banner("PHASE 5/7  FBWB for %.0f s (altitude hold, auto throttle)" % p.fbwb_time)
        plane.fly_mode("FBWB", p.fbwb_time, "FBWB",
                       hold=dict(roll=RC_NEUTRAL, pitch=RC_NEUTRAL, throttle=0, yaw=RC_NEUTRAL))

        banner("PHASE 6/7  LOITER for %.0f s (radius %.0f m)" % (p.loiter_time, p.loiter_radius))
        plane.fly_mode("LOITER", p.loiter_time, "LOITER", hold=None)

        result = do_landing(plane, p)
    except MissionAborted as exc:
        print("\n  !! MISSION ABORTED during %s phase: %s" % (plane.phase, exc))
        print("  The remaining phases are skipped (the aircraft is no longer flying).")
        result = "aborted-in-%s" % plane.phase
    finally:
        # Always report what we measured, even if a phase failed mid-flight.
        plane.print_performance(result)
    return result


# --------------------------------------------------------------------------- #
#  Entry point
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description="Full scripted ArduPlane SITL mission (custom uSTOL FDM).")
    ap.add_argument("--defaults", action="store_true",
                    help="skip the prompts and fly with the built-in defaults")
    ap.add_argument("--log", metavar="CSV", default=None,
                    help="record full telemetry to this CSV file at ~5 Hz")
    ap.add_argument("--print-interval", type=float, default=2.0,
                    help="seconds between printed telemetry lines (default: 2.0)")
    ap.add_argument("--restore", metavar="PARM", default=None,
                    help="restore the COMPLETE config from this .parm file (recommended -- "
                         "e.g. --restore good_flight.parm) instead of the baked-in subset")
    ap.add_argument("--no-backup", action="store_true",
                    help="skip saving the current params to params_backup_<time>.parm first")
    args = ap.parse_args()

    p = MissionParams() if args.defaults else prompt_parameters()

    csv_path = args.log
    if csv_path is None and not args.defaults:
        csv_path = ask("CSV log file (blank = none)", "") or None

    summarise(p, csv_path)
    if not args.defaults:
        try:
            input("\n  Press Enter to begin the mission (Ctrl-C to abort)... ")
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            return

    plane = Plane(p.connect, csv_path=csv_path, print_interval=args.print_interval)
    try:
        plane.open()
        if not args.no_backup:
            backup_path = "params_backup_%s.parm" % time.strftime("%Y%m%d_%H%M%S")
            banner("BACKING UP CURRENT PARAMS -> %s" % backup_path)
            print("  saved %d params  (undo this run later with:  --restore %s)"
                  % (backup_params(plane, backup_path), backup_path))
        apply_custom_config(plane, restore_file=args.restore)   # full file if --restore, else baked-in
        check_speed_envelope(plane, p)
        apply_setup_params(plane, p)

        banner("ARMING")
        plane.wait_ready_and_position()
        if p.auto_arm:
            if not plane.arm():
                print("Auto-arm failed; falling back to manual arm.")
                plane.wait_until_armed()
        else:
            plane.wait_until_armed()
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


if __name__ == "__main__":
    main()
