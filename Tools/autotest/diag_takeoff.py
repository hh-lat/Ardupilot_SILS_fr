#!/usr/bin/env python3
"""
diag_takeoff.py  -  diagnose why the TAKEOFF climb departs (rolls/yaws) and crashes.

It arms, switches to TAKEOFF, and logs the signals that distinguish the three
causes that all look like "it rolled off and crashed":

  * commanded roll/pitch   (NAV_CONTROLLER_OUTPUT.nav_roll / nav_pitch)
  * actual roll/pitch/yaw  (ATTITUDE)
  * body rates p/q/r       (ATTITUDE)
  * airspeed               (VFR_HUD)        -> stall?
  * actual servo outputs   (SERVO_OUTPUT_RAW servo10/11/12 = ail/elev/rud) -> sign?

At the end it prints a VERDICT (stall/spin vs sign error vs gain instability) and
saves diag_takeoff.csv plus the samples around the departure, which you can paste
back for analysis.

Run with SITL already up (sim_vehicle.py ...):
    python3 ./Tools/autotest/diag_takeoff.py
    python3 ./Tools/autotest/diag_takeoff.py --no-arm     # if you arm manually
"""

import argparse
import csv
import math
import sys
import time

from pymavlink import mavutil

RAD2DEG = 180.0 / math.pi
# servo function -> output channel for this airframe (from mav.parm):
AIL_CH, ELEV_CH, RUD_CH = 10, 11, 12


class Diag:
    def __init__(self, connect):
        self.connect = connect
        self.m = None
        self.tsys = self.tcomp = None
        self.s = dict(t=0.0, alt=None, asp=0.0, gsp=0.0,
                      roll=0.0, pitch=0.0, yaw=0.0, p=0.0, q=0.0, r=0.0,
                      navroll=0.0, navpitch=0.0,
                      ail=0, elev=0, rud=0, thr=0.0, armed=False, mode=None)
        self.rows = []
        self.t0 = None

    def open(self):
        print("Connecting to %s ..." % self.connect)
        self.m = mavutil.mavlink_connection(self.connect)
        self.m.wait_heartbeat()
        self.tsys, self.tcomp = self.m.target_system, self.m.target_component
        print("  heartbeat from system %u" % self.tsys)
        # stream the messages we need at 20 Hz
        for msgid in (30, 33, 36, 62, 74):   # ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, NAV_CTRL_OUT, VFR_HUD
            self.m.mav.command_long_send(
                self.tsys, self.tcomp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msgid, 50000, 0, 0, 0, 0, 0)
        self.m.mav.request_data_stream_send(
            self.tsys, self.tcomp, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

    def pump(self, seconds=0.05):
        end = time.time() + seconds
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                if time.time() >= end:
                    return
                time.sleep(0.002)
                continue
            if msg.get_srcSystem() != self.tsys:
                continue
            self._ingest(msg)

    def _ingest(self, msg):
        t = msg.get_type()
        s = self.s
        if t == "HEARTBEAT":
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            s["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            s["mode"] = msg.custom_mode
        elif t == "ATTITUDE":
            s["roll"], s["pitch"], s["yaw"] = (msg.roll * RAD2DEG, msg.pitch * RAD2DEG, msg.yaw * RAD2DEG)
            s["p"], s["q"], s["r"] = (msg.rollspeed * RAD2DEG, msg.pitchspeed * RAD2DEG, msg.yawspeed * RAD2DEG)
        elif t == "GLOBAL_POSITION_INT":
            s["alt"] = msg.relative_alt / 1000.0
        elif t == "VFR_HUD":
            s["asp"], s["gsp"], s["thr"] = msg.airspeed, msg.groundspeed, msg.throttle
        elif t == "NAV_CONTROLLER_OUTPUT":
            s["navroll"], s["navpitch"] = msg.nav_roll, msg.nav_pitch
        elif t == "SERVO_OUTPUT_RAW":
            s["ail"] = getattr(msg, "servo%d_raw" % AIL_CH, 0)
            s["elev"] = getattr(msg, "servo%d_raw" % ELEV_CH, 0)
            s["rud"] = getattr(msg, "servo%d_raw" % RUD_CH, 0)

    def set_mode(self, name):
        mp = self.m.mode_mapping() or {}
        if name not in mp:
            sys.exit("mode %s unavailable" % name)
        self.m.set_mode(mp[name])

    def arm(self, timeout=30):
        print("Arming...")
        end = time.time() + timeout
        while time.time() < end:
            self.m.mav.command_long_send(
                self.tsys, self.tcomp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
            inner = time.time() + 3
            while time.time() < inner:
                self.pump(0.3)
                if self.s["armed"]:
                    print("  ARMED")
                    return True
        return False

    def record(self):
        s = self.s
        self.rows.append(dict(s, t=time.time() - self.t0))

    def run(self, do_arm, timeout):
        self.t0 = time.time()
        # wait for position
        for _ in range(60):
            self.pump(0.3)
            if self.s["alt"] is not None:
                break
        if do_arm and not self.s["armed"]:
            if not self.arm():
                print("could not arm; arm manually and re-run with --no-arm")
                return
        else:
            print("waiting for you to arm...")
            while not self.s["armed"]:
                self.pump(0.3)
        self.set_mode("TAKEOFF")
        print("TAKEOFF -- logging until it lands/crashes or %ds...\n" % timeout)

        airborne = False
        last_print = 0.0
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.05)
            self.record()
            s = self.s
            alt = s["alt"] or 0.0
            if alt > 8:
                airborne = True
            now = time.time()
            if now - last_print > 0.5:
                print("  t=%5.1f alt=%6.1f AS=%5.1f roll=%6.1f navroll=%6.1f p=%6.1f r=%6.1f ail=%4d rud=%4d"
                      % (now - self.t0, alt, s["asp"], s["roll"], s["navroll"], s["p"], s["r"], s["ail"], s["rud"]))
                last_print = now
            if not s["armed"] and airborne:
                print("\n  DISARMED (crash/land).")
                break
            if airborne and alt < 1.5:
                print("\n  back on the ground.")
                break

    def save(self, path):
        if not self.rows:
            return
        cols = ["t", "alt", "asp", "gsp", "roll", "pitch", "yaw", "p", "q", "r",
                "navroll", "navpitch", "ail", "elev", "rud", "thr", "armed", "mode"]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for r in self.rows:
                w.writerow(["%.3f" % r["t"] if isinstance(r["t"], float) else r["t"]] +
                           [r.get(c) for c in cols[1:]])
        print("saved %s (%d samples)" % (path, len(self.rows)))

    def verdict(self):
        rows = [r for r in self.rows if (r["alt"] or 0) > 8]   # airborne only
        if not rows:
            print("\nVERDICT: never got airborne (>8 m). Likely couldn't rotate / under-power.")
            return
        # find departure onset
        dep = next((r for r in rows if abs(r["roll"]) > 25), None)
        min_as = min(r["asp"] for r in rows)
        max_roll = max(abs(r["roll"]) for r in rows)
        max_r = max(abs(r["r"]) for r in rows)
        max_p = max(abs(r["p"]) for r in rows)
        # read AIRSPEED_MIN for stall reference
        self.m.mav.param_request_read_send(self.tsys, self.tcomp, b"AIRSPEED_MIN", -1)
        amin = None
        d = time.time() + 3
        while time.time() < d:
            mm = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if mm and mm.param_id.strip("\x00") == "AIRSPEED_MIN":
                amin = mm.param_value
                break

        print("\n" + "=" * 70)
        print("VERDICT")
        print("=" * 70)
        print("  airborne samples: min airspeed=%.1f  max|roll|=%.0f  max|p|=%.0f  max|r|=%.0f deg(/s)"
              % (min_as, max_roll, max_p, max_r))
        print("  AIRSPEED_MIN = %s" % ("%.1f" % amin if amin is not None else "n/a"))
        if dep is None:
            print("  No roll departure (|roll| stayed < 25). If it still crashed it is a")
            print("  PITCH/altitude problem, not lateral -- look at pitch & alt in the CSV.")
            return
        as_dep = dep["asp"]
        print("  DEPARTURE at t=%.1f s, alt=%.0f m, airspeed=%.1f, nav_roll(cmd)=%.0f, roll(actual)=%.0f, r=%.0f deg/s"
              % (dep["t"], dep["alt"] or 0, as_dep, dep["navroll"], dep["roll"], dep["r"]))
        print()
        if amin is not None and (as_dep <= amin + 1.5 or min_as <= amin + 1.0):
            print("  => STALL / SPIN.  Airspeed decayed to ~stall before the wing dropped.")
            print("     Fix: shallower climb (PTCH_LIM_MAX_DEG ~8-10), more power, or a")
            print("     higher cruise/scaling speed. It is NOT a tuning/sign fault.")
        elif abs(dep["navroll"]) < 10:
            print("  => INNER-LOOP roll problem.  Controller commanded wings-level")
            print("     (nav_roll ~ 0) but the aircraft rolled to %.0f anyway." % dep["roll"])
            print("     Either the aileron SIGN is wrong (positive feedback) or roll gains")
            print("     are too weak. Check the AIL column vs roll in the samples below:")
            print("     if roll goes +, the aileron output should move to push it back -,")
            print("     not further +. If it adds to the roll -> sign/reversal is wrong.")
        else:
            print("  => OUTER loop commanded the bank (nav_roll=%.0f), e.g. heading/nav" % dep["navroll"])
            print("     fighting at low speed. Less likely a pure instability.")

        # dump samples around departure for manual/sign inspection
        idx = self.rows.index(dep)
        lo, hi = max(0, idx - 4), min(len(self.rows), idx + 6)
        print("\n  samples around departure (paste these back):")
        print("   t     alt   AS   roll  navroll   p     r    ail  elev rud")
        for r in self.rows[lo:hi]:
            print("  %5.1f %5.0f %5.1f %6.1f %7.1f %6.0f %6.0f %5d %5d %5d"
                  % (r["t"], r["alt"] or 0, r["asp"], r["roll"], r["navroll"],
                     r["p"], r["r"], r["ail"], r["elev"], r["rud"]))


def main():
    ap = argparse.ArgumentParser(description="Diagnose a departing TAKEOFF climb.")
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    ap.add_argument("--no-arm", action="store_true", help="wait for manual arm instead of arming")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--csv", default="diag_takeoff.csv")
    args = ap.parse_args()

    d = Diag(args.connect)
    d.open()
    try:
        d.run(do_arm=not args.no_arm, timeout=args.timeout)
    finally:
        d.save(args.csv)
        d.verdict()


if __name__ == "__main__":
    main()
