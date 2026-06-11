#!/usr/bin/env python3
"""
auto_flight.py  -  Scripted ArduPlane SITL flight (standalone, does NOT touch sim_vehicle.py).

Sequence:
    1. connect + wait for heartbeat
    2. wait for YOU to arm the aircraft (e.g. in Mission Planner)
    3. wait 1 s
    4. TAKEOFF  (auto-climb toward --alt)
    5. wait until altitude >= --alt  (default 100 m)
    6. wait 1 s
    7. switch to FBWA and cruise for --cruise seconds (throttle held via RC override)
    8. LAND  (default mode RTL)

This is a CLIENT: start the simulator first with sim_vehicle.py as usual, e.g.

    ./Tools/autotest/sim_vehicle.py -v ArduPlane --console --map

then in another terminal:

    ./Tools/autotest/auto_flight.py

By default it connects to SITL's spare MAVLink TCP port 5762 (SERIAL1), so it does
NOT fight MAVProxy on 5760. If that port isn't available, add an output to your
sim_vehicle command, e.g.  --out=udp:127.0.0.1:14551 , and run this with
--connect=udp:127.0.0.1:14551 .
"""

import argparse
import sys
import time

from pymavlink import mavutil


def wait_heartbeat(m):
    print("Waiting for heartbeat...")
    m.wait_heartbeat()
    print("  heartbeat from system %u component %u" % (m.target_system, m.target_component))


def request_streams(m, rate_hz=5):
    """Ask the autopilot to stream position/attitude so we can read altitude."""
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1)


def set_param(m, name, value):
    print("Setting param %s = %s" % (name, value))
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    # best-effort confirmation
    deadline = time.time() + 3
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.strip("\x00") == name:
            print("  confirmed %s = %.3f" % (name, msg.param_value))
            return
    print("  (no confirmation for %s, continuing)" % name)


def set_mode(m, mode_name):
    mapping = m.mode_mapping() or {}
    if mode_name not in mapping:
        print("ERROR: mode %r not available. Known modes: %s"
              % (mode_name, ", ".join(sorted(mapping))))
        sys.exit(1)
    mode_id = mapping[mode_name]
    print("Mode -> %s (%u)" % (mode_name, mode_id))
    m.set_mode(mode_id)
    deadline = time.time() + 5
    while time.time() < deadline:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.custom_mode == mode_id:
            print("  mode confirmed: %s" % mode_name)
            return
    print("  WARNING: mode %s not confirmed" % mode_name)


def wait_until_armed(m, poll=1.0):
    """Block until the vehicle reports ARMED (i.e. you armed it in Mission Planner)."""
    print("Waiting for you to ARM the aircraft (arm it now in Mission Planner)...")
    dots = 0
    while True:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=poll)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("\n  ARM detected - starting flight.")
            return
        dots = (dots + 1) % 4
        sys.stdout.write("\r  waiting%-3s" % ("." * dots))
        sys.stdout.flush()


def get_rel_alt(m, timeout=2):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    if msg is not None:
        return msg.relative_alt / 1000.0   # mm -> m
    return None


def wait_altitude(m, target, timeout=180):
    print("Climbing to %.0f m ..." % target)
    deadline = time.time() + timeout
    while time.time() < deadline:
        alt = get_rel_alt(m)
        if alt is not None:
            sys.stdout.write("  alt = %6.1f m\r" % alt)
            sys.stdout.flush()
            if alt >= target:
                print("\n  reached %.1f m" % alt)
                return True
    print("\n  TIMEOUT: did not reach %.0f m" % target)
    return False


def rc_override(m, roll=0, pitch=0, throttle=0, yaw=0):
    """0 = release that channel (autopilot/RC keeps control)."""
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        roll, pitch, throttle, yaw, 0, 0, 0, 0)


def main():
    ap = argparse.ArgumentParser(description="Scripted ArduPlane SITL flight.")
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762",
                    help="MAVLink connection string (default: tcp:127.0.0.1:5762)")
    ap.add_argument("--alt", type=float, default=100.0,
                    help="target takeoff altitude in metres (default: 100)")
    ap.add_argument("--cruise", type=float, default=30.0,
                    help="seconds to cruise in FBWA before landing (default: 30)")
    ap.add_argument("--throttle", type=int, default=60,
                    help="FBWA cruise throttle percent (default: 60)")
    ap.add_argument("--land-mode", default="RTL",
                    help="mode used to land/recover (default: RTL; use QLAND for quadplanes)")
    args = ap.parse_args()

    print("Connecting to %s ..." % args.connect)
    m = mavutil.mavlink_connection(args.connect)
    wait_heartbeat(m)
    request_streams(m)

    # Make TAKEOFF mode aim for the requested altitude.
    set_param(m, "TKOFF_ALT", args.alt)

    # The elevator output is reversed relative to the uSTOL FDM sign convention
    # (FDM: high PWM = elevator down = nose-down). Without this, the autopilot's
    # nose-up takeoff command drives the nose DOWN and it won't rotate. Reversing
    # SERVO11 makes auto-takeoff pitch up correctly.
    set_param(m, "SERVO11_REVERSED", 1)

    # Same sign clash on roll: with the aileron un-reversed the wings-level loop is
    # positive feedback, so it rolls off and crashes just after liftoff. Reverse
    # SERVO10 so roll/wings-level control is stable.
    set_param(m, "SERVO10_REVERSED", 1)

    # And on yaw: an un-reversed rudder drives the wrong way, holding the aircraft
    # in a continuous banked turn (it spirals instead of climbing wings-level).
    # Reverse SERVO12 so heading/yaw control is stable.
    set_param(m, "SERVO12_REVERSED", 1)

    # ---- 2/3. wait for YOU to arm (in Mission Planner), then wait a second --
    wait_until_armed(m)
    print("Holding 1 s before takeoff...")
    time.sleep(1)

    # ---- 4/5. takeoff and climb -------------------------------------------
    set_mode(m, "TAKEOFF")
    if not wait_altitude(m, args.alt):
        print("Continuing anyway.")

    # ---- 6. wait a second --------------------------------------------------
    print("Holding 1 s at altitude...")
    time.sleep(1)

    # ---- 7. FBWA cruise ----------------------------------------------------
    set_mode(m, "FBWA")
    thr_pwm = int(1000 + 10 * max(0, min(100, args.throttle)))
    print("Cruising in FBWA for %.0f s (wings level, throttle %d%%)..."
          % (args.cruise, args.throttle))
    cruise_end = time.time() + args.cruise
    while time.time() < cruise_end:
        # neutral roll/pitch/yaw (1500) + held throttle so it flies straight & level
        rc_override(m, roll=1500, pitch=1500, throttle=thr_pwm, yaw=1500)
        alt = get_rel_alt(m, timeout=1)
        if alt is not None:
            sys.stdout.write("  cruising... alt = %6.1f m\r" % alt)
            sys.stdout.flush()
    print("\n  cruise complete")

    # release the sticks so the landing mode has full control
    rc_override(m, 0, 0, 0, 0)

    # ---- 8. land -----------------------------------------------------------
    set_mode(m, args.land_mode)
    print("Handed over to %s. Script done." % args.land_mode)
    print("(For a true fixed-wing autoland you need an AUTO mission with a "
          "LAND waypoint; RTL will return & loiter unless RTL_AUTOLAND is set.)")


if __name__ == "__main__":
    main()
