#!/usr/bin/env python3
"""
fbw_joystick_test.py  -  Fly the uSTOL ArduPlane SITL in FBWA / FBWB with a JOYSTICK.

HOW THIS DIFFERS FROM auto_flight.py
------------------------------------
auto_flight.py was a *hands-off autopilot demo*: it OVERRODE the RC channels
(rc_channels_override) and flew a fixed, scripted sequence (takeoff -> climb ->
FBWA cruise with the sticks pinned to centre -> RTL). You never touched the
joystick.

This script is the opposite. It NEVER sends an RC override, so YOUR joystick
(through Mission Planner / QGC / MAVProxy) flies the aircraft at all times. The
script only does three things:

    1. sets the servo reversals the uSTOL FDM needs for stable FBW flight
       (SERVO10/11/12_REVERSED = 1 -- same ones auto_flight.py set),
    2. waits for YOU to ARM (in Mission Planner or with the joystick),
    3. acts as a keyboard mode-switch + live telemetry readout so you can flip
       between FBWA / FBWB / MANUAL / TAKEOFF / RTL while you fly.

So: full manual stick authority the whole time; the script just changes modes
and shows mode / altitude / airspeed.

TOPOLOGY (so the joystick and this script don't fight over one MAVLink port)
----------------------------------------------------------------------------
Start SITL with one EXTRA output dedicated to this script:

    ./Tools/autotest/sim_vehicle.py -v ArduPlane --console --map \
        --out=udp:127.0.0.1:14551

Connect Mission Planner WITH YOUR JOYSTICK the normal way (its own TCP 5760 or a
separate UDP out -- NOT 14551). Then, in another terminal:

    python3 fbw_joystick_test.py --connect=udpin:127.0.0.1:14551

KEYS  (type the letter, then press Enter)
-----------------------------------------
    a = FBWA        b = FBWB        m = MANUAL
    t = TAKEOFF     (auto-climb to --alt; press 'a'/'b' once airborne to fly it)
    r = RTL         (return / land)
    s = status now      h = help      q = quit (leaves the vehicle as-is)

NOTE ON STICK DIRECTION
-----------------------
Because SERVO10/11/12 are reversed (to make the autopilot fly correctly), your
MANUAL stick directions are flipped vs the un-reversed setup. If, in FBWA, the
stick feels backwards (push right -> banks left, etc.), set the INPUT-side
reverse RC1_REVERSED (roll) / RC2_REVERSED (pitch) / RC4_REVERSED (yaw) -- do it
EITHER in params OR with the joystick "Reverse" checkbox, never both. This script
does NOT change RCn_* for you, so you can verify the feel yourself.
"""

import argparse
import select
import sys
import time

from pymavlink import mavutil


def wait_heartbeat(m):
    print("Waiting for heartbeat...")
    m.wait_heartbeat()
    print("  heartbeat from system %u component %u" % (m.target_system, m.target_component))


def request_streams(m, rate_hz=5):
    """Ask the autopilot to stream position/attitude/VFR so we can read telemetry."""
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
        return
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
    """Block until the vehicle reports ARMED (you arm it in MP or with the joystick)."""
    print("Waiting for you to ARM (arm now in Mission Planner or with the joystick)...")
    dots = 0
    while True:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=poll)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("\n  ARM detected.")
            return
        dots = (dots + 1) % 4
        sys.stdout.write("\r  waiting%-3s" % ("." * dots))
        sys.stdout.flush()


HELP = """
------------------------------------------------------------------
  JOYSTICK is flying. Type a key + Enter to change mode:
    a = FBWA      b = FBWB      m = MANUAL
    t = TAKEOFF (auto-climb to target, then press a/b)
    r = RTL       s = status      h = help      q = quit
------------------------------------------------------------------"""


def handle_cmd(m, cmd, alt_target):
    print()  # drop off the live status line
    if cmd == "a":
        set_mode(m, "FBWA")
    elif cmd == "b":
        set_mode(m, "FBWB")
    elif cmd == "m":
        set_mode(m, "MANUAL")
    elif cmd == "t":
        set_param(m, "TKOFF_ALT", alt_target)
        set_mode(m, "TAKEOFF")
        print("  auto-climbing to %.0f m -- press 'a' (FBWA) or 'b' (FBWB) once airborne."
              % alt_target)
    elif cmd == "r":
        set_mode(m, "RTL")
    elif cmd == "s":
        pass  # the next telemetry tick will print
    elif cmd in ("h", "help", "?"):
        print(HELP)
    elif cmd == "q":
        print("Quitting -- vehicle left in its current mode/state.")
    elif cmd == "":
        pass
    else:
        print("  unknown command %r (h for help)" % cmd)


def interactive(m, alt_target):
    rev_modes = {v: k for k, v in (m.mode_mapping() or {}).items()}
    print(HELP)
    cur_mode = "?"
    rel_alt = 0.0
    aspd = 0.0
    gspd = 0.0
    last_status = 0.0
    while True:
        # ---- pump MAVLink for telemetry (short timeout keeps the loop responsive) --
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg is not None:
            t = msg.get_type()
            if t == "HEARTBEAT":
                cur_mode = rev_modes.get(msg.custom_mode, str(msg.custom_mode))
            elif t == "GLOBAL_POSITION_INT":
                rel_alt = msg.relative_alt / 1000.0
            elif t == "VFR_HUD":
                aspd = msg.airspeed
                gspd = msg.groundspeed

        # ---- periodic single-line status -------------------------------------
        now = time.time()
        if now - last_status >= 2.0:
            sys.stdout.write("\r[mode %-8s alt %6.1f m   AS %4.1f  GS %4.1f m/s]   "
                             % (cur_mode, rel_alt, aspd, gspd))
            sys.stdout.flush()
            last_status = now

        # ---- non-blocking keyboard ------------------------------------------
        if select.select([sys.stdin], [], [], 0)[0]:
            cmd = sys.stdin.readline().strip().lower()
            handle_cmd(m, cmd, alt_target)
            if cmd == "q":
                return


def main():
    ap = argparse.ArgumentParser(
        description="Fly uSTOL ArduPlane SITL in FBWA/FBWB with a joystick (no RC override).")
    ap.add_argument("--connect", default="udpin:127.0.0.1:14551",
                    help="MAVLink connection string (default: udpin:127.0.0.1:14551; "
                         "add --out=udp:127.0.0.1:14551 to sim_vehicle.py)")
    ap.add_argument("--alt", type=float, default=80.0,
                    help="auto-TAKEOFF target altitude in metres (default: 80)")
    ap.add_argument("--no-reverse", action="store_true",
                    help="do NOT set SERVO10/11/12_REVERSED (use if they're already set in eeprom)")
    ap.add_argument("--no-wait-arm", action="store_true",
                    help="skip waiting for arm and go straight to the keyboard loop")
    args = ap.parse_args()

    print("Connecting to %s ..." % args.connect)
    m = mavutil.mavlink_connection(args.connect)
    wait_heartbeat(m)
    request_streams(m)

    # Servo reversals the uSTOL FDM needs so the autopilot (FBWA/FBWB/TAKEOFF)
    # stabilises in the right direction. These are GLOBAL output-stage settings
    # (same in every mode) -- identical to what auto_flight.py applied.
    if not args.no_reverse:
        set_param(m, "SERVO11_REVERSED", 1)  # elevator
        set_param(m, "SERVO10_REVERSED", 1)  # aileron
        set_param(m, "SERVO12_REVERSED", 1)  # rudder

    if not args.no_wait_arm:
        wait_until_armed(m)

    interactive(m, args.alt)


if __name__ == "__main__":
    main()
