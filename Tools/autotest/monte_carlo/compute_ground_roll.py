#!/usr/bin/env python3
"""
compute_ground_roll.py <sim_output.csv>

Computes uSTOL ground roll from a LAT FDM sim_output CSV by integrating ground
speed over the takeoff roll window (plane_moving_state: >=1 rolling -> ==3 IN_AIR
i.e. main gear off). pos_ned/lat-lon in the FDM log are unreliable, so velocity
integration is used. Exit code 2 if the aircraft has not lifted off yet (useful
for polling a live, still-being-written CSV).
"""
import csv, math, sys

def main():
    try:
        rows = list(csv.DictReader(open(sys.argv[1])))
    except Exception:
        sys.exit(2)
    if len(rows) < 5:
        sys.exit(2)

    def S(r):  return int(r["plane_moving_state"])
    def gs(r): return math.hypot(float(r["V_ned_gnd_0"]), float(r["V_ned_gnd_1"]))

    try:
        if not any(S(r) == 3 for r in rows):   # not airborne yet
            sys.exit(2)
        i0 = next(i for i, r in enumerate(rows) if S(r) >= 1)         # start of roll
        iL = next(i for i, r in enumerate(rows) if i > i0 and S(r) == 3)  # main gear off
    except (ValueError, KeyError, StopIteration):
        sys.exit(2)

    def integ(i, j):
        s = 0.0
        for k in range(i + 1, j + 1):
            dt = float(rows[k]["Time_s"]) - float(rows[k-1]["Time_s"])
            s += 0.5 * (gs(rows[k]) + gs(rows[k-1])) * dt
        return s

    iR = next((i for i in range(i0, iL + 1) if float(rows[i]["FLG_NR"]) <= 1), i0)
    gr = integ(i0, iL)
    vlof = float(rows[iL]["TAS_mps"])
    tlof = float(rows[iL]["Time_s"]) - float(rows[i0]["Time_s"])
    print(f"GROUND ROLL = {gr:.1f} m  |  Vlof = {vlof:.1f} m/s  |  "
          f"rotation @ {integ(i0, iR):.1f} m  |  time-to-liftoff = {tlof:.2f} s (sim)")

if __name__ == "__main__":
    main()
