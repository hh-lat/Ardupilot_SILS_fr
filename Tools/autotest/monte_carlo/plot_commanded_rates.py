#!/usr/bin/env python3
"""
plot_commanded_rates.py [dir]

Reads the ArduPlane dataflash (.BIN) RATE message and plots COMMANDED vs ACTUAL
body angular rates (roll/pitch/yaw). RATE fields: RDes/R, PDes/P, YDes/Y (deg/s).
Default search dir: mc_watch_failure/. Saves commanded_rates.png next to the BIN.
"""
import glob, os, sys
from pymavlink import mavutil
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

argv = sys.argv[1:]
tmax = None
if "--tmax" in argv:
    i = argv.index("--tmax"); tmax = float(argv[i+1]); del argv[i:i+2]
root = argv[0] if argv else "mc_watch_failure"
bins = glob.glob(os.path.join(root, "**", "*.BIN"), recursive=True) + \
       glob.glob(os.path.join(root, "**", "*.bin"), recursive=True)
if not bins:
    sys.exit(f"no .BIN dataflash found under {root}")
f = max(bins, key=os.path.getmtime)
print("reading", f)

# ArduPlane logs the rate-loop target (commanded) vs actual in PIDR/PIDP/PIDY:
#   .Tar = commanded body rate, .Act = achieved body rate (deg/s).
m = mavutil.mavlink_connection(f)
data = {"PIDR": [[], [], []], "PIDP": [[], [], []], "PIDY": [[], [], []]}  # t, Tar, Act
while True:
    msg = m.recv_match(type=["PIDR", "PIDP", "PIDY"], blocking=False)
    if msg is None:
        break
    d = data[msg.get_type()]
    d[0].append(msg.TimeUS / 1e6); d[1].append(msg.Tar); d[2].append(msg.Act)
if not data["PIDP"][0]:
    sys.exit("no PIDR/PIDP/PIDY messages in the log (check LOG_BITMASK)")
t0 = min(d[0][0] for d in data.values() if d[0])
for d in data.values():
    d[0][:] = [x - t0 for x in d[0]]
if tmax is not None:
    for k, d in data.items():
        keep = [i for i, x in enumerate(d[0]) if x <= tmax]
        data[k] = [[d[j][i] for i in keep] for j in range(3)]
print(f"{len(data['PIDP'][0])} samples" + (f"  (t<={tmax}s)" if tmax else ""))

fig, ax = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
for a, key, name in [(ax[0], "PIDR", "Roll"), (ax[1], "PIDP", "Pitch"), (ax[2], "PIDY", "Yaw")]:
    tt, tar, act = data[key]
    a.plot(tt, tar, color="tab:red",  lw=1.2, label=f"{name} rate COMMANDED (.Tar)")
    a.plot(tt, act, color="tab:blue", lw=1.0, label=f"{name} rate actual (.Act)")
    a.set_ylabel(f"{name} rate (deg/s)"); a.grid(alpha=0.3); a.legend(loc="upper right", fontsize=9)
ax[2].set_xlabel("Time (s)")
ax[0].set_title("Commanded vs actual body angular rates (ArduPlane RATE controller)")
fig.tight_layout()
out = os.path.join(os.path.dirname(f), "commanded_rates" + (f"_t{int(tmax)}" if tmax else "") + ".png")
fig.savefig(out, dpi=140)
print("saved:", out)
