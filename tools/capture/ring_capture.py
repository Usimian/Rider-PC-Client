#!/usr/bin/env python3
"""Record a whole tap session in one piece: the firmware's ring-down capture (main.cpp 'ringcap' / 'ringdump',
2026-10-29), every control cycle for ~16 s, balancing or not -- no download gaps in the middle (the drive capture
holds 2.4 s, then spends ~5 s downloading it over the 115200-baud Pi link, and misses what happens meanwhile).

    /usr/bin/python3 tools/capture/ring_capture.py data/captures/<out>.csv

It arms the capture, counts the seconds down while it records, then fetches it (~12 s) and saves it with a
'window' column (0) in the columns tools/balance/ring_down.py reads. The broker comes from gui/rider_config.ini.
"""
import configparser
import json
import os
import sys
import time

import paho.mqtt.client as mqtt

_ini = configparser.ConfigParser()
_ini.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gui", "rider_config.ini"))
BROKER = _ini.get("mqtt", "broker_host", fallback="localhost")
OUT = sys.argv[1] if len(sys.argv) > 1 else "ring.csv"
SECS = 16.0                           # main.cpp RCAP_N 4000 cycles at ~250 Hz with balancing off
COLS = ["i", "t_us", "pitch_cdeg", "rate_x10", "v", "vR"]
st = {"rows": [], "done": False, "n": None}


def on_connect(c, u, fl, rc, p=None):
    c.subscribe("rider/debug/dcap")


def on_message(c, u, m):
    pl = m.payload.decode(errors="replace").strip()
    if pl.startswith("# dcapr done"):
        st["done"] = True
    elif pl.startswith("# dcapr n="):
        st["n"] = int(pl.split("n=")[1].split()[0])
    elif pl.startswith("# dcapr "):
        p = pl.split()
        if len(p) == 2 + len(COLS):
            st["rows"].append(p[2:])


def line(c, s):
    c.publish("rider/control/line", json.dumps({"line": s}), qos=1)


cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ring_capture")
cl.on_connect = on_connect
cl.on_message = on_message
cl.connect(BROKER, 1883, 30)
cl.loop_start()
time.sleep(0.5)
line(cl, "ringcap")
print(f"RECORDING for {SECS:.0f} s -- tap now", flush=True)
t0 = time.time()
while time.time() - t0 < SECS + 0.3:
    left = SECS + 0.3 - (time.time() - t0)
    print(f"  {left:4.0f} s left", flush=True)
    time.sleep(min(2.0, max(0.1, left)))
line(cl, "ringdump")
print("recording done -- fetching it (~12 s)", flush=True)
t0 = time.time()
while not st["done"] and time.time() - t0 < 60:
    time.sleep(0.2)
cl.loop_stop()
with open(OUT, "w") as f:
    f.write("window," + ",".join(COLS) + "\n")
    for r in st["rows"]:
        f.write("0," + ",".join(r) + "\n")
n = len(st["rows"])
if n:
    t = [int(r[1]) for r in st["rows"]]
    print(f"{n} samples over {(t[-1] - t[0]) * 1e-6:.1f} s -> {OUT}" +
          ("" if st["done"] and n == st["n"] else f"  (INCOMPLETE: {n} of {st['n']}, done={st['done']})"), flush=True)
else:
    print("nothing received -- is the firmware with 'ringcap' flashed?", flush=True)
