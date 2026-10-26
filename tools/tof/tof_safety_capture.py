#!/usr/bin/env python3
"""Record what the ToF safety governor sees while the robot drives (workstation side, over the Pi broker).

One CSV row per rider/safety/fwd_limit message (~the ToF frame rate), with the drive state at that moment from
rider/debug/telem: time, governor on?, factor applied, factor it WOULD apply, reason, forward distance (mm),
the 64 zone classes (row 0 = far, col 0 = robot's left: o obstacle, f floor, d dropped floor, h overhead,
b below floor, . no return), and the robot's commanded speed (vdes), pitch (th), wheel speed (wv) and enable.

Ctrl-C (or SIGTERM) ends it and prints a summary: how long and how often the governor would have limited, by
reason, split into "driving" (|vdes| > 0.02) and "standing". On open floor any limiting is a false trigger.

Run:  /usr/bin/python3 tools/tof/tof_safety_capture.py out.csv
The broker comes from gui/rider_config.ini, as for tools/capture/drivecap_capture.py.
"""
import configparser
import csv
import json
import os
import re
import signal
import sys
import time
from collections import Counter

import paho.mqtt.client as mqtt

_ini = configparser.ConfigParser()
_ini.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gui", "rider_config.ini"))
BROKER = _ini.get("mqtt", "broker_host", fallback="localhost")
OUT = sys.argv[1] if len(sys.argv) > 1 else "tof_safety.csv"

f = open(OUT, "w", newline="", buffering=1)
w = csv.writer(f)
w.writerow(["t", "on", "factor", "would", "reason", "min_mm", "cls", "vdes", "th", "wv", "en"])
tel = {}
stats = Counter()
NUM = re.compile(r"(\w+)=(-?\d+(?:\.\d+)?)")


def on_connect(c, u, fl, rc, p=None):
    c.subscribe([("rider/safety/fwd_limit", 0), ("rider/debug/telem", 0)])
    print(f"recording to {OUT} (Ctrl-C to stop) ...", flush=True)


def on_message(c, u, m):
    if m.topic == "rider/debug/telem":
        for k, v in NUM.findall(m.payload.decode(errors="replace")):
            if k in ("vdes", "th", "wv", "en"):
                tel[k] = float(v)
        return
    try:
        p = json.loads(m.payload)
    except ValueError:
        return
    vdes = tel.get("vdes", 0.0)
    w.writerow([f"{time.time():.3f}", int(bool(p.get("on"))), p.get("factor"), p.get("would"), p.get("reason"),
                p.get("min_mm"), p.get("cls", ""), vdes, tel.get("th"), tel.get("wv"), int(tel.get("en", 0))])
    state = "driving" if abs(vdes) > 0.02 else "standing"
    stats[state] += 1
    if float(p.get("would", 1.0)) < 0.999:
        stats[(state, p.get("reason"))] += 1


def finish(*_):
    f.close()
    print("\nframes  driving %d  standing %d" % (stats["driving"], stats["standing"]))
    for k, n in sorted(((k, n) for k, n in stats.items() if isinstance(k, tuple)), key=str):
        tot = stats[k[0]] or 1
        print(f"  would limit while {k[0]:8s} for '{k[1]}': {n} frames ({100.0 * n / tot:.1f} %)")
    sys.exit(0)


signal.signal(signal.SIGTERM, finish)
cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="tofsafetycap")
cl.on_connect = on_connect
cl.on_message = on_message
cl.connect(BROKER, 1883, 30)
try:
    cl.loop_forever()
except KeyboardInterrupt:
    finish()
