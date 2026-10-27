#!/usr/bin/env python3
"""Record RAW ToF frames with the body state (2026-10-27): what the sensor itself reports, before the governor.

One CSV row per rider/tof frame (~10 Hz): time, the latest telemetry (th pitch, roll, wx wheel position, ptgt
position target, vdes, en, legR, legL), then d0..d63 (distance mm) and s0..s63 (VL53L5CX target_status; the
governor and the web page accept only 5 and 6). Row 0 = top/far, column 0 = robot's left, as in rider_tof_safety.

Ctrl-C (or SIGTERM) ends it.
Run:  /usr/bin/python3 tools/tof/tof_raw_capture.py out.csv
The broker comes from gui/rider_config.ini, as for tools/tof/tof_safety_capture.py.
"""
import configparser
import csv
import json
import os
import re
import signal
import sys
import time

import paho.mqtt.client as mqtt

_ini = configparser.ConfigParser()
_ini.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "gui", "rider_config.ini"))
BROKER = _ini.get("mqtt", "broker_host", fallback="localhost")
OUT = sys.argv[1] if len(sys.argv) > 1 else "tof_raw.csv"
TEL = ("th", "roll", "wx", "ptgt", "vdes", "en", "legR", "legL")

f = open(OUT, "w", newline="", buffering=1)
w = csv.writer(f)
w.writerow(["t", *TEL, *(f"d{k}" for k in range(64)), *(f"s{k}" for k in range(64))])
tel = {}
n = 0
NUM = re.compile(r"(\w+)=(-?\d+(?:\.\d+)?)")


def on_connect(c, u, fl, rc, p=None):
    c.subscribe([("rider/tof", 0), ("rider/debug/telem", 0)])
    print(f"recording raw frames to {OUT} (Ctrl-C to stop) ...", flush=True)


def on_message(c, u, m):
    global n
    if m.topic == "rider/debug/telem":
        for k, v in NUM.findall(m.payload.decode(errors="replace")):
            if k in TEL:
                tel[k] = v
        return
    try:
        p = json.loads(m.payload)
        d, s = p["d"], p["s"]
    except (ValueError, KeyError, TypeError):
        return
    w.writerow([f"{time.time():.3f}", *(tel.get(k, "") for k in TEL), *d, *s])
    n += 1


def finish(*_):
    f.close()
    print(f"\n{n} frames", flush=True)
    os._exit(0)


signal.signal(signal.SIGTERM, finish)
signal.signal(signal.SIGINT, finish)
cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="tofrawcap")
cl.on_connect = on_connect
cl.on_message = on_message
cl.connect(BROKER, 1883, 30)
cl.loop_forever()
