#!/usr/bin/env python3
"""Capture the Rider's raw debug telemetry to CSV for drive diagnostics.

Subscribes to rider/debug/telem on the Pi broker, parses key=value fields, and
writes timestamped rows. Run on the workstation; Ctrl-C / SIGTERM to stop.
"""
import sys, time, re, signal
import paho.mqtt.client as mqtt

BROKER = sys.argv[1] if len(sys.argv) > 1 else "10.0.0.95"
OUT = sys.argv[2] if len(sys.argv) > 2 else "drive_capture.csv"
FIELDS = ["th", "rate", "wx", "ptgt", "tgteff", "vdes", "wv", "u", "en", "yaw", "px", "py"]

run = True
def stop(*_a):
    global run; run = False
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)

f = open(OUT, "w", buffering=1)
f.write("t," + ",".join(FIELDS) + "\n")
t0 = time.time()
n = [0]

def on_c(c, u, fl, rc, p=None):
    c.subscribe("rider/debug/telem")

def on_m(c, u, m):
    line = m.payload.decode(errors="replace")
    d = dict(re.findall(r"(\w+)=(-?[\d.]+)", line))
    row = [("%.3f" % (time.time() - t0))] + [d.get(k, "") for k in FIELDS]
    f.write(",".join(row) + "\n")
    n[0] += 1

cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
cl.on_connect = on_c
cl.on_message = on_m
cl.connect(BROKER, 1883, 30)
cl.loop_start()
print("capturing -> %s  (rows print every 2s)" % OUT, flush=True)
last = 0
while run:
    time.sleep(0.5)
    if time.time() - last > 2:
        last = time.time()
        print("  rows=%d" % n[0], flush=True)
cl.loop_stop()
f.close()
print("done, %d rows -> %s" % (n[0], OUT), flush=True)
