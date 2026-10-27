#!/usr/bin/env python3
"""Wall-stop approaches in a tof_safety_capture.py recording (2026-10-27).

An approach = a run of frames with the commanded forward speed vdes > 0.02 that comes within 0.6 m of an obstacle.
Prints, per approach, frame by frame from 0.6 m in: the distance ahead (min_mm), the cap the governor applied
(factor), the commanded speed (vdes, m/s) and the measured wheel speed (wv), then the closest the wall got.
The governor's taper: full speed beyond SLOW_FWD, zero at STOP_FWD (pi/rider_tof_safety.py).

Run:  /usr/bin/python3 tools/tof/wall_stop_report.py data/captures/2026-10-27_wall_stop.csv
"""
import csv
import sys
import time

R = list(csv.DictReader(open(sys.argv[1])))


def num(x, d=0.0):
    try:
        return float(x)
    except (TypeError, ValueError):
        return d


i, n = 0, 0
while i < len(R):
    if num(R[i]["vdes"]) <= 0.02:
        i += 1
        continue
    s = i
    while i < len(R) and num(R[i]["vdes"]) > 0.02:
        i += 1
    e = min(len(R), i + 15)                       # 1.5 s after the commanded speed reaches 0: the roll-on,
    e = next((k for k in range(i, e) if num(R[k]["vdes"]) > 0.02), e)   # cut at the next drive
    near = [k for k in range(s, e) if R[k]["reason"] == "obstacle" and 0 <= num(R[k]["min_mm"], -1) < 600]
    if not near:
        continue
    n += 1
    k0 = near[0]
    print(f"--- approach {n} at {time.strftime('%H:%M:%S', time.localtime(num(R[s]['t'])))} "
          f"(drive {num(R[i - 1]['t']) - num(R[s]['t']):.1f} s)")
    for k in range(k0, e):
        r = R[k]
        print(f"  {num(r['t']) - num(R[k0]['t']):5.2f} s  ahead {r['min_mm']:>5} mm  cap {num(r['factor']):4.2f}  "
              f"vdes {num(r['vdes']):4.2f}  wv {num(r['wv']):6.1f}  th {num(r['th']):5.1f}")
    closest = min(num(R[k]["min_mm"]) for k in near)
    print(f"  closest: {closest:.0f} mm")
