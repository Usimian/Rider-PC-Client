#!/usr/bin/env python3
"""Raw sensor status vs the governor's verdict, frame by frame (2026-10-27).

Pairs a tof_raw_capture.py recording with a tof_safety_capture.py recording made at the same time. Per raw frame:
the robot's speed (from the wheel position wx), the count of each raw target_status over the 64 zones, and the
count of '.' (rejected) zones in the governor verdict nearest in time. Rejected = status not in the governor's
VALID set, or distance 0. A '.' count above the raw non-VALID count would be the governor losing good data.

Run:  /usr/bin/python3 tools/tof/tof_raw_vs_verdict.py raw.csv verdict.csv
"""
import bisect
import csv
import sys
from collections import Counter

raw = list(csv.DictReader(open(sys.argv[1])))
ver = list(csv.DictReader(open(sys.argv[2])))
vt = [float(r["t"]) for r in ver]

prev = None
for r in raw:
    t = float(r["t"])
    wx = float(r["wx"]) if r["wx"] else None
    v = (wx - prev[1]) / (t - prev[0]) if prev and wx is not None and t > prev[0] else 0.0
    if wx is not None:
        prev = (t, wx)
    st = Counter(r[f"s{k}"] for k in range(64))
    k = min(max(bisect.bisect_left(vt, t), 0), len(ver) - 1)
    dots = ver[k]["cls"].count(".")
    print(f"{t % 100:6.2f}  speed {v:+.2f} m/s  raw status {dict(sorted(st.items()))}  verdict '.' {dots:2d} "
          f"(dt {vt[k] - t:+.2f} s)")
