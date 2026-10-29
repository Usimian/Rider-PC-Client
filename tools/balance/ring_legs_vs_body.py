#!/usr/bin/env python3
"""Ring-down taps, the body against its legs (2026-10-29): did the body flex on the legs, or did the whole robot
pivot about the axle?

    python3 tools/balance/ring_legs_vs_body.py data/captures/<tilt_capture recording>.csv

Balancing off, the wheels held still on the table: the encoders read the wheel turning against the LEGS (the wheel
servos sit on them), so with the wheels not turning over the ground the legs' rate is -(wheel speed / r) (model
frame, + forward lean); the gyro gives the body's. Per tap (gyro over TAP_DPS), over the next RING_S: the rms of
each and of their difference (the spring's flexing rate), and how well the two follow each other (correlation).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "capture"))
from balance_watch import sample_times                                  # noqa: E402

V_COUNTS = 327.6       # speed register counts per m/s (both wheels; the right one reads negative going forward)
R = 0.030
TAP_DPS, RING_S, GAP_S = 40.0, 0.4, 0.8


def main():
    a = np.genfromtxt(sys.argv[1], delimiter=",", names=True, dtype=None, encoding=None)
    for w in np.unique(a["window"]):
        r = a[a["window"] == w]
        if len(r) < 300 or np.any(np.diff(r["i"]) != 1):
            continue
        t = sample_times(r["i"], r["t_us"])
        body = -np.radians(r["rate_x10"] / 10.0)                      # model frame: + forward lean
        legs = -((r["v"] - r["vR"]) / 2.0 / V_COUNTS) / R              # wheels still: the legs turn the other way
        last = -1e9
        for k in np.nonzero(np.abs(body) > np.radians(TAP_DPS))[0]:
            if t[k] - last < GAP_S:
                continue
            last = t[k]
            m = (t >= t[k]) & (t < t[k] + RING_S)
            b, l = np.degrees(body[m]), np.degrees(legs[m])
            c = np.corrcoef(b, l)[0, 1] if l.std() > 0 else float("nan")
            print(f"window {int(w) + 1:2d} at {t[k]:.2f} s: body {np.sqrt(np.mean(b ** 2)):5.1f} deg/s rms, legs "
                  f"{np.sqrt(np.mean(l ** 2)):5.1f}, body - legs (the flexing) {np.sqrt(np.mean((b - l) ** 2)):5.1f}; "
                  f"they follow each other r = {c:+.2f}")


if __name__ == "__main__":
    main()
