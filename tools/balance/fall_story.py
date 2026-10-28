#!/usr/bin/env python3
"""What happened before a fall, quarter-second by quarter-second (2026-10-28: LQR2 fell at full speed).

    python3 tools/balance/fall_story.py data/captures/<balance_watch file>.csv 26,27,48,49,50

For each window given (1-based, as balance_watch.py prints them), in 0.25 s steps: the wheel speed (both
registers, m/s), the pitch (deg, firmware frame: forward lean is LOWER), the gyro rate, the balance command and
how much of the step it spent at the +/-400 limit -- a command pinned at the limit means the wheels had no more
authority to catch the lean.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "capture"))
from balance_watch import sample_times

V_COUNTS = 327.6


def main():
    a = np.genfromtxt(sys.argv[1], delimiter=",", names=True, dtype=None, encoding=None)
    wins = [int(w) - 1 for w in sys.argv[2].split(",")]
    for w in wins:
        r = a[a["window"] == w]
        if not len(r):
            continue
        t = sample_times(r["i"], r["t_us"])
        v = (r["v"] - r["vR"]) / 2.0 / V_COUNTS
        print(f"window {w + 1} ({r['ctrl'][0]}):")
        for t0 in np.arange(0, t[-1], 0.25):
            m = (t >= t0) & (t < t0 + 0.25)
            if not m.any():
                continue
            u = r["cmd"][m]
            print(f"   {t0:4.2f} s  speed {v[m].mean():+5.2f} m/s  pitch {r['pitch_cdeg'][m].mean() / 100:+6.1f} deg "
                  f"(min {r['pitch_cdeg'][m].min() / 100:+6.1f})  rate {np.abs(r['rate_x10'][m]).max() / 10:5.0f} deg/s  "
                  f"cmd {u.mean():+5.0f} (range {u.min():+4.0f}..{u.max():+4.0f}), at limit {100 * np.mean(np.abs(u) >= 399):3.0f}%")


if __name__ == "__main__":
    main()
