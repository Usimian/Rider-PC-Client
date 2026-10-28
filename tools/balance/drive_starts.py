#!/usr/bin/env python3
"""Every drive start and reversal in a balance_watch.py recording, up close (2026-10-28: Marc feels a small twitch right
before or during the lean when driving forward and back at 0.3-0.45 m/s).

    python3 tools/balance/drive_starts.py data/captures/<file>.csv [--detail N]

A start = the stick leaving zero or changing sign (recordings with stick_mms, firmware 2026-10-28 on; then also the
stick and the plan's lean per start); in older ones the robot's speed (both wheel registers) leaving +/-0.03 m/s or
changing sign -- which also counts the wheels' small roll the other way just before a start as a start of its own.
Per start: the command's zero crossings in -0.3..+0.3 s (each one flips the LQR's friction compensation between
-32.5 and +32.5 units), the largest command jump between two cycles, the body's peak swing (gyro), and the gyro's
fast part (above 8 Hz, rms and peak) in the same span against the standing level -- a twitch shows up there. --detail N prints start N every 20 ms.
"""
import os
import sys

import numpy as np
from scipy.signal import butter, filtfilt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "capture"))
from balance_watch import sample_times

V_COUNTS = 327.6


def main():
    a = np.genfromtxt(sys.argv[1], delimiter=",", names=True, dtype=None, encoding=None)
    detail = int(sys.argv[sys.argv.index("--detail") + 1]) if "--detail" in sys.argv else None
    has_plan = "stick_mms" in a.dtype.names             # firmware 2026-10-28 on: the stick and the LQR plan recorded
    n = 0
    quiet = []
    for w in np.unique(a["window"]):
        r = a[a["window"] == w]
        if np.mean(r["cmd"] != 0) < 0.5 or len(r) < 300 or np.any(np.diff(r["i"]) != 1):
            continue                                    # not balancing, or an incomplete window
        t = sample_times(r["i"], r["t_us"])
        fs = 1 / np.median(np.diff(t))
        v = (r["v"] - r["vR"]) / 2.0 / V_COUNTS
        b, aa = butter(2, 2.0 / (fs / 2))
        vs = filtfilt(b, aa, v)
        u = r["cmd"].astype(float)
        rate = r["rate_x10"] / 10.0
        bh, ah = butter(2, 8.0 / (fs / 2), btype="high")
        hf = filtfilt(bh, ah, rate)
        if np.abs(vs).max() < 0.03:
            quiet.append(hf.std())
            continue
        if has_plan:                                    # the stick itself: leaving zero, or changing sign
            go = np.sign(r["stick_mms"])
        else:                                           # the wheels (their small roll the other way before a start
            go = np.where(np.abs(vs) > 0.03, np.sign(vs), 0)   # counts as one more start)
        for i in range(1, len(t)):
            start = go[i] != 0 and go[i - 1] == 0
            flip = go[i] != 0 and go[i - 1] != 0 and go[i] != go[i - 1]
            if not (start or flip) or t[i] < 0.3 or t[i] > t[-1] - 0.3:
                continue
            n += 1
            m = (t > t[i] - 0.3) & (t < t[i] + 0.3)
            s = np.sign(u[m][u[m] != 0])
            crossings = np.nonzero(np.diff(s))[0]
            jump = np.abs(np.diff(u[m])).max()
            kind = ("reversal " if flip else "start ") + ("forward" if go[i] > 0 else "backward")
            asked = ""
            if has_plan:
                after = (t >= t[i]) & (t < t[i] + 0.3)          # the stick's furthest in the start's direction
                asked = (f"stick to {go[i] * np.max(go[i] * r['stick_mms'][after]) / 1000:+.2f} m/s, planned lean "
                         f"up to {np.abs(r['plan_lean_x100'][m]).max() / 100:4.1f} deg; ")
            print(f"{n:2d}. window {int(w) + 1:2d} at {t[i]:4.2f} s, {kind:17s}: {asked}command crosses zero "
                  f"{len(crossings)}x (at {', '.join(f'{(t[m][c] - t[i]) * 1000:+.0f}' for c in crossings[:4])} ms); "
                  f"largest step {jump:4.0f}; body swing peak {np.abs(rate[m]).max():4.0f} deg/s; "
                  f"gyro above 8 Hz rms {hf[m].std():4.1f}, peak {np.abs(hf[m]).max():4.1f} deg/s")
            if detail == n:
                for k in np.nonzero(m)[0][::5]:
                    print(f"      {(t[k] - t[i]) * 1000:+5.0f} ms  speed {vs[k]:+5.2f}  pitch {r['pitch_cdeg'][k] / 100:+6.2f}"
                          f"  rate {rate[k]:+6.1f}  fast {hf[k]:+5.1f}  cmd {u[k]:+5.0f}"
                          + (f"  stick {r['stick_mms'][k] / 1000:+5.2f}  plan acc {r['plan_acc_x1000'][k] / 1000:+5.2f}"
                             f"  plan lean {r['plan_lean_x100'][k] / 100:+6.2f}" if has_plan else ""))
    if quiet:
        print(f"\nstanding windows: gyro above 8 Hz rms {np.median(quiet):.1f} deg/s (median of {len(quiet)})")


if __name__ == "__main__":
    main()
