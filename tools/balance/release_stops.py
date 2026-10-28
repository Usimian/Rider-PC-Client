#!/usr/bin/env python3
"""Every stick release in a balance_watch.py recording (firmware 2026-10-28 on: stick_mms recorded): how far the
robot rolled on and how long it took to stop (2026-10-28: Marc, the policy keeps rolling 20-30 cm after a release).

    python3 tools/balance/release_stops.py data/captures/<file>.csv [more files]

A release = the stick from beyond +/-0.02 m/s to inside it. Per release: the speed then (both wheel registers,
m/s), the distance rolled (the wheels' speed integrated: the furthest past the release point, mm) until the speed
stays under 0.02 m/s or the window ends ("still rolling" -- a lower bound), the time to stop, and the body's lean swing (deg) in the next 1.5 s against
its lean at the release. A release within 0.3 s of a window's end is skipped (nothing to measure).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "capture"))
from balance_watch import sample_times

V_COUNTS = 327.6
STOPPED = 0.02


def releases(path):
    a = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    out = []
    for w in np.unique(a["window"]):
        r = a[a["window"] == w]
        if np.mean(r["cmd"] != 0) < 0.5 or len(r) < 300 or np.any(np.diff(r["i"]) != 1):
            continue
        t = sample_times(r["i"], r["t_us"])
        st = r["stick_mms"] / 1000.0
        v = (r["v"] - r["vR"]) / 2.0 / V_COUNTS
        x = np.cumsum(v * np.gradient(t)) * 1000.0
        p = r["pitch_cdeg"] / 100.0
        for i in np.nonzero((np.abs(st[:-1]) > 0.02) & (np.abs(st[1:]) <= 0.02))[0] + 1:
            if t[-1] - t[i] < 0.3:
                continue
            moving = np.abs(v[i:]) >= STOPPED
            still = np.nonzero(~moving)[0]
            j = next((i + k for k in still if not moving[k:].any() or np.all(~moving[k:k + 25])), None)
            end = j if j is not None else len(t) - 1
            m = (t >= t[i]) & (t <= t[i] + 1.5)
            rolled = np.max(np.sign(v[i]) * (x[i:end + 1] - x[i]))     # the furthest past the release point
            out.append(dict(window=int(w) + 1, ctrl=r["ctrl"][0], t=t[i], stick=st[i - 1], v=v[i], rolled=rolled,
                            stop_s=(t[j] - t[i]) if j is not None else None, seen=t[-1] - t[i],
                            swing=np.abs(p[m] - p[i]).max()))
    return out


def main():
    for path in sys.argv[1:]:
        rs = releases(path)
        print(f"{os.path.basename(path)}: {len(rs)} releases")
        for r in rs:
            stop = f"stopped after {r['stop_s']:.2f} s" if r["stop_s"] is not None else f"still rolling ({r['seen']:.1f} s seen)"
            print(f"   window {r['window']:2d} {r['ctrl']:3s} at {r['t']:.2f} s: stick {r['stick']:+.2f}, speed {r['v']:+.2f} m/s -> rolled "
                  f"{r['rolled']:4.0f} mm, {stop}; lean swing {r['swing']:.1f} deg")


if __name__ == "__main__":
    main()
