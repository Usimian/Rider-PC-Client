#!/usr/bin/env python3
"""Ring-down of the body on its legs (2026-10-29): the robot held by its legs, balancing off (wheels free, legs
powered at ride height), the top of the body tapped. Per tap: the ringing's frequency and damping from the gyro.

    python3 tools/balance/ring_down.py data/captures/<tilt_capture recording>.csv

A tap = the gyro (pitch rate) passing TAP_DPS. From 20 ms after it, over RING_S: the zero crossings of the rate
(less its slow part: 30 Hz-wide band around the dominant peak is not assumed -- the rate is high-passed at 3 Hz)
give the frequency; the successive peaks' amplitudes give the decay (log decrement) and so the damping ratio.
With the legs held, the body is an inverted pendulum on the spring: k = I_body w_n^2 + m_body g l_body
(sim/lqr/model_flex.py's split), printed for the measured frequency.
"""
import os
import sys

import numpy as np
from scipy.signal import butter, filtfilt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "capture"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "sim", "lqr"))
from balance_watch import sample_times                                  # noqa: E402

TAP_DPS = 40.0
RING_S = 0.5
GAP_S = 0.8                     # a new tap at least this long after the last


def taps(path):
    a = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    out = []
    for w in np.unique(a["window"]):
        r = a[a["window"] == w]
        if len(r) < 300 or np.any(np.diff(r["i"]) != 1):
            continue
        t = sample_times(r["i"], r["t_us"])
        fs = 1 / np.median(np.diff(t))
        q = r["rate_x10"] / 10.0
        b, aa = butter(2, 3.0 / (fs / 2), btype="high")
        hq = filtfilt(b, aa, q)
        last = -1e9
        for k in np.nonzero(np.abs(q) > TAP_DPS)[0]:
            if t[k] - last < GAP_S:
                continue
            last = t[k]
            m = (t >= t[k] + 0.02) & (t < t[k] + 0.02 + RING_S)
            if m.sum() < 0.8 * RING_S * fs:
                continue
            y, tt = hq[m], t[m]
            zc = np.nonzero(np.diff(np.sign(y)) != 0)[0]
            if len(zc) < 4:
                continue
            f = (len(zc) - 1) / 2 / (tt[zc[-1]] - tt[zc[0]])
            pk = [np.abs(y[zc[i]:zc[i + 1] + 1]).max() for i in range(len(zc) - 1)]
            pk = np.array(pk[: max(2, min(len(pk), 8))])
            dec = np.polyfit(np.arange(len(pk)), np.log(pk), 1)[0] * -2   # per full cycle (two half-cycles)
            zeta = dec / np.sqrt(4 * np.pi ** 2 + dec ** 2)
            out.append(dict(window=int(w) + 1, t=t[k], peak=q[k], f=f, zeta=zeta, n=len(zc) // 2,
                            amp0=pk[0]))
    return out


def main():
    from model_flex import FlexParams
    p = FlexParams()
    rs = taps(sys.argv[1])
    for r in rs:
        print(f"window {r['window']:2d} at {r['t']:.2f} s: tap {r['peak']:+5.0f} deg/s -> rings {r['f']:5.1f} Hz over "
              f"{r['n']} cycles, first swing {r['amp0']:.0f} deg/s, damping ratio {r['zeta']:.3f}")
    if rs:
        f = np.median([r["f"] for r in rs])
        z = np.median([r["zeta"] for r in rs])
        wn = 2 * np.pi * f / np.sqrt(max(1e-9, 1 - z * z))
        k = p.i_body * wn ** 2 + p.m_body * 9.81 * p.l_body
        c = 2 * z * wn * p.i_body
        print(f"\nmedian {f:.1f} Hz, damping ratio {z:.3f} -> with model_flex's body ({p.i_body * 1e4:.1f} kg cm^2, "
              f"{p.m_body * 1000:.0f} g at {p.l_body * 1000:.1f} mm): flex_k {k:.1f} N m/rad, flex_c {c:.4f} N m s/rad")


if __name__ == "__main__":
    main()
