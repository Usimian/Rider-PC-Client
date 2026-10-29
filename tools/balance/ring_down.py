#!/usr/bin/env python3
"""Ring-down of the body on its legs (2026-10-29): the robot clamped at its legs (wheels on the table), balancing
off, the legs powered at ride height, the top of the body tapped. Per tap: the flex -- the body's angle against
the legs' -- and its frequency, damping, and the spring they give.

    python3 tools/balance/ring_down.py data/captures/<tilt_capture recording>.csv

The flex = body angle - legs angle, both from the moment of the tap:
  body  the gyro (pitch rate), integrated
  legs  the wheel encoders: they read the wheel turning against the legs (the wheel servos sit on them), so with
        the wheels held on the table the legs' angle is -(wheel travel / r); the recorder logs the servos' speed
        registers (both wheels, 327.6 counts per m/s; one count = 5.8 deg/s of the legs), integrated
A tap = the gyro past TAP_DPS. Over RING_S from 20 ms after it, the flex's zero crossings give the frequency and
its successive swings the decay (log decrement -> damping ratio). The spring for the body on fixed legs, an
inverted pendulum on it: k = I_body w_n^2 + m_body g l_body (sim/lqr/model_flex.py's split).
"""
import os
import sys

import numpy as np
from scipy.signal import butter, filtfilt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "capture"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "sim", "lqr"))
from balance_watch import sample_times                                  # noqa: E402

V_COUNTS, R = 327.6, 0.030
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
        dt = np.gradient(t)
        body_rate = -(r["rate_x10"] / 10.0)                            # deg/s, + forward lean
        legs_rate = -np.degrees(((r["v"] - r["vR"]) / 2.0 / V_COUNTS) / R)
        flex = np.cumsum((body_rate - legs_rate) * dt)                 # deg, body against legs
        legs = np.cumsum(legs_rate * dt)
        b, aa = butter(2, 2.0 / (fs / 2), btype="high")                # the slow drift of the integration out
        hflex = filtfilt(b, aa, flex)
        last = -1e9
        for k in np.nonzero(np.abs(body_rate) > TAP_DPS)[0]:
            if t[k] - last < GAP_S:
                continue
            last = t[k]
            m = (t >= t[k] + 0.02) & (t < t[k] + 0.02 + RING_S)
            if m.sum() < 0.8 * RING_S * fs:
                continue
            y, tt = hflex[m], t[m]
            zc = np.nonzero(np.diff(np.sign(y)) != 0)[0]
            if len(zc) < 4:
                continue
            f = (len(zc) - 1) / 2 / (tt[zc[-1]] - tt[zc[0]])
            pk = np.array([np.abs(y[zc[i]:zc[i + 1] + 1]).max() for i in range(len(zc) - 1)][:8])
            dec = -2 * np.polyfit(np.arange(len(pk)), np.log(pk), 1)[0]   # per full cycle
            zeta = dec / np.sqrt(4 * np.pi ** 2 + dec ** 2)
            legs_moved = np.ptp(legs[m])
            out.append(dict(window=int(w) + 1, t=t[k], f=f, zeta=zeta, cycles=len(zc) // 2, flex0=pk[0],
                            legs=legs_moved))
    return out


def main():
    from model_flex import FlexParams
    p = FlexParams()
    rs = taps(sys.argv[1])
    for r in rs:
        print(f"window {r['window']:2d} at {r['t']:.2f} s: the flex rings {r['f']:4.1f} Hz over {r['cycles']} cycles, "
              f"first swing {r['flex0']:.2f} deg, damping ratio {r['zeta']:+.3f}; the legs moved {r['legs']:.2f} deg")
    if rs:
        f = np.median([r["f"] for r in rs])
        z = np.median([r["zeta"] for r in rs])
        wn = 2 * np.pi * f / np.sqrt(max(1e-9, 1 - z * z))
        k = p.i_body * wn ** 2 + p.m_body * 9.81 * p.l_body
        print(f"\nmedian {f:.1f} Hz, damping ratio {z:.3f} -> with model_flex's body ({p.i_body * 1e4:.1f} kg cm^2, "
              f"{p.m_body * 1000:.0f} g at {p.l_body * 1000:.1f} mm): flex_k {k:.2f} N m/rad, "
              f"flex_c {2 * z * wn * p.i_body:.4f} N m s/rad")


if __name__ == "__main__":
    main()
