#!/usr/bin/env python3
"""The tap test: does the body ring on its legs? (2026-10-28)

    python3 tools/balance/tap_ring.py data/captures/2026-10-28_tap_test.csv

The capture: tools/capture/balance_watch.py windows (every control cycle, ~250 Hz with balance off), the robot
upright with balance off and the legs powered at ride height, the wheels held against the floor by hand, the
top of the body tapped sharply. A joint between the body (the IMU) and the wheels that is a spring (servo-held
legs) rings after each tap at its own frequency, dying away at its damping.

For each tap (the gyro rate jumping past TAP_DPS): the 0.4 s after its peak, band-passed HF_LO-60 Hz (a hand-held
robot also rocks as a whole at ~3 Hz, which would swamp the ring), its strongest frequency (FFT, zero padded) and a
damped sine fitted to it: frequency, damping ratio, decay time. Then the median over the taps that fit.
"""
import sys

import os

import numpy as np
from scipy.optimize import curve_fit
from scipy.signal import butter, filtfilt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "capture"))
from balance_watch import sample_times

TAP_DPS = 30.0
AFTER_S = 0.4
HF_LO = 8.0


def ring(t, a, zeta, f, ph, c):
    w = 2 * np.pi * f
    return a * np.exp(-zeta * w * t) * np.cos(w * np.sqrt(max(1 - zeta ** 2, 1e-6)) * t + ph) + c


def main():
    a = np.genfromtxt(sys.argv[1], delimiter=",", names=True, dtype=None, encoding=None)
    res = []
    for w in np.unique(a["window"]):
        r = a[a["window"] == w]
        if np.mean(r["cmd"] != 0) > 0.1:
            continue                                     # balancing: not a tap window
        t = sample_times(r["i"], r["t_us"])
        rate = r["rate_x10"] / 10.0
        fs = 1 / np.median(np.diff(t))
        b, aa = butter(3, [HF_LO / (fs / 2), 60 / (fs / 2)], btype="band")
        hf = filtfilt(b, aa, rate)
        i = 1
        while i < len(t):
            if abs(rate[i]) > TAP_DPS and abs(rate[i - 1]) <= TAP_DPS:
                pk = i + int(np.argmax(np.abs(rate[i:i + 10])))
                j = np.searchsorted(t, t[pk] + AFTER_S)
                if j >= len(t) - 1:
                    break
                tt, yy = t[pk:j] - t[pk], hf[pk:j]
                X = np.abs(np.fft.rfft((yy - yy.mean()) * np.hanning(len(yy)), 8 * len(yy)))
                fr = np.fft.rfftfreq(8 * len(yy), 1 / fs)
                m = fr > HF_LO
                f0 = fr[m][np.argmax(X[m])]
                try:
                    a0 = yy[int(np.argmax(np.abs(yy[:int(0.05 * fs)])))]
                    p, _ = curve_fit(ring, tt, yy, p0=[a0, 0.1, f0, 0.0, 0.0],
                                     bounds=([-1e4, 0.0, HF_LO, -np.pi, -1e3], [1e4, 0.99, 60.0, np.pi, 1e3]))
                    fit = 1 - np.sum((yy - ring(tt, *p)) ** 2) / np.sum((yy - yy.mean()) ** 2)
                    res.append((int(w) + 1, t[pk], abs(rate[pk]), f0, p[2], p[1], fit))
                    print(f"window {int(w) + 1:2d} at {t[pk]:4.2f} s: tap {rate[pk]:+6.0f} deg/s, above {HF_LO:g} Hz peak "
                          f"{f0:5.1f} Hz; ring fit {p[2]:5.1f} Hz, damping {p[1]:.2f}, decay "
                          f"{1 / (p[1] * 2 * np.pi * p[2]) * 1000 if p[1] > 0 else np.inf:5.0f} ms (fit R^2 {fit:.2f})")
                except RuntimeError:
                    print(f"window {int(w) + 1:2d} at {t[pk]:4.2f} s: spectrum peak {f0:5.1f} Hz, no ring fit")
                i = j
            i += 1
    good = [x for x in res if x[6] > 0.5]
    if good:
        f = np.median([x[4] for x in good]); z = np.median([x[5] for x in good])
        print(f"\n{len(good)} of {len(res)} taps fit well (R^2 > 0.5): ring {f:.1f} Hz "
              f"({min(x[4] for x in good):.1f}-{max(x[4] for x in good):.1f}), damping {z:.2f} "
              f"({min(x[5] for x in good):.2f}-{max(x[5] for x in good):.2f})")


if __name__ == "__main__":
    main()
