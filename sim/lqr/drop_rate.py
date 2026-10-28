"""Tipping rate of each drop test straight from the gyro, no model (the Hover's board/drop_test_fit.py method).

    python3 sim/lqr/drop_rate.py [capture.csv]

For each fall in a tools/capture/tilt_capture.py capture (fit_drop.falls), fit th'' = lambda^2 sin(th)
over 3-28 deg of lean, th from the gyro integrated from the reading at release, th'' its derivative
smoothed. Also prints the fall itself every 50 ms, to see what the fit sees. The Hover measured
lambda 8.0 /s (two falls, R^2 0.86-0.89).
"""
import sys

import numpy as np
from scipy.signal import savgol_filter

from fit_drop import REC, falls


def main():
    data = falls(sys.argv[1] if len(sys.argv) > 1 else REC)
    for f in data:
        t, th = f["t"], f["th"]
        dt = np.median(np.diff(t))
        n = 11 if len(t) > 11 else (len(t) // 2) * 2 - 1
        thdd = savgol_filter(th, n, 3, deriv=2, delta=dt)
        m = (np.abs(th) > np.radians(3)) & (np.abs(th) < np.radians(28))
        s = np.sin(th[m])
        lam2 = float(np.dot(s, thdd[m]) / np.dot(s, s))
        r2 = 1 - np.sum((thdd[m] - lam2 * s) ** 2) / np.sum((thdd[m] - thdd[m].mean()) ** 2)
        print(f"window {f['window']:2d} {f['dirn']:8s}: release at {np.degrees(th[0]):+.2f} deg, rate "
              f"{np.degrees(f['w0']):+.1f} deg/s; to 30 deg in {f['dur']:.2f} s; lambda "
              f"{np.sqrt(max(lam2, 0)):.1f} /s (R^2 {r2:.2f}, {m.sum()} samples in 3-28 deg)")
        for tk in np.arange(0, t[-1], 0.05):
            i = np.argmin(np.abs(t - tk))
            print(f"      {t[i]:.2f} s  {np.degrees(th[i]):+6.1f} deg  wheel {f['x'][i] * 1000:+5.1f} mm")


if __name__ == "__main__":
    main()
