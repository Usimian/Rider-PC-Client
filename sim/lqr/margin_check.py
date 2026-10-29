"""How far from instability is a gain set? Motors stronger than modelled and extra cycles of delay, on the linear
model (2026-10-28: LQR2 shook at ~15 Hz on the floor, pinned at the +/-400 limit, though the model called it safe).

    python3 sim/lqr/margin_check.py

For each controller: the least damped closed-loop mode (damping, Hz) with the motors x1 .. x3 and 0 .. 2 more
cycles of delay. Negative damping = unstable. Today's LQR runs on the floor with a small steady 21 Hz buzz, so
the plant variant that matches the floor should put today's LQR near zero damping at ~21 Hz and LQR2 unstable
near 15 Hz.
All three gain sets ran on the firmware before the single IMU loop: they are checked with its timing
(fw_timing.FW_2026_10_19, 200 Hz), the one they ran under.
"""
import numpy as np

import linear_model as L
from closed_loop import K_TODAY
from fw_timing import FW_2026_10_19 as RAN
from model import Params


def closed_delay(K, Az, Bz, C, d):
    """The loop with the command applied d cycles late (d extra states hold it)."""
    n = Az.shape[0]
    if d == 0:
        return Az + Bz @ K[None, :] @ C
    A = np.zeros((n + d, n + d))
    A[:n, :n] = Az
    A[:n, n + d - 1] = Bz[:, 0]           # the oldest held command reaches the plant
    A[n, :n] = K @ C                      # the newest command enters the delay line
    for j in range(1, d):
        A[n + j, n + j - 1] = 1.0
    return A


def least_damped(Acl):
    ms = [m for m in L.modes(Acl, RAN.mean_period_s) if m[0] > 0.5]
    if max(abs(np.linalg.eigvals(Acl))) >= 1:
        bad = [m for m in ms if m[3] >= 1]
        return (min(m[1] for m in bad), max(bad, key=lambda m: m[3])[0]) if bad else (-1.0, 0.0)
    return min(((m[1], m[0]) for m in ms), default=(1.0, 0.0))


def main():
    p = Params()
    sets = [("today's LQR (raw, 0.85)", L.from_fw(K_TODAY), False, L.A_V_TODAY),
            ("LQR2 as flashed (8 ms)", L.from_fw(dict(x=-1844, vx=-6.89, q=46.0, dq=3.67)), True,
             float(np.exp(-RAN.mean_period_s / 0.008))),
            ("LQR2 design at 31 ms", L.from_fw(dict(x=-1592, vx=-5.92, q=44.5, dq=2.31)), True,
             float(np.exp(-RAN.mean_period_s / 0.031)))]
    for lab, K, corr, a_v in sets:
        print(lab)
        for motor in (1.0, 1.5, 2.0, 3.0):
            cells = []
            for d in (0, 1, 2):
                Az, Bz, C = L.plant(p, a_v=a_v, corrected=corr, motor=motor, timing=RAN)
                z, f = least_damped(closed_delay(K, Az, Bz, C, d))
                cells.append(f"{z:+.2f} at {f:4.1f} Hz")
            print(f"   motors x{motor:<3g}  delay +0: {cells[0]}   +1: {cells[1]}   +2: {cells[2]}")


if __name__ == "__main__":
    main()
