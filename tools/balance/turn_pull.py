"""The turn's effects on the roll estimate, from the Pi's telemetry (2026-10-29): sign and size.

    python3 tools/balance/turn_pull.py data/riderlog/telem_en1_2026-10-28_29.txt
    (the input: balancing telemetry lines from /home/pi/riderlog.txt, e.g.
     ssh rider "grep -a '^2026-10-2[89].*rider/debug/telem.* en=1 ' /home/pi/riderlog.txt" > that file)

Every stretch where the stick held a turn (yrc, rad/s; the firmware closes it on the gyro) gives the gap between the
accelerometer's roll (racc) and the gyro-fused roll (roll). Two effects are fitted at once:
  * the sideways PULL: driving round a curve the accelerometer feels v * w on top of gravity, so racc tilts by
    atan(s * v * w / g) -- s is the sign the firmware's turn lean needs. v = wv / 271.4 (the LQR's speed units).
  * the gyro TILT: the robot balances pitched ~3.5 deg, so the roll gyro's axis is not level and reads a share of
    the turn rate as roll rate; the fusion integrates it until its accelerometer weight balances it -> the fused
    roll (not racc) is off by a term in w alone, turning in place included.
Steady turns only (each stretch's first STEADY_S dropped while the fusion settles).
"""
import re
import sys

import numpy as np

G = 9.81
SPEED_PER_MPS = 271.4    # wv units per m/s (main.cpp LQR_SPEED_PER_MPS)
YRC_MIN = 0.3            # rad/s, a stick turn held
STEADY_S = 1.5           # s dropped at each stretch's start
MIN_S = 0.5              # s of steady turn a stretch needs


def load(path):
    keys = ("ms", "wv", "yrc", "racc", "roll", "th")
    rows, stamps = [], []
    for line in open(path, errors="replace"):
        d = dict(re.findall(r"(\w+)=(-?[\d.]+)", line))
        if all(k in d for k in keys):
            rows.append([float(d[k]) for k in keys])
            stamps.append(line[:19])
    return np.array(rows), stamps


def stretches(a):
    """(start, end) index ranges of consecutive telemetry with the same held turn direction, upright."""
    ms, wv, yrc, racc, roll, th = a.T
    held = (np.abs(yrc) > YRC_MIN) & (np.abs(th) < 15) & (np.abs(roll) < 15) & (np.abs(racc) < 25)
    out, i = [], 0
    while i < len(a):
        if not held[i]:
            i += 1
            continue
        j = i
        while (j + 1 < len(a) and held[j + 1] and np.sign(yrc[j + 1]) == np.sign(yrc[i])
               and 0 < ms[j + 1] - ms[j] < 250):
            j += 1
        out.append((i, j + 1))
        i = j + 1
    return out


def main():
    a, stamps = load(sys.argv[1])
    ms, wv, yrc, racc, roll, th = a.T
    rows = []
    for i, j in stretches(a):
        k = i + np.searchsorted(ms[i:j] - ms[i], STEADY_S * 1000)
        if (ms[j - 1] - ms[min(k, j - 1)]) < MIN_S * 1000:
            continue
        v, w = wv[k:j] / SPEED_PER_MPS, yrc[k:j]
        rows.append((stamps[k], j - k, v.mean(), w.mean(), np.degrees(np.arctan((v * w).mean() / G)),
                     (racc - roll)[k:j].mean()))
    print(f"{len(a)} lines, {len(rows)} steady held turns")
    print("  start                 n     v m/s   turn rad/s   pull deg   racc-roll deg")
    for r in rows:
        print(f"  {r[0]}  {r[1]:4d}   {r[2]:+.2f}     {r[3]:+.2f}       {r[4]:+.2f}      {r[5]:+.2f}")
    R = np.array([r[1:] for r in rows])
    n, v, w, pull, gap = R.T
    A = np.c_[w, pull, np.ones_like(w)]
    sw = np.sqrt(n)
    (cw, cp, c0), *_ = np.linalg.lstsq(A * sw[:, None], gap * sw, rcond=None)
    res = gap - A @ [cw, cp, c0]
    print(f"\nracc - roll = {cw:+.2f} deg per rad/s of turn  {cp:+.2f} x the pull  {c0:+.2f} deg   "
          f"(rms left {np.sqrt(np.average(res ** 2, weights=n)):.2f} deg; the gap was "
          f"{np.sqrt(np.average(gap ** 2, weights=n)):.2f})")
    print(f"  the pull's sign s = {np.sign(cp):+.0f} (as predicted: +1 x pull; the size is the share the fusion "
          f"has not yet drifted to)")
    print(f"  the gyro tilt: {cw:+.2f} deg per rad/s = sin(pitch) x the fusion's time constant -> "
          f"{abs(cw) / np.degrees(np.sin(np.radians(3.55))):.2f} s at 3.55 deg of pitch")


if __name__ == "__main__":
    main()
