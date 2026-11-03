"""The motors' strength against the battery voltage, from steady cruising in the Pi's telemetry (2026-11-03: the MPC
drive planner, sim/lqr/mpc_plan.py, needs it on the robot).

    python3 tools/balance/motor_strength.py data/riderlog/<file>.txt
    (the input: LQR driving lines from /home/pi/riderlog.txt, e.g.
     ssh rider "grep -a 'rider/debug/telem' /home/pi/riderlog.txt | grep -a ' en=1 ' | grep -a 'ctrl=LQR'
                | grep -av 'vdes=0.00 \\|vdes=-0.00 '" > that file)

Cruising steadily on the flat the wheels only overcome the drive's own damping and its friction: the model
(sim/lqr/model.py) needs U_PER_MPS units per m/s plus FRIC_UNITS (the wheels against the body: the body is still).
A weaker battery needs more of the command for the same torque, so each steady stretch measures the strength,
    motor = (U_PER_MPS v + FRIC_UNITS sign(v)) / u
(1 = the model's motor; sim/lqr's 'motors x0.78' cases are motor 0.78). A stretch: at least MIN_S of telemetry
with the plan's speed constant above MIN_V, the wheels at it, no turn, the command off its limit. The strengths
are fitted against the battery voltage (vbat, the firmware's smoothed reading) as motor = k (vbat - v0) and as
motor = vbat / v_ref (proportional, as an open-loop drive's torque per command should be).
MEASURED 2026-11-03 (25 stretches, 10-21 .. 11-03, 6.96-8.27 V): motor = vbat / 8.29 V, 2.7 % rms left; a free line
gives an intercept of 0.24 V (proportional); forward 1.009, backward 0.993 of it; the three low-battery stretches
(6.96 / 7.73 / 7.87 V) on the line (0.840 / 0.919 / 0.938 measured, 0.84 / 0.93 / 0.95 predicted). On the
firmware's 6.0-8.4 V scale sim/lqr's motors x0.85 is ~7.0 V (~44 %), x0.78 ~6.5 V (~19 %). Aside: at 0.43 m/s the
strength reads ~5 % under 0.29 m/s at the same voltage -- the model's 333 units per m/s a little low, its friction
a little high.
"""
import re
import sys

import numpy as np

U_PER_MPS = 333.3          # model.py: 1 / (r ku_over_b) = 1 / (0.030 x 0.100)
FRIC_UNITS = 32.5          # model.py: fric / ku
SPEED_PER_MPS = 271.4      # telemetry wv units per m/s (main.cpp LQR_SPEED_PER_MPS)
MIN_V, MIN_S = 0.15, 1.0
KEYS = ("ms", "vdes", "wv", "u", "vbat", "yrc", "gturn", "th")


def load(path):
    rows, stamps = [], []
    for line in open(path, errors="replace"):
        d = dict(re.findall(r"(\w+)=(-?[\d.]+)", line))
        if all(k in d for k in KEYS):
            rows.append([float(d[k]) for k in KEYS])
            stamps.append(line[:16])
    return np.array(rows), stamps


def stretches(a):
    ms, vdes, wv, u, vbat, yrc, gturn, th = a.T
    v = wv / SPEED_PER_MPS
    ok = ((np.abs(vdes) > MIN_V) & (np.abs(v - vdes) < 0.04) & (np.abs(yrc) < 1e-3) & (np.abs(gturn) < 5)
          & (np.abs(u) < 380))
    out, i = [], 0
    while i < len(a):
        if not ok[i]:
            i += 1
            continue
        j = i
        while (j + 1 < len(a) and ok[j + 1] and abs(vdes[j + 1] - vdes[i]) < 0.005 and 0 < ms[j + 1] - ms[j] < 250):
            j += 1
        if ms[j] - ms[i] >= MIN_S * 1000:
            out.append((i, j + 1))
        i = j + 1
    return out


def main():
    a, stamps = load(sys.argv[1])
    ms, vdes, wv, u, vbat, yrc, gturn, th = a.T
    rows = []
    for i, j in stretches(a):
        v, uu = (wv[i:j] / SPEED_PER_MPS).mean(), u[i:j].mean()
        m = (U_PER_MPS * v + FRIC_UNITS * np.sign(v)) / uu
        rows.append((stamps[i], (ms[j - 1] - ms[i]) / 1000, v, uu, vbat[i:j].mean(), m))
    print(f"{len(a)} lines, {len(rows)} steady stretches (>= {MIN_S:g} s, |v| > {MIN_V} m/s, no turn)")
    print("  start              s     v m/s   command   vbat V   motor")
    for r in rows:
        print(f"  {r[0]}  {r[1]:4.1f}   {r[2]:+5.2f}   {r[3]:+7.1f}   {r[4]:5.2f}    {r[5]:.3f}")
    if len(rows) < 3:
        return
    R = np.array([r[1:] for r in rows])
    w, v, uu, vb, m = R.T
    good = (m > 0.3) & (m < 2.0)
    w, v, vb, m = w[good], v[good], vb[good], m[good]
    k, c = np.polyfit(vb, m, 1, w=np.sqrt(w))
    res = m - (k * vb + c)
    vref = np.sum(w * vb * vb) / np.sum(w * vb * m)      # m = vb / vref, weighted least squares
    res_p = m - vb / vref
    print(f"\nline fit: motor = {k:.3f} x (vbat - {-c / k:.2f} V)  (rms left {np.sqrt(np.average(res ** 2, weights=w)):.3f})")
    print(f"proportional: motor = vbat / {vref:.2f} V  (rms left {np.sqrt(np.average(res_p ** 2, weights=w)):.3f}); "
          f"the motor at 8.4 V (full) {8.4 / vref:.2f}, at 7.4 V {7.4 / vref:.2f}")
    for lab, s in (("forward", v > 0), ("backward", v < 0)):
        if s.any():
            print(f"  {lab:8s} {s.sum():3d} stretches: motor / (vbat / {vref:.2f}) = {np.mean(m[s] * vref / vb[s]):.3f}")


if __name__ == "__main__":
    main()
