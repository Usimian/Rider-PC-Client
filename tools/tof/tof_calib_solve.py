#!/usr/bin/env python3
"""Solve the ToF floor-reference kinematic model from flat-floor captures.

Takes N capture files (from tof_calib_capture.py, one per leg height) and fits
the four fixed unknowns the model needs:
  - IMU sign convention (s_roll, s_pitch),
  - fixed mount offset (beta_roll, beta_pitch)  [shim + IMU bias],
  - the leg-index -> sensor-height map  H = A*(legR-legL) + C.

Model: a zone return is P = d * unit_ray(el,az) in the SENSOR frame.
World-leveled point = R_imu(s_r*roll, s_p*pitch) . R_mount(beta_r, beta_p) . P.
On a flat floor every floor return's world_z must equal -H(leg_index): flat
(same across zones in a frame), stable across the balance dance, and linear in
leg index. We minimize the SSE of that condition; the signs+offsets that
flatten/stabilize the floor are the calibration. Paste the printed constants
into rider_tof_safety.py and tof_pointcloud_node.py.

    python3 tools/tof/tof_calib_solve.py calib_low.json calib_mid.json calib_high.json
"""
import itertools
import json
import math
import sys

import numpy as np
from scipy.optimize import minimize

FLOOR_ROWS = (5, 6, 7)
DMIN, DMAX = 0.08, 1.10                       # m: plausible floor slant range
_half, _step = 22.5, 45.0 / 8.0
_ang = [-_half + (k + 0.5) * _step for k in range(8)]


def unit_ray(i, j):
    el = math.radians(-_ang[i]); az = math.radians(-_ang[j])
    ce, se, ca, sa = math.cos(el), math.sin(el), math.cos(az), math.sin(az)
    return np.array([ce * ca, ce * sa, se])

RAY = [[unit_ray(i, j) for j in range(8)] for i in range(8)]


def Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def main():
    files = sys.argv[1:]
    if not files:
        sys.exit("usage: tof_calib_solve.py <capture1.json> <capture2.json> ...")

    pts, roll, pitch, idx = [], [], [], []
    for fn in files:
        for s in json.load(open(fn)):
            li = s["legR"] - s["legL"]
            for i in FLOOR_ROWS:
                for j in range(8):
                    k = i * 8 + j
                    if s["s"][k] in (5, 6):
                        dm = s["d"][k] / 1000.0
                        if DMIN < dm < DMAX:
                            pts.append(dm * RAY[i][j]); roll.append(s["roll"])
                            pitch.append(s["pitch"]); idx.append(float(li))
    P = np.array(pts); ROLL = np.array(roll); PITCH = np.array(pitch); IDX = np.array(idx)
    print("floor returns: %d  (idx values: %s)" % (len(P), sorted(set(int(x) for x in IDX))))

    def world_z(sr, sp, br, bp):
        Rm = Rx(math.radians(br)) @ Ry(math.radians(bp))
        PM = P @ Rm.T
        z = np.empty(len(P))
        for n in range(len(P)):
            Ri = Rx(math.radians(sr * ROLL[n])) @ Ry(math.radians(sp * PITCH[n]))
            z[n] = (Ri @ PM[n])[2]
        return z

    def sse(params, sr, sp):
        z = world_z(sr, sp, params[0], params[1])
        A = np.vstack([IDX, np.ones_like(IDX)]).T
        coef, *_ = np.linalg.lstsq(A, z, rcond=None)
        return float(np.sum((z - A @ coef) ** 2))

    best = None
    for sr, sp in itertools.product((1, -1), (1, -1)):
        r = minimize(sse, [0.0, 0.0], args=(sr, sp), method="Nelder-Mead",
                     options={"xatol": 1e-3, "fatol": 1e-6, "maxiter": 4000})
        rms = math.sqrt(r.fun / len(P)) * 1000
        print("  signs r=%+d p=%+d: beta_roll=%+6.2f beta_pitch=%+6.2f  floor RMS=%5.1f mm"
              % (sr, sp, r.x[0], r.x[1], rms))
        if best is None or r.fun < best[0]:
            best = (r.fun, sr, sp, r.x[0], r.x[1])

    _, sr, sp, br, bp = best
    z = world_z(sr, sp, br, bp)
    A = np.vstack([IDX, np.ones_like(IDX)]).T
    m, b = np.linalg.lstsq(A, z, rcond=None)[0]          # z ~ m*idx + b = -H
    rms = math.sqrt(np.mean((z - A @ [m, b]) ** 2)) * 1000
    print("\n==================  SOLVED MODEL (paste into the consumers)  ==================")
    print("  ROLL_SIGN, PITCH_SIGN = %.1f, %.1f" % (sr, sp))
    print("  BETA_ROLL, BETA_PITCH = %.2f, %.2f      # deg" % (br, bp))
    print("  H_A, H_C = %.6f, %.4f                    # H = H_A*(legR-legL) + H_C  [m]" % (-m, -b))
    print("  floor flatness RMS: %.1f mm" % rms)


if __name__ == "__main__":
    main()
