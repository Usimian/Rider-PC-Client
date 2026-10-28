#!/usr/bin/env python3
"""Where is the IMU relative to the wheel axle, and does the encoder read against the body? (2026-10-28)

Input: tools/capture/tilt_capture.py windows -- the robot rocked BY HAND about its wheel axle, balance off,
legs powered at ride height, wheels kept on their spot.

1. ENCODER: with the axle still, a wheel servo that reads the wheel's turn against the body reports a speed of
   -r * (forward lean rate). Fit: encoder speed (m/s, + forward) = k * r * theta_fwd_dot; expect k = -1.
2. IMU POSITION: an accelerometer at (d forward, h up) of the axle on a body pitching about it reads, in body
   axes (theta = forward lean, alpha = its angular acceleration, w = its rate):
       forward  F = -g sin(theta) + alpha*h - w^2*d          up  U = g cos(theta) - alpha*d - w^2*h
   The firmware then applies its lever-arm correction (main.cpp: ay -= imucomp*KIMU*alpha_f,
   az += imucomp*KIMU*w^2, i.e. -0.5 x 120 mm, alpha_f its 0.8/0.2 per-cycle filtered derivative) and records
   pacc = atan2(ayC, azC) in the firmware frame (forward lean makes it decrease). True pitch comes from the
   gyro, integrated (the fused th leans on the accelerometer itself); a per-window offset and drift are fitted
   with h and d. Any axle motion left in the data (wheels rolling) enters F as its acceleration, taken from
   the encoder with step 1's correction.

Run:  /usr/bin/python3 tools/balance/imu_position_fit.py data/captures/2026-10-28_imu_tilt.csv [windows, e.g. 6,7]
"""
import sys
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.optimize import least_squares

R = 0.030                  # wheel radius, m
G = 9.81
D2R = np.pi / 180.0
V_PER_MPS = 327.6          # wheel speed register, raw per m/s forward (encoder_axle_check.py)
COMP_M = -0.5 * 0.12       # firmware lever-arm correction: imucomp x KIMU's 120 mm
LP_HZ = 12.0


def lowp(x, fs, hz=LP_HZ):
    b, a = butter(2, hz / (fs / 2))
    return filtfilt(b, a, x)


def windows(path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    for w in np.unique(a["window"]):
        r = a[a["window"] == w]
        t = (r["t_us"] - r["t_us"][0]) * 1e-6
        if len(t) < 300 or np.any(np.diff(t) <= 0) or np.abs(r["rate_x10"]).max() < 200:   # < 20 deg/s: no rocking
            continue
        fs = 1.0 / np.median(np.diff(t))
        th_fw = r["pitch_cdeg"] / 100.0                     # firmware pitch, deg (forward lean -> decrease)
        w_fwd = -r["rate_x10"] / 10.0 * D2R                 # forward lean rate, rad/s
        pacc_fwd = -r["pacc_x100"] / 100.0 * D2R            # firmware accel pitch, as forward lean, rad
        v_fwd = r["v"] / V_PER_MPS                          # left wheel encoder speed, m/s forward
        yield int(w), dict(t=t, fs=fs, th0=-th_fw[0] * D2R, w=w_fwd, pacc=pacc_fwd, v=v_fwd)


def main():
    path = sys.argv[1]
    only = {int(x) for x in sys.argv[2].split(",")} if len(sys.argv) > 2 else None   # e.g. 6,7
    W = [(wi, d) for wi, d in windows(path) if only is None or wi in only]
    print(f"{len(W)} windows from {path}")

    # ---- 1. encoder speed vs r * lean rate ----
    xs, ys = [], []
    for _, d in W:
        xs.append(lowp(d["w"], d["fs"]) * R); ys.append(lowp(d["v"], d["fs"]))
    xs, ys = np.concatenate(xs), np.concatenate(ys)
    k = float(np.dot(xs, ys) / np.dot(xs, xs))
    r2 = 1 - np.sum((ys - k * xs) ** 2) / np.sum((ys - ys.mean()) ** 2)
    print(f"\n1. ENCODER  speed = {k:+.3f} x r x lean rate  (R^2 {r2:.3f}; -1 = reads the wheel against the body,"
          f" 0 = reads the axle)")
    print(f"   lean rate up to {np.abs(xs).max() / R / D2R:.0f} deg/s; encoder speed up to {np.abs(ys).max()*1000:.0f} mm/s")
    for wi, d in W:                                          # per window: below -1 or above it = the axle rolled
        x, y = lowp(d["w"], d["fs"]) * R, lowp(d["v"], d["fs"])
        kw = float(np.dot(x, y) / np.dot(x, x))
        print(f"     window {wi}: {kw:+.2f}  (R^2 {1 - np.sum((y - kw * x) ** 2) / np.sum((y - y.mean()) ** 2):.2f},"
              f" lean rate up to {np.abs(d['w']).max() / D2R:.0f} deg/s)")

    # ---- 2. IMU position ----
    prep = []
    for wi, d in W:
        fs, t = d["fs"], d["t"]
        w = lowp(d["w"], fs)
        alpha = np.gradient(w, t)
        # the firmware's filtered alpha (0.8/0.2 per cycle on the raw derivative), emulated on the same signal
        af = np.zeros_like(alpha)
        for i in range(1, len(alpha)):
            af[i] = 0.8 * af[i - 1] + 0.2 * alpha[i]
        theta_g = d["th0"] + np.concatenate([[0], np.cumsum(0.5 * (d["w"][1:] + d["w"][:-1]) * np.diff(t))])
        v_axle = lowp(d["v"], fs) + R * w                    # axle speed with the body term taken out
        a_axle = np.gradient(v_axle, t)
        prep.append((wi, t, w, alpha, af, theta_g, a_axle, lowp(d["pacc"], fs)))

    nW = len(prep)

    def resid(p, axle_on):
        # Frames: the firmware's ay is the forward specific force F and az the up one U (pacc_fw = atan2(ay, az)
        # falls on a forward lean); its gyro rate and alpha are minus ours. Its correction is therefore
        # ayC = F + COMP_M*alpha_f, azC = U + COMP_M*w^2 (COMP_M < 0 removes 60 mm of the tangential term).
        h, dd = p[0], p[1]
        out = []
        for j, (wi, t, w, alpha, af, theta_g, a_axle, pacc) in enumerate(prep):
            th = theta_g + p[2 + 2 * j] + p[3 + 2 * j] * t
            a = a_axle if axle_on else 0.0                  # the axle's own forward acceleration (world horizontal)
            F = -G * np.sin(th) + alpha * h - w ** 2 * dd + a * np.cos(th)
            U = G * np.cos(th) - alpha * dd - w ** 2 * h + a * np.sin(th)
            out.append(-np.arctan2(F + COMP_M * af, U + COMP_M * w ** 2) - pacc)
        return np.concatenate(out)

    for axle_on in (True, False):
        p0 = np.zeros(2 + 2 * nW); p0[0] = 0.10
        sol = least_squares(resid, p0, args=(axle_on,))
        res = sol.fun
        J = sol.jac; cov = np.linalg.pinv(J.T @ J) * np.sum(res ** 2) / max(1, len(res) - len(p0))
        se = np.sqrt(np.diag(cov))
        tot = np.concatenate([pp[7] for pp in prep])
        print(f"\n2. IMU POSITION ({'axle motion from the corrected encoder' if axle_on else 'axle assumed still'}):")
        print(f"   height above the axle h = {sol.x[0]*1000:6.1f} +/- {se[0]*1000:.1f} mm")
        print(f"   forward of the axle   d = {sol.x[1]*1000:6.1f} +/- {se[1]*1000:.1f} mm   (+ = ahead)")
        print(f"   fit: residual {np.degrees(res.std()):.2f} deg rms against accel pitch {np.degrees(tot.std()):.2f} deg rms")


if __name__ == "__main__":
    main()
