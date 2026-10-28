#!/usr/bin/env python3
"""Does the wheel encoder read the axle, or the wheel's turn against the body? (Hover finding, 2026-10-28)

The wheel servos sit on the legs, so their encoders measure the wheel's turn RELATIVE TO THE BODY: when the
body rocks by d(theta) with the axle still, the encoder reports r*d(theta) of travel that did not happen.
If so, the true axle is  x_axle = x_enc + r*theta_fwd  (theta_fwd = forward lean, rad; r = 30 mm).

Two checks on existing drivecap captures (data/captures/*, columns v, pitch_cdeg, rate_x10, pacc_x100,
pol_xvel_mm), no robot needed:

 1. FRAME: at 0.3-1.5 Hz a balancing robot accelerates the way it leans, so the sign of the encoder
    acceleration against the firmware pitch says which way forward lean runs in 'th'.
 2. AXLE: the lever-compensated accelerometer pitch minus the fused pitch reads the axle's true
    acceleration (-kappa*a/g). Fitted on [encoder acceleration, body angular acceleration] at 0.5-6 Hz,
    the ratio of the two coefficients is an effective lever L = s*r - (IMU-height error)/kappa:
    ~+30 mm if the encoder carries the body's rocking and the firmware's 120 mm IMU height is right,
    ~0 if the encoder were a true axle reading. Only the SUM is identifiable from these signals.

Run:  /usr/bin/python3 tools/balance/encoder_axle_check.py [capture.csv ...]   (default: every drivecap
      file with pacc_x100). Spin episodes are skipped (one wheel's speed is not the axle's in a spin).
"""
import sys, glob, os
import numpy as np
from scipy.signal import butter, filtfilt

R = 0.030          # wheel radius, m (sim/rider_params.py, KNOWN)
G = 9.81
D2R = np.pi / 180.0
FS = 160.0         # uniform resample rate, Hz (drivecap rows are ~6 ms apart)


def segments(path):
    """Yield (label, dict of uniform-rate arrays) per continuous non-spin run in one capture."""
    a = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    eps = a["episode"]
    for e in np.unique(eps):
        r = a[eps == e]
        label = str(r["dir"][0])
        if label.startswith("spin"):
            continue
        if r["v"].std() < 3 or np.abs(r["pitch_cdeg"]).max() > 2000:   # standing still / disabled, or a fall
            continue
        t = (r["t_us"] - r["t_us"][0]) * 1e-6
        cuts = np.where(np.diff(t) > 0.05)[0] + 1          # split at gaps
        for idx in np.split(np.arange(len(t)), cuts):
            if len(idx) < 200:
                continue
            tt = t[idx]
            tu = np.arange(tt[0], tt[-1], 1.0 / FS)
            out = {k: np.interp(tu, tt, r[k][idx].astype(float))
                   for k in ("v", "pitch_cdeg", "rate_x10", "pacc_x100", "pol_xvel_mm")}
            yield f"{os.path.basename(path)}:{label}", out


def band(x, lo, hi):
    b, a = butter(2, [lo / (FS / 2), hi / (FS / 2)], btype="band")
    return filtfilt(b, a, x)


def lowp(x, hi):
    b, a = butter(2, hi / (FS / 2))
    return filtfilt(b, a, x)


def main():
    files = sys.argv[1:] or sorted(f for f in glob.glob(os.path.join(
        os.path.dirname(__file__), "..", "..", "data", "captures", "*.csv"))
        if "pacc_x100" in open(f).readline())
    segs = [s for f in files for s in segments(f)]
    print(f"{len(segs)} non-spin runs from {len(files)} captures")

    # --- speed register scale + sign: v (raw, one wheel) against the policy's encoder velocity (mm/s,
    #     + = forward), both low-passed to 1 Hz (pol_xvel is already smoothed in firmware) ---
    vv, xv = [], []
    for _, s in segs:
        if np.any(s["pol_xvel_mm"] != 0):
            vv.append(lowp(s["v"], 1.0)); xv.append(lowp(s["pol_xvel_mm"], 1.0) / 1000.0)
    vv, xv = np.concatenate(vv), np.concatenate(xv)
    k = float(np.dot(vv, vv) / np.dot(vv, xv))              # v = k * v_fwd[m/s]
    r2 = 1 - np.sum((vv - k * xv) ** 2) / np.sum((vv - vv.mean()) ** 2)
    print(f"speed register: {k:+.1f} raw per m/s forward (fit R^2 {r2:.3f})")

    # --- 1. FRAME: encoder acceleration vs firmware pitch at 0.3-1.5 Hz ---
    A, TH = [], []
    for _, s in segs:
        vf = s["v"] / k
        A.append(np.gradient(band(vf, 0.3, 1.5), 1.0 / FS))
        TH.append(band(s["pitch_cdeg"] / 100.0 * D2R, 0.3, 1.5))
    A, TH = np.concatenate(A), np.concatenate(TH)
    c = np.corrcoef(A, TH)[0, 1]
    print(f"\n1. FRAME  corr(forward accel, th) at 0.3-1.5 Hz = {c:+.2f}"
          f"  -> forward lean makes th {'INCREASE' if c > 0 else 'DECREASE'}")
    fsign = 1.0 if c > 0 else -1.0                          # theta_fwd = fsign * th

    # --- 2. AXLE: (pacc - th) vs [encoder accel, body angular accel], 0.5-6 Hz ---
    Y, XA, XB, per = [], [], [], []
    for lab, s in segs:
        vf = s["v"] / k
        thf = fsign * s["pitch_cdeg"] / 100.0 * D2R
        wf = fsign * s["rate_x10"] / 10.0 * D2R
        pf = fsign * s["pacc_x100"] / 100.0 * D2R
        y = band(pf - thf, 0.5, 6.0)
        xa = np.gradient(band(vf, 0.5, 6.0), 1.0 / FS) / G
        xb = np.gradient(band(wf, 0.5, 6.0), 1.0 / FS) / G
        Y.append(y); XA.append(xa); XB.append(xb)
        M = np.column_stack([xa, xb]); cf, *_ = np.linalg.lstsq(M, y, rcond=None)
        r2 = 1 - np.sum((y - M @ cf) ** 2) / np.sum((y - y.mean()) ** 2)
        per.append((lab, cf[1] / cf[0] * 1000.0, -cf[0], r2))
    Y, XA, XB = map(np.concatenate, (Y, XA, XB))

    def fit(cols):
        M = np.column_stack(cols); cf, *_ = np.linalg.lstsq(M, Y, rcond=None)
        return cf, 1 - np.sum((Y - M @ cf) ** 2) / np.sum((Y - Y.mean()) ** 2)
    (ca,), r2a = fit([XA])
    (cb, cc), r2b = fit([XA, XB])
    print(f"\n2. AXLE   accelerometer gap vs encoder accel only:          kappa {-ca:.2f}, R^2 {r2a:.3f}")
    print(f"          + body angular accel:  kappa {-cb:.2f}, lever L = {cc / cb * 1000:+.1f} mm, R^2 {r2b:.3f}")
    print(f"          (encoder = axle -> L ~ 0;  encoder against the body, 120 mm IMU right -> L ~ +{R*1000:.0f} mm)")
    for lab, s_ in [(f"axle = enc + r*theta_fwd", +1), (f"axle = enc - r*theta_fwd", -1), ("axle = enc", 0)]:
        (c1,), r2_ = fit([XA + s_ * R * XB])
        print(f"          {lab:26s}  kappa {-c1:.2f}  R^2 {r2_:.3f}")
    good = [p for p in per if p[3] > 0.3 and p[2] > 0.3]          # the accelerometer gap explained at all
    Ls = np.array([p[1] for p in good])
    print(f"\n   per run (fit R^2 > 0.3, kappa > 0.3: {len(good)} of {len(per)}): L median {np.median(Ls):+.1f} mm, "
          f"quartiles {np.percentile(Ls,25):+.1f} .. {np.percentile(Ls,75):+.1f} mm")
    for lab, L, kap, r2 in per:
        print(f"     {lab:42s} L {L:+7.1f} mm  kappa {kap:.2f}  R^2 {r2:.2f}")

    # --- 3. SIZE: on the LQR standing frequency sweep, how much of the encoder's motion is r*theta ---
    chirp = os.path.join(os.path.dirname(__file__), "..", "..", "data", "captures", "2026-10-19_chirp_lqr.csv")
    a = np.genfromtxt(chirp, delimiter=",", names=True, dtype=None, encoding=None)
    t = (a["t_us"] - a["t_us"][0]) * 1e-6
    tu = np.arange(0, t[-1], 1.0 / FS)
    wx = np.interp(tu, t, a["wx_01mm"] / 10.0)                          # encoder position, mm
    body = np.interp(tu, t, fsign * a["th_x100"] / 100.0 * D2R) * R * 1000.0   # r*theta_fwd, mm
    print(f"\n3. SIZE   LQR standing sweep ({os.path.basename(chirp)}), encoder position vs r*theta:")
    for lo, hi in [(0.3, 1.0), (1.0, 2.0), (2.0, 3.5), (3.5, 6.0)]:
        e, b = band(wx, lo, hi), band(body, lo, hi)
        print(f"     {lo:.1f}-{hi:.1f} Hz  encoder {e.std():5.2f} mm rms   r*theta {b.std():5.2f} mm rms "
              f"({b.std() / e.std() * 100:4.0f}% of it)   corr {np.corrcoef(e, b)[0, 1]:+.2f}")


if __name__ == "__main__":
    main()
