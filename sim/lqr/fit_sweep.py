"""Fit the motors to the floor frequency sweeps by short-horizon prediction (ported from the Hover's
sim/fit_sweep.py, e2fa373).

    python3 sim/lqr/fit_sweep.py                       # score the model as it stands, then fit
    python3 sim/lqr/fit_sweep.py --free b fric th_off --score-only

At many moments the model starts from the robot's measured state, is fed the commands the robot sent over
the next HORIZON samples (60 ms), and its change of pitch and encoder position is compared with what was
measured (cost: pitch error in deg + wheel error in mm / 2, RMS, as on the Hover). The Rider's sweeps
(tools/capture/chirp_capture.py: the firmware 'chirp', a known swept sine added to the wheel command)
record every 2nd cycle, ~10 ms apart; each recorded command is held until the next sample.

  * pitch: the recorded estimate at the start, as forward lean, less th_off (fitted: where the robot
    really balances; the drop tests put it ~0.8 deg off the stored 3.55); then the gyro integrated
  * wheel position and speed: the encoder (wheel_x, the wheel against the body), speed over 20 ms
  * command: model u = (cmdL - cmdR) / 2 (the firmware's balance output plus the sweep; turn cancels)

Fitted on the LQR standing sweep (chirp_lqr, 10-19); the driving sweeps are scored alongside as a check:
the LQR at ~0.11 m/s (chirp_drive, chirp_full) and the policy at ~0.25 m/s (chirp_fast_*), where the
MuJoCo sim had to make its wheel lag grow with speed.

Free parameters: any of b, u0, fric, fric_w, ku_over_b, ib_axle, iw_one (model.Params), plus th_off.
"""
import argparse
import os
from dataclasses import replace

import numpy as np
from scipy.optimize import minimize

from model import Params, encoder, step

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "..", "data", "captures")
FIT = ["2026-10-29_chirp_lqr_stand_10hz.csv", "2026-10-29_chirp_lqr_drive_fwd.csv"]
CHECK = ["2026-10-29_chirp_lqr_stand_30hz.csv", "2026-10-29_chirp_lqr_drive_rev.csv"]
HORIZON = 6    # samples, ~60 ms
EVERY = 2      # a window starts every 2nd sample
SUB = 10       # model steps per sample
LOG = {"b", "fric", "ib_axle", "ku_over_b", "fric_w", "iw_one"}  # fitted on a log scale
TH_OFF0 = -2.7  # deg, model frame: the drop tests' balance point (2.7 deg on the IMU)


def windows(name):
    """Start states, commands, sample lengths and measured changes for every clean window."""
    d = np.genfromtxt(os.path.join(DATA, name), delimiter=",", names=True, dtype=None, encoding=None)
    t = (d["t_us"] - d["t_us"][0]) * 1e-6
    th = -np.radians(d["th_x100"] / 100.0)        # forward lean (firmware: forward makes th decrease)
    w = -np.radians(d["rate_x10"] / 10.0)
    x = d["wx_01mm"] / 1e4
    u = (d["cmdL"] - d["cmdR"]) / 2.0
    ok = np.diff(t) < 0.03
    starts = [i for i in range(2, len(t) - HORIZON - 1, EVERY) if ok[i - 2:i + HORIZON].all()]
    i = np.array(starts)
    j = i[:, None] + np.arange(HORIZON + 1)[None, :]
    tw = t[j]
    v0 = (x[i] - x[i - 2]) / (t[i] - t[i - 2])
    dth = np.sum(0.5 * (w[j[:, :-1]] + w[j[:, 1:]]) * np.diff(tw, axis=1), axis=1)
    speed = np.abs(np.mean(np.diff(x) / np.diff(t)))
    return dict(th=th[i], w=w[i], v=v0, u=u[j[:, :-1]], dt=np.diff(tw, axis=1),
                dth=dth, dx=x[i + HORIZON] - x[i], name=name, ctrl=str(d["ctrl"][0]), speed=speed)


def predict(p, W, th_off):
    th = W["th"] - th_off
    # the measured speed is the encoder's (v - r w); the axle's is v_enc + r w
    s = np.vstack([p.r * th, W["v"] + p.r * W["w"], th, W["w"]])
    th0 = s[2].copy()
    for k in range(HORIZON):
        h = W["dt"][:, k] / SUB
        for _ in range(SUB):
            s = step(p, s, W["u"][:, k], h)
    return s[2] - th0, encoder(p, s)[0]


def score(p, th_off, sets):
    out = []
    for W in sets:
        pth, px = predict(p, W, th_off)
        rth = np.degrees(np.sqrt(np.mean((pth - W["dth"]) ** 2)))
        rx = np.sqrt(np.mean((px - W["dx"]) ** 2)) * 1000
        out.append((rth, rx, rth + rx / 2))
    return np.array(out)


def unpack(z, free, base):
    vals = {n: (np.exp(v) if n in LOG else v) for n, v in zip(free, z)}
    th_off = np.radians(vals.pop("th_off", TH_OFF0))
    return replace(base, **vals), th_off


def report(label, p, th_off, fit, check):
    print(f"  {label}:")
    for k, (W, (rth, rx, c)) in enumerate(zip(fit + check, score(p, th_off, fit + check))):
        print(f"      {W['name']:30s} {W['ctrl']} {W['speed']:.2f} m/s: pitch {rth:.3f} deg, wheels {rx:.2f} mm, "
              f"cost {c:.3f}{'   (fitted on)' if k < len(fit) else ''}")
    print(f"      b {p.b:.2e}  u0 {p.u0:.1f}  fric {p.fric * 1000:.2f} mN m ({p.fric / p.ku:.0f} units)  "
          f"fric_w {p.fric_w:.3f} rad/s  ku/b {p.ku_over_b:.3f}  ib {p.ib_axle * 1e4:.2f} kg cm^2  "
          f"iw {p.iw_one * 1e5:.2f}e-5  th_off {np.degrees(th_off):+.2f} deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--free", nargs="+", default=["b", "u0", "fric", "fric_w", "th_off"])
    ap.add_argument("--score-only", action="store_true")
    a = ap.parse_args()
    fit = [windows(f) for f in FIT]
    check = [windows(f) for f in CHECK]
    for W in fit + check:
        print(f"{W['name']}: {len(W['th'])} windows, measured over ~60 ms: pitch change sd "
              f"{np.degrees(W['dth'].std()):.3f} deg, wheel travel sd {W['dx'].std() * 1000:.2f} mm")
    base = Params()
    z0 = np.array([np.log(getattr(base, n)) if n in LOG else (TH_OFF0 if n == "th_off" else getattr(base, n))
                   for n in a.free])
    p, th = unpack(z0, a.free, base)
    report("as it stands", p, th, fit, check)
    if a.score_only:
        return

    def cost(z):
        p, th = unpack(z, a.free, base)
        if p.u0 < 0 or p.fric < 0:
            return 1e3
        return np.mean(score(p, th, fit)[:, 2])

    res = minimize(cost, z0, method="Nelder-Mead", options=dict(xatol=1e-3, fatol=1e-4, maxiter=1500))
    p, th = unpack(res.x, a.free, base)
    report(f"fitted ({res.nfev} tries)", p, th, fit, check)


if __name__ == "__main__":
    main()
