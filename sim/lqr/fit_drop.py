"""Fit the body's inertia about the axle to drop tests (ported from the Hover's sim/fit_drop.py, e2fa373).

    python3 sim/lqr/fit_drop.py [capture.csv] [--free-b]

The capture: tools/capture/tilt_capture.py windows, balance off, legs powered at ride height, the robot
held upright on the floor and let go (data/captures/2026-10-28_drop_tests.csv). With balance off the
wheels' torque is off, yet they feel braked by hand: the model keeps its motors at u = 0 (damping b);
--free-b also fits a factor on b for this state.

For each fall (|pitch| passing 30 deg from the upright reading): from release (the gyro rate last under
3 deg/s before it) to 30 deg, the angle (gyro integrated from the reading at release) and the encoder
travel (the left wheel's speed register integrated: the wheel against the body, model.encoder) are
compared with the model started from the same angle and rate. Fitted: ib_axle, plus each fall's
starting-angle correction (a fall from near upright is very sensitive to it; Hover: 0.3-0.8 deg).
Cost: RMS angle error (deg) + RMS wheel error (mm) / 2, as on the Hover.
"""
import os
import sys
from dataclasses import replace

import numpy as np
from scipy.optimize import minimize

from model import Params, encoder, step

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.join(HERE, "..", "..", "data", "captures", "2026-10-28_drop_tests.csv")
UPRIGHT_DEG = 3.55          # the IMU's reading with the centre of mass over the axle (firmware 'set')
V_PER_MPS = 327.6           # wheel speed register, raw per m/s forward (tools/balance/encoder_axle_check.py)
END_DEG = 30.0              # each fall is compared up to this lean (see falls())


def falls(path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    out = []
    for wdw in np.unique(a["window"]):
        r = a[a["window"] == wdw]
        t = (r["t_us"] - r["t_us"][0]) * 1e-6
        th = -np.radians(r["pitch_cdeg"] / 100.0 - UPRIGHT_DEG)   # model frame: + forward lean
        w = -np.radians(r["rate_x10"] / 10.0)
        v = r["v"] / V_PER_MPS
        past = np.nonzero(np.abs(th) > np.radians(30))[0]
        if not len(past):
            continue
        i30 = past[0]
        i0 = i30
        while i0 > 0 and abs(np.degrees(w[i0])) > 3.0:
            i0 -= 1
        if i0 == 0 or abs(th[i0]) > np.radians(15):              # no clean release in this window
            continue
        seg = slice(i0, min(len(t), i30 + 40))
        ts = t[seg] - t[i0]
        thg = th[i0] + np.concatenate([[0], np.cumsum(0.5 * (w[seg][1:] + w[seg][:-1]) * np.diff(ts))])
        xe = np.concatenate([[0], np.cumsum(0.5 * (v[seg][1:] + v[seg][:-1]) * np.diff(ts))])
        # up to 30 deg: by hand the falls were caught at 33-39 deg (2026-10-28), and a caught robot held
        # still must not count as falling (cutting at 40 deg, as on the Hover, kept the catch in)
        over = np.nonzero(np.abs(thg) >= np.radians(END_DEG))[0]
        keep = np.arange(len(thg)) < (over[0] if len(over) else len(thg))
        out.append(dict(window=int(wdw), t=ts[keep], th=thg[keep], x=xe[keep], w0=w[i0],
                        dur=ts[keep][-1], dirn="forward" if thg[keep][-1] > 0 else "backward"))
    return out


def simulate(p, f, dth0=0.0, dt=0.0005):
    s = np.array([0.0, 0.0, f["th"][0] + dth0, f["w0"]])
    th, x = [], []
    t = 0.0
    for tk in f["t"]:
        while t < tk - 1e-9:
            s = step(p, s, 0.0, dt)
            t += dt
        th.append(s[2])
        x.append(encoder(p, s)[0])
    return np.array(th), np.array(x)


def cost(z, data, free_b, verbose=False):
    """z: log ib_axle, [log b factor], then each fall's starting-angle correction (rad)."""
    p = replace(Params(), ib_axle=np.exp(z[0]))
    k = 1
    if free_b:
        p = replace(p, b=p.b * np.exp(z[1]), ku_over_b=p.ku_over_b / np.exp(z[1]))   # u = 0: only b acts
        k = 2
    c = 0.0
    for f, dth0 in zip(data, z[k:]):
        th, x = simulate(p, f, dth0)
        e_th = np.degrees(np.sqrt(np.mean((th - f["th"]) ** 2)))
        e_x = np.sqrt(np.mean((x - f["x"]) ** 2)) * 1000
        if verbose:
            print(f"    window {f['window']:2d} {f['dirn']:8s} ({f['dur']:.2f} s to 30 deg, start corrected "
                  f"{np.degrees(dth0):+.2f} deg): angle error {e_th:.2f} deg rms, wheel error {e_x:.1f} mm rms; "
                  f"wheel travel model {x[-1] * 1000:+.1f} mm, measured {f['x'][-1] * 1000:+.1f} mm")
        c += e_th + e_x / 2
    return c


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    free_b = "--free-b" in sys.argv
    data = falls(args[0] if args else REC)
    p0 = Params()
    print(f"{len(data)} clean falls; start: ib_axle {p0.ib_axle * 1e4:.1f} kg cm^2, body {p0.mb * 1000:.0f} g "
          f"with its centre {p0.L * 1000:.1f} mm above the axle")
    if not data:
        return
    z0 = np.r_[np.log(p0.ib_axle), [0.0] if free_b else [], np.zeros(len(data))]
    cost(z0, data, free_b, verbose=True)
    res = minimize(cost, z0, args=(data, free_b), method="Nelder-Mead",
                   options=dict(xatol=1e-4, fatol=1e-4, maxiter=4000))
    ib = np.exp(res.x[0])
    p = replace(p0, ib_axle=ib)
    lam = np.sqrt(p.mb * 9.81 * p.L / ib)
    print(f"fitted: ib_axle {ib * 1e4:.1f} kg cm^2 (about its own centre {p.ib_com * 1e4:.2f}); fixed-pivot "
          f"tipping rate {lam:.1f} /s" + (f"; damping x{np.exp(res.x[1]):.2f}" if free_b else ""))
    cost(res.x, data, free_b, verbose=True)


if __name__ == "__main__":
    main()
