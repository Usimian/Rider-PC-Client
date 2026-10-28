"""Today's LQR against the designed gains, on the full model with the firmware copy (closed_loop.py).

    python3 sim/lqr/compare_controllers.py [--x 0.015 --v 0.03 --th 1 --w 20 --u 100]

Each controller runs on the nonlinear model (friction, cogging, the encoder and speed-register steps, sensor
noise), the firmware cycle as closed_loop.py copies it:
  standing   10 s after 2 s to settle: pitch sd, wheel sd, command sd (the noise the readings put in)
  push       a 30 deg/s shove forward at 3 s: largest lean, largest travel, mean distance from home 1-2 s after
  push, motors x2   the same with the motors twice as strong as modelled (the robot answers up to 1.6x more)
Today = the flashed law (raw encoder, speed smoothing 0.85 per cycle); the designs = lqr_design.py's gains
with the encoders' body part taken off, at each speed smoothing, with friction compensation in place of the
factory boost (closed_loop.FRIC_*), and without it. All keep today's position integral (-30).
"""
import argparse
from dataclasses import replace

import numpy as np

import linear_model as L
from closed_loop import CYCLE_S, K_TODAY, LP_VEL, Firmware, run
from lqr_design import design
from model import Params

UP = 2.8


def standing(p, mk):
    rows, fell = run(p, mk(), T=12.0, th0_deg=0.3)
    if fell:
        return None
    r = rows[int(2.0 / (2 * CYCLE_S)):]
    return r[:, 5].std() / 100, r[:, 4].std() / 10, r[:, 2].std()


def push(p, mk):
    rows, fell = run(p, mk(), T=8.0, th0_deg=0.0, kick=(3.0, 30.0))
    if fell:
        return None
    t = (rows[:, 0] - rows[0, 0]) * 1e-6
    after = t >= 3.0
    lean = np.abs(rows[after, 5] / 100 - UP).max()
    x = rows[:, 4] / 10
    home = x[(t > 2.0) & (t < 3.0)].mean()
    dev = np.abs(x[after] - home)
    late = (t >= 4.0) & (t < 5.0)
    return lean, dev.max(), np.abs(x[late] - home).mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.015)
    ap.add_argument("--v", type=float, default=0.03)
    ap.add_argument("--th", type=float, default=1.0)
    ap.add_argument("--w", type=float, default=20.0)
    ap.add_argument("--u", type=float, default=100.0)
    a = ap.parse_args()
    p = Params()
    p2 = replace(p, ku_over_b=2 * p.ku_over_b)
    ctrls = [("today (as flashed)", lambda: Firmware(up_deg=UP))]
    found = []
    for tv in (0.100, 0.008):                                # the Hover's (flashed) and the one that shook
        a_v = float(np.exp(-CYCLE_S / tv))
        (K, J), _ = design(p, a_v, a.x, a.v, np.radians(a.th), np.radians(a.w), a.u, starts=found)
        found.append(K)
        k = dict(L.to_fw(K), xi=K_TODAY["xi"])
        lab = f"designed, {tv * 1000:.0f} ms [{k['x']:.0f} {k['vx']:.2f} {k['q']:.1f} {k['dq']:.2f}]"
        ctrls.append((lab, lambda k=k, a_v=a_v: Firmware(k=k, up_deg=UP, corrected=True, a_v=a_v)))
        ctrls.append(("   + friction compensation", lambda k=k, a_v=a_v: Firmware(k=k, up_deg=UP, corrected=True,
                                                                                a_v=a_v, fric_comp=True)))
    print(f"{'controller':44s} {'standing: pitch sd  wheel sd  cmd sd':38s} {'push: lean  travel  1-2 s after':31s} "
          f"{'motors x2: lean  travel  1-2 s after'}")
    for lab, mk in ctrls:
        st, pu, p2r = standing(p, mk), push(p, mk), push(p2, mk)
        s1 = "FELL" if st is None else f"{st[0]:6.3f} deg {st[1]:6.2f} mm {st[2]:6.1f}"
        f = lambda r: "FELL" if r is None else f"{r[0]:5.1f} deg {r[1]:5.1f} mm {r[2]:4.1f} mm"
        print(f"{lab:44s} {s1:38s} {f(pu):31s} {f(p2r)}")


if __name__ == "__main__":
    main()
