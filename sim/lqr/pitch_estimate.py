"""The LQR's pitch estimate while driving, with and without the robot's acceleration taken out of the accelerometer
(2026-11-03; closed_loop.Firmware acc_comp).

    python3 sim/lqr/pitch_estimate.py                     # the estimate's error, then the driving cases
    python3 sim/lqr/pitch_estimate.py --a-max 0.6 1.0     # the plan's acceleration cap(s)

The accelerometer reads a forward acceleration a as a tilt of a/g; the complementary filter blends it in at
1/(|rate| + 200) a sample (~0.5 s), so through a start the estimate drifts toward reading less lean than the body
has, and recovers as slowly after. The LQR balances on the estimate. Per variant: the estimate less the true lean
over a full-stick start and its stop (worst, and what is left 1 s after the start), then drive_design.py's cases.
"""
import argparse
from dataclasses import replace

import numpy as np

import closed_loop as CL
import drive_design as DD
from model import Params

VARIANTS = ((None, "as flashed"), ("plan", "less the plan's acceleration"), ("wheels", "less the wheels' acceleration"))


class _Rec(CL.Firmware):
    """The flashed LQR, recording the LQR's pitch (its frame turned to the model's: + forward) per cycle."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.est = []

    def _plan_acc(self, dt, stick, held, x, pitch):
        super()._plan_acc(dt, stick, held, x, pitch)
        self.est.append(-pitch)


def flashed(acc_comp, a_max, rec=False):
    plan = dict(CL.FLASHED_PLAN, a_max=a_max)
    return (_Rec if rec else CL.Firmware)(k=CL.FLASHED_K, up_deg=DD.UP, fric_comp=True, plan=plan, hover_imu=True,
                                          tau_v=0.100, acc_comp=acc_comp)


def estimate_error(a_max, V=0.45):
    print(f"\nthe estimate less the true lean (deg), full stick {V} m/s at 2 s, released at 4 s, plan {a_max} m/s^2:")
    for acc, name in VARIANTS:
        fw = flashed(acc, a_max, rec=True)
        log, fell = CL.run(Params(), fw, T=6.0, th0_deg=0.0, drive=(np.array([0.0, 2.0, 4.0]), np.array([0.0, V, 0.0])),
                           log_all=True)
        t, err = log[:, 0], np.array(fw.est[:len(log)]) - log[:len(fw.est), 4]
        start, stop = (t >= 2.0) & (t < 4.0), (t >= 4.0) & (t < 6.0)
        after = np.abs(t - 3.0) < 0.05
        print(f"  {name:32s} start: worst {err[start].min():+5.2f} / {err[start].max():+5.2f}, 1 s on "
              f"{err[after].mean():+5.2f}   stop: worst {err[stop].min():+5.2f} / {err[stop].max():+5.2f}"
              + ("   FELL" if fell else ""))


def cases(a_max, motors, speeds):
    p0 = Params()
    for motor in motors:
        p = replace(p0, ku_over_b=p0.ku_over_b * motor)
        print(f"\nplan {a_max} m/s^2, motors x{motor:g}:      start: 90% in  lean (deg)     at limit | stop: rolls on  "
              f"lean (deg)     | reversal: lean (deg)     at limit")
        for V in speeds:
            for acc, name in VARIANTS:
                mk = lambda acc=acc: flashed(acc, a_max)
                log, fell = DD.case(p, mk, (np.array([0.0, 2.0, 5.0]), np.array([0.0, V, 0.0])), 9.0)
                lab = f"  {V:.2f} m/s, {name}"
                if fell:
                    print(f"{lab:46s} FELL at {log[-1, 0]:.2f} s ({'start' if log[-1, 0] < 5 else 'stop'})")
                    continue
                t90, l0, l1, lim = DD.describe(log, 2.0, V)
                t = log[:, 0]
                st = (t >= 5.0) & (t < 8.0)
                x = np.cumsum(log[:, 3] * np.gradient(t))
                roll = (x[st][-1] - x[st][0]) * 1000
                rl, fr = DD.case(p, mk, (np.array([0.0, 2.0, 5.0, 7.5]), np.array([0.0, V, -V, 0.0])), 9.0)
                if fr:
                    rev = f"FELL at {rl[-1, 0]:.2f} s"
                else:
                    _, r0, r1, rlim = DD.describe(rl, 5.0, -V, window=2.0)
                    rev = f"{r0:+5.1f}..{r1:+5.1f}   {rlim:4.0f}%"
                print(f"{lab:46s} {t90:5.2f} s  {l0:+5.1f}..{l1:+5.1f}  {lim:5.0f}%  | {roll:5.0f} mm  "
                      f"{log[st, 4].min():+5.1f}..{log[st, 4].max():+5.1f}  | {rev}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a-max", type=float, nargs="+", default=[0.6, 1.0])
    ap.add_argument("--motors", type=float, nargs="+", default=[1.0, 0.78])
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.30, 0.45, 0.60])
    ap.add_argument("--vmax", type=float, default=0.6, help="the firmware's top speed (closed_loop.VMAX)")
    a = ap.parse_args()
    CL.VMAX = a.vmax
    for am in a.a_max:
        estimate_error(am)
    for am in a.a_max:
        cases(am, a.motors, a.speeds)


if __name__ == "__main__":
    main()
