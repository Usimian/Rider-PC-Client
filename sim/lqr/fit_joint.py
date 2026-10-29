"""The body's inertia and the motor fitted to the floor sweeps and the drop tests together (2026-10-29).

    python3 sim/lqr/fit_joint.py           # model.py's rigid robot
    python3 sim/lqr/fit_joint.py --flex    # model_flex.py's body sprung on its legs (+ flex_k, flex_c)
    python3 sim/lqr/fit_joint.py --flex-k 2.8 --flex-c 0.015   # the sprung body, its spring held at these

The sweeps alone trade inertia against motor strength, the drop tests alone (balancing off: the wheels' torque off,
model.Params.off()) inertia against the braking; each pins what the other cannot. One cost, both at once:
fit_sweep.py's 60 ms predictions on its FIT sweeps (mean cost: pitch deg + wheel mm / 2) plus fit_drop.py's falls
(mean cost per fall, the same units), over the shared inertia, the drive (b, fric, fric_w, ku_over_b), the
torque-off braking (b_off, fric_off), the sweeps' balance offset and each drop's starting angle.
(10-29: alternating the two fits oscillated, the inertia 13 <-> 36 kg cm^2; one cost with the drops on the drive's
damping and friction drove the friction to 0 -- the torque-off wheels are not the drive at u = 0.)
Prints the result for model.Params, the sweeps (and fit_sweep's CHECK sweep) and each drop.
"""
import sys
from dataclasses import replace

import numpy as np
from scipy.optimize import minimize

import fit_drop as FD
import fit_sweep as FS
from model import Params
from model_flex import FlexParams

def _arg(name):
    return float(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else None


HELD = dict(flex_k=_arg("--flex-k"), flex_c=_arg("--flex-c"))     # a measured spring (tools/balance/ring_down.py)
FLEX = "--flex" in sys.argv or HELD["flex_k"] is not None
MOTOR = ["b", "fric", "fric_w", "ku_over_b", "b_off", "fric_off"] + \
        ([n for n in ("flex_k", "flex_c") if HELD[n] is None] if FLEX else [])   # all on a log scale
N = 1 + len(MOTOR)                                                  # then th_off, then each drop's start
FLEX_START = {n: (HELD[n] if HELD[n] is not None else v) for n, v in dict(flex_k=10.0, flex_c=0.02).items()}


def unpack(z, n_drops, base):
    p = replace(base, ib_axle=np.exp(z[0]), **{n: np.exp(v) for n, v in zip(MOTOR, z[1:N])})
    return p, np.radians(z[N]), z[N + 1:N + 1 + n_drops]


def main():
    fit = [FS.windows(f) for f in FS.FIT]
    check = [FS.windows(f) for f in FS.CHECK]
    drops = FD.falls(FD.REC)
    base = FlexParams(**FLEX_START) if FLEX else Params()

    def cost(z):
        p, th_off, dz = unpack(z, len(drops), base)
        c_sw = np.mean(FS.score(p, th_off, fit)[:, 2])
        c_dr = FD.cost(np.r_[z[0], dz], drops, False, False, p) / len(drops)
        return c_sw + c_dr if np.isfinite(c_sw + c_dr) else 1e3

    z0 = np.r_[np.log(base.ib_axle), [np.log(getattr(base, n)) for n in MOTOR], FS.TH_OFF0, np.zeros(len(drops))]
    print(f"start: cost {cost(z0):.3f}", flush=True)
    res = minimize(cost, z0, method="Nelder-Mead", options=dict(xatol=1e-4, fatol=1e-5, maxiter=6000, adaptive=True))
    p, th_off, dz = unpack(res.x, len(drops), base)
    print(f"fitted ({res.nfev} tries, cost {res.fun:.3f}):")
    for n in ["ib_axle"] + MOTOR:
        print(f"   {n} = {getattr(p, n):.4g}")
    if FLEX:
        print(f"   (the body alone on its legs rings at {np.sqrt(p.flex_k / p.i_body) / (2 * np.pi):.1f} Hz)")
    FS.report("sweeps", p, th_off, fit, check)
    print("drops:")
    FD.cost(np.r_[res.x[0], dz], drops, False, True, p)


if __name__ == "__main__":
    main()
