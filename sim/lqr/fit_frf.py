"""Fit the model to the plant's measured frequency response (2026-10-29).

    python3 sim/lqr/fit_frf.py                          # the rigid model (model.py)
    python3 sim/lqr/fit_frf.py --flex-k 2.8 --flex-c 0.015   # the body sprung on its legs, the spring held

The robot's plant per band (plant_frf.py: the known sweep as the reference cancels the controller -- encoder
travel and lean per unit of wheel command) from the four 10-29 sweeps, against the model's, made linear about each
sweep's operating point: standing (upright at rest: the friction's tanh acts as damping fric / fric_w there) or
driving at the sweep's speed (the steady lean and command found first; the friction then adds no damping).
Cogging is left out (a ripple, not a response). The command reaches the motor DELAY after it is logged (fitted).
No starting state is needed -- the 60 ms-window fit (fit_sweep.py) had to assume the joint relaxed at each start.

Cost: over the bands with coherence >= COH, |log(P_robot / P_model)|^2 (size and phase together), encoder and lean.
Free: ku_over_b, b, fric, fric_w, ib_axle, the delay. Prints the fit and the per-band ratios before and after.
"""
import os
import sys
from dataclasses import replace

import numpy as np
from scipy.optimize import fsolve, minimize

import freq_response as FR
import model as M
import model_flex as MF
import plant_frf as PF

COH = 0.6
FREE = ["ku_over_b", "b", "fric", "fric_w", "ib_axle"]       # on a log scale, then the delay (s)
BOUND = 3.0                                                  # each within this factor of model.Params' value


def _arg(name):
    return float(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else None


def base_params():
    k, c = _arg("--flex-k"), _arg("--flex-c")
    if k is None:
        return M.Params(cog20_rate=0.0, cog24_rate=0.0)
    return MF.FlexParams(flex_k=k, flex_c=c if c is not None else 0.015, cog20_rate=0.0, cog24_rate=0.0)


def operating_point(p, v):
    """The steady state at speed v (m/s): the axle at v, the bodies at rest at their steady lean, and the command
    holding it; v = 0: upright at rest."""
    m = M.plant_module(p)
    n = len(m.state(p, 0.0, 0.0))
    if v == 0.0:
        return np.zeros(n), 0.0
    flex = n == 6

    def eqs(z):
        if flex:
            s = np.array([0.0, v, z[0], 0.0, z[1], 0.0])
            d = m.deriv(p, s, z[2])
            return [d[1], d[3], d[5]]
        s = np.array([0.0, v, z[0], 0.0])
        d = m.deriv(p, s, z[1])
        return [d[1], d[3]]

    z = fsolve(eqs, [0.0, 0.0, 30.0] if flex else [0.0, 30.0])
    s = np.array([0.0, v, z[0], 0.0, z[1], 0.0]) if flex else np.array([0.0, v, z[0], 0.0])
    return s, z[-1]


def linear(p, v):
    """A, B about the operating point at speed v, and the output rows: the encoder (m) and the IMU body (rad)."""
    m = M.plant_module(p)
    s0, u0 = operating_point(p, v)
    n = len(s0)
    A, B = np.zeros((n, n)), np.zeros((n, 1))
    f0 = m.deriv(p, s0, u0)
    for j in range(n):
        h = 1e-6 * max(1.0, abs(s0[j]))
        d = np.zeros(n)
        d[j] = h
        A[:, j] = (m.deriv(p, s0 + d, u0) - m.deriv(p, s0 - d, u0)) / (2 * h)
    hu = 1e-3
    B[:, 0] = (m.deriv(p, s0, u0 + hu) - m.deriv(p, s0, u0 - hu)) / (2 * hu)
    c_enc = np.zeros(n)
    c_enc[0], c_enc[m.BODY - (2 if n == 6 else 0)] = 1.0, -p.r     # x - r * (the legs' angle: s[2] in both)
    c_body = np.zeros(n)
    c_body[m.BODY] = 1.0
    return A, B, c_enc, c_body, f0


def model_frf(p, v, freqs, delay):
    A, B, c_enc, c_body, _ = linear(p, v)
    out = []
    for f in freqs:
        w = 2j * np.pi * f
        G = np.linalg.solve(w * np.eye(A.shape[0]) - A, B[:, 0]) * np.exp(-w * delay)
        # the robot's units: encoder mm; lean deg in the firmware frame (forward lean negative)
        out.append((c_enc @ G * 1000.0, -(c_body @ G) * 180.0 / np.pi))
    return out


def robot_frfs():
    up = float(FR.load(os.path.join(FR.DATA, FR.SWEEPS[0][0]))["th"].mean())
    data = []
    for name, sweep, v in FR.SWEEPS:
        edges = FR.bands_for(sweep[1], sweep[2])
        rows = PF.plant(FR.bands(FR.load(os.path.join(FR.DATA, name)), edges))
        keep = [(f, px if cx >= COH else None, pth if cth >= COH else None) for f, px, pth, cx, cth in rows
                if cx >= COH or cth >= COH]
        data.append((name, v, keep))
    return data, up


def cost(p, delay, data):
    """Each channel's term only where its own coherence passed COH."""
    c = 0.0
    for _, v, rows in data:
        mod = model_frf(p, v, [r[0] for r in rows], delay)
        for (f, px, pth), (mx, mth) in zip(rows, mod):
            if px is not None:
                c += abs(np.log(px / mx)) ** 2
            if pth is not None:
                c += abs(np.log(pth / mth)) ** 2
    return c


def report(label, p, delay, data):
    print(f"\n{label}: cost {cost(p, delay, data):.2f}; delay {delay * 1000:.2f} ms; " +
          ", ".join(f"{n} {getattr(p, n):.4g}" for n in FREE) +
          (f", flex_k {p.flex_k:g}, flex_c {p.flex_c:g}" if hasattr(p, "flex_k") else ""))
    for name, v, rows in data:
        mod = model_frf(p, v, [r[0] for r in rows], delay)
        r = lambda a, b: "   -   " if a is None else f"x{abs(a / b):.2f}/{np.degrees(np.angle(a / b)):+.0f}"
        cells = [f"{f:4.1f}:{r(px, mx)} {r(pth, mth)}" for (f, px, pth), (mx, mth) in zip(rows, mod)]
        print(f"   {name[11:]} (robot/model: band Hz: wheel size/phase, lean size/phase): " + "  ".join(cells))


def main():
    data, _ = robot_frfs()
    p0 = base_params()
    z0 = np.r_[[np.log(getattr(p0, n)) for n in FREE], 0.002]

    def unpack(z):
        return replace(p0, **{n: np.exp(v) for n, v in zip(FREE, z[:-1])}), z[-1]

    report("as it stands", *unpack(z0), data)

    def f(z):
        p, d = unpack(z)
        if not 0.0 <= d <= 0.02 or np.any(np.abs(z[:-1] - z0[:-1]) > np.log(BOUND)):
            return 1e6                       # each parameter within a factor BOUND of the model's prior
        c = cost(p, d, data)
        return c if np.isfinite(c) else 1e6

    res = minimize(f, z0, method="Nelder-Mead", options=dict(xatol=1e-4, fatol=1e-5, maxiter=4000, adaptive=True))
    report(f"fitted ({res.nfev} tries)", *unpack(res.x), data)


if __name__ == "__main__":
    main()
