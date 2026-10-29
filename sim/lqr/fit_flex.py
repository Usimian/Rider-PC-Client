"""Fit the body-on-legs spring (model_flex.py) to the 1-30 Hz floor sweep (2026-10-28). PARKED: see model_flex.py.

    python3 sim/lqr/fit_flex.py                 # a coarse grid of stiffness x damping, then a refinement

The robot: data/captures/2026-10-28_chirp_lqr_30hz.csv (LQR standing, +/-30, 1 -> 30 Hz over 20 s, balance point
2.8 deg). The model: closed_loop.py with model_flex.FlexParams running the same sweep. Cost over the 1/3-octave
bands 5-25 Hz (freq_response.bands), for the pitch and the LQR's reply: (log of the robot/model gain ratio)^2 +
(phase difference / 45 deg)^2, weighted by the robot's coherence. The rigid model scores ~gain ratio 1.3 -> 4.1 and
~0 -> 54 deg of extra lag across those bands.
"""
import os
import sys
from dataclasses import replace

import numpy as np
from scipy.optimize import minimize

from closed_loop import Firmware, run
from freq_response import DATA, bands, bands_for, from_rows, load
from fw_timing import FW_2026_10_19          # the sweep it fits ran on that firmware
from model import Params
from model_flex import FlexParams

REC = "2026-10-28_chirp_lqr_30hz.csv"
SWEEP = (30.0, 1.0, 30.0, 20.0)
UP = 2.8
F_LO, F_HI = 5.0, 25.0
EDGES = bands_for(SWEEP[1], SWEEP[2])
ROBOT = bands(load(os.path.join(DATA, REC)), EDGES)


def model_bands(p):
    rows, fell = run(p, Firmware(up_deg=UP), T=SWEEP[3] + 4.0, sweep=SWEEP, sweep_at=3.0, timing=FW_2026_10_19)
    return None if fell else bands(from_rows(rows), EDGES)


def cost(mb, verbose=False):
    if mb is None:
        return 1e3
    c = 0.0
    for q in ("pitch", "reply"):
        for (f, hr, coh), (_, hm, _) in zip(ROBOT[q], mb[q]):
            if F_LO <= f <= F_HI:
                g = np.log(abs(hr) / abs(hm))
                ph = np.angle(hr / hm)
                c += coh * (g ** 2 + (ph / np.radians(45)) ** 2)
                if verbose:
                    print(f"      {q:5s} {f:4.1f} Hz  robot/model {abs(hr) / abs(hm):4.2f}, robot lags {np.degrees(-ph):+4.0f} deg")
    return c


def main():
    base = FlexParams()
    print(f"rigid model: cost {cost(model_bands(Params())):.2f}")
    grid = []
    for k in (8.0, 15.0, 30.0, 60.0, 120.0):
        for cdamp in (0.005, 0.02, 0.06):
            J = cost(model_bands(replace(base, flex_k=k, flex_c=cdamp)))
            grid.append((J, k, cdamp))
            print(f"k {k:6.1f} N m/rad  c {cdamp:.3f} N m s/rad: cost {J:7.2f}", flush=True)
    J0, k0, c0 = min(grid)
    res = minimize(lambda z: cost(model_bands(replace(base, flex_k=np.exp(z[0]), flex_c=np.exp(z[1])))),
                   [np.log(k0), np.log(c0)], method="Nelder-Mead", options=dict(xatol=0.02, fatol=0.01, maxiter=40))
    k, cd = np.exp(res.x)
    p = replace(base, flex_k=k, flex_c=cd)
    print(f"\nfitted: k {k:.1f} N m/rad, c {cd:.4f} N m s/rad, cost {res.fun:.2f} "
          f"(body alone on fixed legs: {np.sqrt(k / p.i_body) / (2 * np.pi):.1f} Hz)")
    cost(model_bands(p), verbose=True)


if __name__ == "__main__":
    main()
