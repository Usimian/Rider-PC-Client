"""Does the model's friction make the standing response depend on the sweep's size, as the robot's does? (2026-10-29)

    python3 sim/lqr/amplitude_check.py

The robot, standing, answers the +/-60 sweep ~2x the +/-30 one per unit of command at 1-3 Hz (plant_frf.py) -- a
linear model cannot. Here the full nonlinear model with the flashed LQR runs both standing sweeps; per band, the
plant's lean per command for the +/-60 sweep over the +/-30 one, robot and model, for a few dead zones in the drive
(model.Params.u0).
"""
import os
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

import freq_response as FR
import plant_frf as PF
from model import Params

P0 = Params()
# 2026-10-29 (robot 1.63-2.14 across 1.1-3.6 Hz): no dead zone 1.11-1.21; a sharp friction (fric_w 0.05) 0.42-0.95;
# a stiction term (break-away 2-3x, since removed from model.py) erratic 0.19-7.1; dead zone 10 units 1.29-1.71 (now
# model.Params.u0); 20-30 units stick-slip. These are the dead-zone cases:
SETTINGS = [("dead zone 0", dict(u0=0.0)),
            ("dead zone 10 units", dict(u0=10.0)),
            ("dead zone 20 units", dict(u0=20.0))]
STAND = [s for s in FR.SWEEPS if s[2] == 0.0]                 # the +/-60 (1-10 Hz) and +/-30 (1-30 Hz) sweeps
BANDS = (1.1, 1.4, 1.8, 2.2, 2.8, 3.6)


def lean(rows):
    return {round(f, 1): pth for f, px, pth, cx, cth in rows}


def model_lean(job):
    label, over = job
    up = float(FR.load(os.path.join(FR.DATA, STAND[0][0]))["th"].mean())
    out = []
    for name, sweep, v in STAND:
        rows, fell = FR.model_rows(sweep, v, up, replace(P0, **over))
        out.append(None if fell else lean(PF.plant(FR.bands(FR.from_rows(rows), FR.bands_for(sweep[1], sweep[2])))))
    return label, out


def main():
    rob = [lean(PF.plant(FR.bands(FR.load(os.path.join(FR.DATA, n)), FR.bands_for(s[1], s[2])))) for n, s, v in STAND]
    with Pool(len(SETTINGS)) as pool:
        res = pool.map(model_lean, SETTINGS)
    print(f"lean per command, the +/-60 sweep / the +/-30 sweep, by band ({', '.join(f'{b} Hz' for b in BANDS)}):")
    print(f"   {'robot':22s} " + "  ".join(f"{abs(rob[0][b] / rob[1][b]):5.2f}" for b in BANDS))
    for label, (m60, m30) in res:
        if m60 is None or m30 is None:
            print(f"   {label:22s} FELL")
            continue
        print(f"   {label:22s} " + "  ".join(f"{abs(m60[b] / m30[b]):5.2f}" for b in BANDS) +
              "   | vs the robot, +/-60: " + " ".join(f"x{abs(rob[0][b] / m60[b]):.2f}" for b in BANDS[:3]) +
              "  +/-30: " + " ".join(f"x{abs(rob[1][b] / m30[b]):.2f}" for b in BANDS[:3]))


if __name__ == "__main__":
    main()
