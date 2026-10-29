"""The plant's own frequency response from the closed-loop sweeps, robot against model (2026-10-29).

    python3 sim/lqr/plant_frf.py [name filter ...]

The sweep r is known and independent of the robot, so dividing what it does to an output by what it does to the
total wheel command u = r + the controller's reply cancels the controller (the instrumental-variable estimate):
    P_x(f)  = H(r -> encoder travel) / H(r -> u)    mm of wheel against the body per unit of command
    P_th(f) = H(r -> pitch)          / H(r -> u)    deg of lean per unit of command
The same from the model, running the flashed LQR at the robot's timing on the same sweep (freq_response.py's
model_rows). If the robot's plant matches the model's and only the closed loop differs, the difference is on the
controller's side (IMU path, timing); if the plant itself differs, the motor or the body is missing something.
"""
import os
import sys

import numpy as np

import freq_response as FR


def plant(b):
    """P_x and P_th per band from freq_response.bands output: (f, P_x, P_th, coherence of r -> u)."""
    out = []
    for (f, hx, _), (_, hth, cth), (_, hr, cr) in zip(b["wheel"], b["pitch"], b["reply"]):
        hu = 1.0 + hr                                   # u = r + reply
        out.append((f, hx / hu, hth / hu, min(cth, cr)))
    return out


def main():
    picked = [s for s in FR.SWEEPS if not sys.argv[1:] or any(a in s[0] for a in sys.argv[1:])]
    up = float(FR.load(os.path.join(FR.DATA, FR.SWEEPS[0][0]))["th"].mean())
    for name, sweep, v in picked:
        edges = FR.bands_for(sweep[1], sweep[2])
        rob = plant(FR.bands(FR.load(os.path.join(FR.DATA, name)), edges))
        rows, fell = FR.model_rows(sweep, v, up)
        if fell:
            print(f"\n{name}: the model fell")
            continue
        mod = plant(FR.bands(FR.from_rows(rows), edges))
        print(f"\n{name} (stick {v:+.2f} m/s): the plant, robot / model -- size ratio and the robot's phase lead (deg)")
        print("     band    wheel per command     lean per command     coherence")
        for (f, rx, rth, c), (_, mx, mth, _) in zip(rob, mod):
            print(f"   {f:5.1f} Hz   x{abs(rx) / abs(mx):5.2f} {np.degrees(np.angle(rx / mx)):+5.0f}"
                  f"        x{abs(rth) / abs(mth):5.2f} {np.degrees(np.angle(rth / mth)):+5.0f}        {c:.2f}")


if __name__ == "__main__":
    main()
