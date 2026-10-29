"""The plant's transfer function back-calculated from the driving sweeps, without assuming the mechanics (2026-10-29).

    python3 sim/lqr/fit_tf.py               # the robot's driving sweeps
    python3 sim/lqr/fit_tf.py --simulated   # the same sweeps on the model (its own delays) -- the difference is
                                            # what the model is missing

The driving sweeps (full stick +/-0.30 m/s, the wheels never stop: the robot is linear there) give the plant's lean
per unit of command per band (plant_frf.py: the known sweep cancels the LQR). The model made linear at 0.30 m/s
(fit_frf.linear) says what shape that transfer function has -- its poles, zeros and gain. Here the same shape, with
every pole, zero, the gain and a pure delay free, is fitted to the robot's bands (coherence >= COH; the wheel channel
is swamped by the drive's own motion while driving, so the lean channel only). Printed side by side: each pole and
zero (1/s; + = unstable), the model's and the robot's -- what the physical model has to reproduce.
The sweeps ran 1-10 Hz, so the good bands stop at ~7 Hz: poles far above that are poorly determined.
"""
import os

import numpy as np
from scipy.optimize import minimize
from scipy.signal import ss2tf

import fit_frf as FF
import freq_response as FR
import plant_frf as PF
from model import Params

COH = 0.6
V = 0.30


def model_tf(p):
    """Lean (deg, firmware frame) per unit of command at V: zeros, poles, gain (scipy convention)."""
    A, B, c_enc, c_body, _ = FF.linear(p, V)
    num, den = ss2tf(A, B, -c_body[None, :] * 180.0 / np.pi, np.zeros((1, 1)))
    num = np.trim_zeros(np.round(num[0], 12), "f")
    return np.roots(num), np.roots(den), num[0] / den[0]


def frf(zeros, poles, k, delay, f):
    s = 2j * np.pi * np.asarray(f)
    return k * np.prod([s - z for z in zeros], axis=0) / np.prod([s - q for q in poles], axis=0) * np.exp(-s * delay)


def robot_bands(simulated=False):
    """The driving sweeps' lean per command per band: the robot's recordings, or (simulated) the same sweeps run on
    the model with the flashed LQR at the robot's timing -- with every delay the simulation knows of (the cycle,
    the IMU loop's sample age, the filters), so the robot's delay beyond the simulated one is what the model lacks."""
    up = float(FR.load(os.path.join(FR.DATA, FR.SWEEPS[0][0]))["th"].mean())
    out = []
    for name, sweep, v in FR.SWEEPS:
        if v == 0.0:
            continue
        edges = FR.bands_for(sweep[1], sweep[2])
        rec = FR.from_rows(FR.model_rows(sweep, v, up)[0]) if simulated else FR.load(os.path.join(FR.DATA, name))
        rows = PF.plant(FR.bands(rec, edges))
        out += [(f, pth) for f, px, pth, cx, cth in rows if cth >= COH]
    return np.array([r[0] for r in out]), np.array([r[1] for r in out])


def main():
    p = Params()
    z0, p0, k0 = model_tf(p)
    real = lambda r: np.isreal(r) | (np.abs(np.imag(r)) < 1e-9)
    z0, p0 = np.real_if_close(z0[real(z0)]), np.real_if_close(p0[real(p0)])     # this model's are all real
    p0 = p0[np.abs(p0) > 1e-6]                         # the free integrator (position) does not reach the lean
    sim = "--simulated" in __import__("sys").argv
    f, H = robot_bands(sim)
    print(f"{'the simulated sweeps' if sim else 'robot'}: {len(f)} bands, {f.min():.1f}-{f.max():.1f} Hz "
          f"(both driving sweeps)")

    def unpack(x):
        nz, npl = len(z0), len(p0)
        return x[:nz], x[nz:nz + npl], x[nz + npl], x[nz + npl + 1]

    def cost(x):
        zz, pp, k, d = unpack(x)
        if not 0.0 <= d <= 0.03:
            return 1e6
        m = frf(zz, pp, k, d, f)
        return float(np.sum(np.abs(np.log(H / m)) ** 2))

    x0 = np.r_[np.real(z0), np.real(p0), k0, 0.002]
    print(f"the model as it stands: cost {cost(x0):.2f}")
    best = min((minimize(cost, x0 * s, method="Nelder-Mead", options=dict(maxiter=20000, xatol=1e-6, fatol=1e-8))
                for s in (1.0, 0.7, 1.4)), key=lambda r: r.fun)
    zz, pp, k, d = unpack(best.x)
    print(f"fitted: cost {best.fun:.2f}, delay {d * 1000:.1f} ms, gain x{k / k0:.2f} of the model's\n")
    print("            model         robot   (1/s; + unstable)")
    for lab, a, b in (("pole", np.sort(np.real(p0)), np.sort(pp)), ("zero", np.sort(np.real(z0)), np.sort(zz))):
        for m, r in zip(a, b):
            print(f"   {lab}  {m:+9.2f}    {r:+9.2f}   ({abs(r) / (2 * np.pi):.2f} Hz)")
    print("\nband    robot / model      robot / fitted")
    mm, ff = frf(z0, p0, k0, 0.0, f), frf(zz, pp, k, d, f)
    for i in np.argsort(f):
        print(f"  {f[i]:4.1f} Hz  x{abs(H[i] / mm[i]):.2f} {np.degrees(np.angle(H[i] / mm[i])):+4.0f}     "
              f"x{abs(H[i] / ff[i]):.2f} {np.degrees(np.angle(H[i] / ff[i])):+4.0f}")


if __name__ == "__main__":
    main()
