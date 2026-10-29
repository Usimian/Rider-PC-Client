"""How policies ride the full forward-to-backward reversal (2026-10-29: only the anchor-free measured-plant retrains
survived it at 85 % motor -- what do they do differently?).

    ./.venv/bin/python reversal_compare.py ppo_lb25_s4 ppo_mr_a0_s1 [more] [--seeds 5] [--motor 0.85 1.0]

Per policy and motor strength, the stop_check flip (full stick forward, 0.15 s through the dead zone, full stick
backward) on the measured plant with the robot's position loop, medians over the tries that stayed up (falls
counted): the peak backward speed, the hardest backward acceleration (speed smoothed 50 ms), the time from the
flip to 0.25 m/s backward, the peak backward lean (deg past the lean at the flip), and the share of the 2 s after
the flip with the policy's output at its +/-1 limit.
"""
import argparse
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

T_EV = 4.5


def run(job):
    name, g, seeds = job
    import drive_sim as ds
    import measured_plant as mp
    from rider_params import DEFAULT
    base = mp.lqr_model.Params()
    mp.measured = lambda **o: replace(base, ku_over_b=base.ku_over_b * g, **o)
    infer = ds.load_policy(name)
    rows, falls = [], 0
    for s in range(seeds):
        L, fell, dt = ds.simulate(infer, +1, ds.VMAX, T=T_EV + 3.0, seed=s, drive="lead", reverse_at=T_EV,
                                  release_s=0.15, params=replace(DEFAULT, plant="measured"))
        if fell is not None:
            falls += 1
            continue
        t, p, a, v = L[:, 0], np.degrees(L[:, 1]), L[:, 3], L[:, 4]
        m = (t > T_EV) & (t < T_EV + 2.0)
        k = max(1, int(round(0.05 / dt)))
        vs = np.convolve(v, np.ones(k) / k, mode="same")
        acc = np.gradient(vs, t)
        i0 = int(np.argmax(t >= T_EV))
        k25 = np.nonzero(m & (v < -0.25))[0]
        rows.append((-v[m].min(), -acc[m].min(), (t[k25[0]] - T_EV) if len(k25) else np.nan,
                     p[i0] - p[m].min(),                      # backward lean is negative (sim frame: + forward)
                     np.mean(np.abs(a[m]) > 0.99) * 100))
    med = np.nanmedian(np.array(rows), axis=0) if rows else [np.nan] * 5
    return name, g, falls, seeds, med


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policies", nargs="+")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--motor", type=float, nargs="+", default=[0.85, 1.0])
    a = ap.parse_args()
    jobs = [(n, g, a.seeds) for n in a.policies for g in a.motor]
    with Pool(min(len(jobs), 12)) as p:
        res = p.map(run, jobs)
    print(f"{'policy':16s} motor | falls | peak back speed | hardest back accel | to 0.25 m/s | lean | output pinned")
    for name, g, falls, n, m in res:
        print(f"{name:16s} {g:4.2f}x | {falls}/{n}  | {m[0]:5.2f} m/s       | {m[1]:5.2f} m/s^2         | {m[2]:4.2f} s      |"
              f" {m[3]:4.1f}deg | {m[4]:4.0f} %")


if __name__ == "__main__":
    main()
