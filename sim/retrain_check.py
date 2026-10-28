"""Judge retrained policies against the deployed one, offline (2026-10-28, the measured-plant retrain).

    ./.venv/bin/python retrain_check.py ppo_lb25_s4 ppo_mr_a10_s0 [more] [--seeds 5]

Per policy, on the measured plant (measured_plant.py) with the position loop as the robot runs it (fw_drive.Gains):
  reversal   full stick into full stick the other way, 0.15 s through the dead zone, at 100 / 85 / 80 % motor
             strength (the floor fall at 69 % battery looked like ~85 %): falls forward-to-back and back-to-forward
  release    full stick then let go: the distance rolled in the next 3 s (cm) and the lean swing (deg)
  standing   12 s, sensor noise on: pitch sd (deg) and pitch-rate rms (deg/s) -- floor 2026-10-28 for ppo_lb25_s4:
             0.14-0.29 deg, 1.6-2.8 deg/s
and on the servo two-wheel plant (the only one that turns):
  spin       spinning in place at full turn stick: pitch-rate rms (deg/s) -- a straight-line retrain must not
             spin worse than the policy it started from
"""
import argparse
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

T_EV = 4.5


def _strength(g):
    import measured_plant as mp
    base = mp.lqr_model.Params()
    mp.measured = lambda **o: replace(base, ku_over_b=base.ku_over_b * g, **o)


def check(job):
    name, seeds = job
    import drive_sim as ds
    from rider_params import DEFAULT
    infer = ds.load_policy(name)
    M = replace(DEFAULT, plant="measured")
    out = dict(name=name)
    for g in (1.0, 0.85, 0.80):
        _strength(g)
        f = {1: 0, -1: 0}
        for d in (+1, -1):
            for s in range(seeds):
                _, fell, _ = ds.simulate(infer, d, ds.VMAX, T=T_EV + 3.0, seed=s, drive="lead", reverse_at=T_EV,
                                         release_s=0.15, params=M)
                f[d] += fell is not None
        out[f"rev{g}"] = f
    _strength(1.0)
    travel, swing, rfalls = [], [], 0
    for d in (+1, -1):
        for s in range(seeds):
            L, fell, _ = ds.simulate(infer, d, ds.VMAX, T=T_EV + 3.5, seed=s, drive="lead", release_at=T_EV, params=M)
            rfalls += fell is not None
            if fell is None:
                t, p, x = L[:, 0], np.degrees(L[:, 1]), L[:, 6]
                i0 = int(np.argmax(t >= T_EV)); j = int(np.argmax(t >= T_EV + 3.0))
                travel.append(abs(x[j] - x[i0]) * 100); swing.append(np.abs(p[(t >= T_EV) & (t <= T_EV + 1.5)] - p[i0]).max())
    out.update(travel=np.median(travel) if travel else np.nan, swing=np.median(swing) if swing else np.nan, rfalls=rfalls)
    sd, rr = [], []
    for s in range(seeds):
        L, fell, _ = ds.simulate(infer, +1, 0.0, T=12.0, seed=s, drive="lead", noise=True, params=M)
        t, p = L[:, 0], np.degrees(L[:, 1]); m = t > 4.0
        sd.append(p[m].std()); rr.append(np.std(np.gradient(p[m], t[m])))
    out.update(stand_sd=np.median(sd), stand_rate=np.median(rr))
    sp = []
    for yc in (1.0, -1.0):
        L, fell, _ = ds.simulate(infer, +1, 0.0, T=8.0, seed=0, drive="lead", twowheel=True, yaw_cmd=yc, noise=True)
        t = L[:, 0]; m = t > ds.HOLD_S + 1.0
        sp.append(np.nan if fell is not None else np.degrees(L[m, 2]).std())
    out["spin"] = np.nanmax(sp)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policies", nargs="+")
    ap.add_argument("--seeds", type=int, default=5)
    a = ap.parse_args()
    with Pool(len(a.policies)) as p:
        res = p.map(check, [(n, a.seeds) for n in a.policies])
    n = a.seeds
    print(f"{'policy':16s} | reversal falls fwd-to-back / back-to-fwd (of {n}) at motor 100 / 85 / 80 % | "
          f"release cm, swing | standing sd, rate | spin rate")
    for r in res:
        rev = "  ".join(f"{r[f'rev{g}'][1]}/{r[f'rev{g}'][-1]}" for g in (1.0, 0.85, 0.80))
        print(f"{r['name']:16s} | {rev:46s} | {r['travel']:5.1f} cm {r['swing']:4.1f} deg"
              f"{' (' + str(r['rfalls']) + ' fell)' if r['rfalls'] else ''} | {r['stand_sd']:.2f} deg {r['stand_rate']:.1f} deg/s"
              f" | {r['spin']:.1f} deg/s")


if __name__ == "__main__":
    main()
