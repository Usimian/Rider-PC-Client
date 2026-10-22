"""Policy position-loop settings sweep (2026-10-22): can firmer setpoint-bias gains make the policy stop like the LQR?

The policy is a pure balancer; how hard it brakes after a stick release is set by the firmware's position loop,
biasDeg = clamp(K*posErr + KI*int + KD*(xvelF - vdes), +/-POSMAX) (gPosHoldK / gDriveKd / gPosHoldMax). With the
deployed K 20, KD 10, POSMAX 8 the sim (stop_check.py) has ppo_lb25_s4 travel 28 cm after a release vs the LQR's
8 cm (floor 18-22 vs 7.5). This sweeps K, KD and POSMAX and reports, per setting, both directions x seeds:
  stop     travel (cm) in the 3 s after a release from full speed, and the pitch swing in the next 1.5 s
  flip     pitch swing after a full flip (0.15 s in the dead zone)
  cruise   mean speed over the last second before the release (target 0.30 m/s)
  hold     pitch std (deg) and position wander (cm) over 8 s standing still
  falls    any run that fell
Run:  ./.venv/bin/python drive_gain_sweep.py ppo_lb25_s4 [--seeds 2]
"""
import argparse
import itertools
from multiprocessing import Pool
import numpy as np
import drive_sim as ds
import stop_check as sc

GRID = dict(K=[20, 40, 60, 80], KD=[10, 15, 20], POSMAX=[8.0, 12.0])


def run_setting(args):
    name, k, kd, pmax, seeds = args
    ds.K, ds.KD, ds.POSMAX_DEG = float(k), float(kd), float(pmax)
    infer, kw0 = ds.load_policy(name), dict(drive="lead")
    tr, sw_r, sw_f, cruise, falls = [], [], [], [], 0
    for d in (+1, -1):
        for s in range(seeds):
            f1, s1, t1 = sc.one(infer, kw0, d, s, release_at=sc.T_EVENT)
            f2, s2, _ = sc.one(infer, kw0, d, s, reverse_at=sc.T_EVENT, release_s=0.15)
            L, _, _ = ds.simulate(infer, d, ds.VMAX, T=sc.T_EVENT, seed=s, **kw0)
            w = L[:, 0] > sc.T_EVENT - 1.0
            cruise.append(abs(float(np.mean(L[w, 4]))))
            falls += f1 + f2
            tr.append(t1); sw_r.append(s1); sw_f.append(s2)
    hold_std, wander = [], []
    for s in range(seeds):
        L, fell, _ = ds.simulate(infer, +1, 0.0, T=8.0, seed=s, **kw0)
        falls += fell is not None
        w = L[:, 0] > 2.0
        hold_std.append(float(np.std(np.degrees(L[w, 1]))))
        wander.append(float(np.max(np.abs(L[w, 6]))) * 100.0)
    return (k, kd, pmax, np.median(tr), np.median(sw_r), np.median(sw_f), np.mean(cruise),
            np.mean(hold_std), np.max(wander), falls, 5 * seeds)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("--seeds", type=int, default=2)
    a = ap.parse_args()
    jobs = [(a.policy, k, kd, pm, a.seeds) for k, kd, pm in itertools.product(*GRID.values())]
    with Pool(min(len(jobs), 20)) as pool:
        res = pool.map(run_setting, jobs)
    print(f"{a.policy}   (deployed: K 20 KD 10 POSMAX 8)")
    print(f"{'K':>4s} {'KD':>4s} {'max':>5s} | {'stop cm':>7s} {'stop sw':>7s} {'flip sw':>7s} | {'cruise':>6s} |"
          f" {'hold std':>8s} {'wander cm':>9s} | falls")
    for r in sorted(res, key=lambda r: r[3]):
        print(f"{r[0]:4.0f} {r[1]:4.0f} {r[2]:5.0f} | {r[3]:7.1f} {r[4]:6.1f}° {r[5]:6.1f}° | {r[6]:6.3f} |"
              f" {r[7]:7.2f}° {r[8]:9.1f} | {r[9]}/{r[10]}")


if __name__ == "__main__":
    main()
