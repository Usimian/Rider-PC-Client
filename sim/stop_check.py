"""Stop and reversal check (2026-10-22): how a controller handles letting go of the stick and flipping it.

On the floor (2026-10-22, firmware releaseStop) a release from full speed (0.30 m/s) travelled 7.5 cm in the next
3 s under the LQR and 17.7-22.4 cm under ppo_lb25_s4. Training the policy on releases and flips did not change
it (eight fine-tunes, all 28-29 cm in sim): the braking is set by the firmware's position loop, which the policy
only follows -- see drive_gain_sweep.py. Full flips swung the pitch 2.6-3.4 deg (policy, ride height).

Per controller, both directions, several seeds (drive_sim, the floor's 'lead' stick drive):
  RELEASE  full stick 1.5 -> 4.5 s, then released: travel in the next 3 s (cm), pitch swing in the next 1.5 s
  FLIP     full stick, at 4.5 s 0.15 s in the dead zone, then full the other way: pitch swing in the next 1.5 s
Run:  ./.venv/bin/python stop_check.py ppo_lb25_s4 [more policies] [--seeds 5]   (LQR is always included)
"""
import argparse
import numpy as np
import drive_sim as ds

T_EVENT = 4.5


def one(infer, kw0, d, seed, **kw):
    L, fell, dt = ds.simulate(infer, d, ds.VMAX, T=T_EVENT + 3.5, seed=seed, **kw0, **kw)
    t, pitch, x = L[:, 0], np.degrees(L[:, 1]), L[:, 6]
    i0 = int(np.argmax(t >= T_EVENT))
    w = (t >= T_EVENT) & (t <= T_EVENT + 1.5)
    swing = float(np.max(np.abs(pitch[w] - pitch[i0]))) if w.any() else float("nan")
    j = int(np.argmax(t >= T_EVENT + 3.0)) if t[-1] >= T_EVENT + 3.0 else len(t) - 1
    travel = abs(float(x[j] - x[i0])) * 100.0
    return fell is not None, swing, travel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policies", nargs="*")
    ap.add_argument("--seeds", type=int, default=5)
    a = ap.parse_args()
    print(f"{'controller':14s} {'release travel cm':>18s} {'release swing':>14s} {'flip swing':>11s} {'falls':>6s}")
    for name in ["LQR"] + a.policies:
        infer, kw0 = (None, dict(controller="lqr")) if name == "LQR" else (ds.load_policy(name), dict(drive="lead"))
        rel, fl, falls = [], [], 0
        for d in (+1, -1):
            for s in range(a.seeds):
                f1, sw1, tr = one(infer, kw0, d, s, release_at=T_EVENT)
                f2, sw2, _ = one(infer, kw0, d, s, reverse_at=T_EVENT, release_s=0.15)
                falls += f1 + f2
                rel.append((tr, sw1)); fl.append(sw2)
        tr = np.array([r[0] for r in rel]); sw = np.array([r[1] for r in rel]); fl = np.array(fl)
        print(f"{name:14s} {np.median(tr):8.1f} ({tr.min():4.1f}-{tr.max():4.1f}) {np.median(sw):8.1f} deg"
              f" {np.median(fl):7.1f} deg {falls:4d}/{4 * a.seeds}")


if __name__ == "__main__":
    main()
