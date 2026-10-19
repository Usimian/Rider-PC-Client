#!/usr/bin/env python3
"""Deeper offline check of seed-sweep finalists, with the firmware LQR as the reference (2026-10-19).

The sweep screen runs one seed per case. This runs every case over several seeds and more conditions, so a
floor-test pick rests on more than one rollout:
  * full stick at 0.25 / 0.30 / 0.35 m/s, FWD + REV, on the clean, stick-slip and weak (x0.85) wheels
  * the floor's pitch errors at 0.30: balance point +/-3 deg (leg height), turn drift +/-2 deg
  * stick REVERSALS (full forward straight into full reverse, and back) -- clean and with low legs
  * a 10 s hold on the stick-slip wheel (hunt)
Per controller: falls, worst speed error over the last 3 s (vs the command), worst 99th-pct |output|, and the
hold's pitch-rate RMS + dominant 1-6 Hz frequency. All in drive_sim, measured accelerometer coupling.

Run:  ./.venv/bin/python deep_check.py ppo_ld8_s3 ppo_ld8_s9 ... [--drive lead] [--seeds 3]
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import numpy as np
import drive_sim as ds
from rider_params import DEFAULT


def cases():
    ss = replace(DEFAULT, stick_slip=True)
    weak = replace(DEFAULT, vel_max_rad_s=0.85 * DEFAULT.vel_max_rad_s)
    out = []
    for plant, p in (("clean", DEFAULT), ("ss", ss), ("weak", weak)):
        for v in (0.25, 0.30, 0.35):
            for d in (+1, -1):
                out.append((f"{plant} {'FWD' if d > 0 else 'REV'} {v:.2f}", p, d, v, {}))
    for off in (-3.0, 3.0):
        for d in (+1, -1):
            out.append((f"legs {off:+.0f} {'FWD' if d > 0 else 'REV'}", DEFAULT, d, 0.30, dict(pitch_offset_deg=off)))
    for dr in (-2.0, 2.0):
        for d in (+1, -1):
            out.append((f"drift {dr:+.0f} {'FWD' if d > 0 else 'REV'}", DEFAULT, d, 0.30, dict(drift_deg=dr)))
    for off in (0.0, -3.0):
        for d in (+1, -1):
            out.append((f"reversal legs {off:+.0f} {'F>R' if d > 0 else 'R>F'}", DEFAULT, d, 0.30,
                        dict(pitch_offset_deg=off, reverse_at=4.5)))
    return out


def run_one(args):
    name, drive, seeds = args
    if name == "LQR":
        infer, kw0 = None, dict(controller="lqr")
    else:
        infer, kw0 = ds.load_policy(name), dict(drive=drive)
    falls, worst_err, worst_a99, fell_cases = 0, 0.0, 0.0, []
    for label, p, d, v, kw in cases():
        for seed in range(seeds):
            T = 10.0 if "reversal" in label else 9.0
            L, fell, dt = ds.simulate(infer, d, v, T=T, seed=seed, params=p, **kw0, **kw)
            drv = L[:, 0] > ds.HOLD_S
            a99 = float(np.percentile(np.abs(L[drv, 3]), 99)) if drv.any() else 1.0
            worst_a99 = max(worst_a99, a99)
            if fell is not None:
                falls += 1
                fell_cases.append(f"{label}/s{seed}")
                continue
            m = L[:, 0] > L[-1, 0] - 3.0
            final_dir = -d if "reversal" in label else d
            err = abs(float(np.mean(L[m, 4])) - final_dir * v)
            worst_err = max(worst_err, err)
    L, fell, dt = ds.simulate(infer, +1, 0.0, T=10.0, seed=0, params=replace(DEFAULT, stick_slip=True), **kw0)
    k = L[:, 0] > 2.0
    f, prom = ds.dom_freq(L[k, 2], 1.0 / dt, 1.0, 6.0)
    hold = f"{'up' if fell is None else 'FELL'} {np.degrees(np.sqrt((L[k, 2] ** 2).mean())):4.1f} deg/s {f:3.1f}Hz x{prom:<3.0f}"
    n = len(cases()) * seeds
    return name, n, falls, worst_err, worst_a99, hold, fell_cases


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policies", nargs="+")
    ap.add_argument("--drive", default="lead")
    ap.add_argument("--seeds", type=int, default=3)
    a = ap.parse_args()
    names = ["LQR"] + a.policies
    print(f"{'controller':14s} {'runs':>5s} {'falls':>5s} {'worst speed err':>16s} {'worst a99':>10s}   hold (stick-slip, 10 s)")
    with ProcessPoolExecutor(max_workers=len(names)) as ex:
        for name, n, falls, err, a99, hold, fc in ex.map(run_one, [(nm, a.drive, a.seeds) for nm in names]):
            print(f"{name:14s} {n:5d} {falls:5d} {err:13.3f} m/s {a99:10.2f}   {hold}"
                  + (f"   FELL: {', '.join(fc[:6])}{' ...' if len(fc) > 6 else ''}" if fc else ""), flush=True)


if __name__ == "__main__":
    main()
