#!/usr/bin/env python3
"""TEST #1: replay the bench command ladder through the sim ActuatorModel and
overlay against measured wheel speed (sim/actuator_bench_lqr.csv).

Both curves are normalized to fraction-of-max (the speed at the max command =
1.0) so we compare SHAPE -- where the wheel breaks away, and whether the middle
is linear -- without needing the raw->rad/s constant. This validates the static
command->speed map, which is exactly what the stiction change touches.

    sim/.venv/bin/python sim/replay_actuator.py
"""
import csv, os, numpy as np
from collections import defaultdict
from rider_params import DEFAULT
from rider_env import ActuatorModel

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "actuator_bench_lqr.csv")
CMD_MAX = 233.0          # action=1.0 (cmd 233 -> vel_max)

# ---- load + reduce to steady-state per command (positive rungs, drop bogus cmd=0) ----
by_cmd = defaultdict(list)
with open(CSV) as f:
    for r in csv.DictReader(f):
        c, i, v = int(r["cmd"]), int(r["i"]), int(r["vel"])
        if c > 0 and i >= 80:
            by_cmd[c].append(v)
cmds = sorted(by_cmd)
meas = np.array([np.mean(by_cmd[c]) for c in cmds])
a_meas = np.array(cmds) / CMD_MAX
meas_frac = meas / meas[-1]            # normalize by speed at the max command

# ---- run the model to steady state for the same actions ----
def model_steady(action, n=400):
    am = ActuatorModel(DEFAULT, 1.0 / DEFAULT.control_hz)
    am.reset()
    out = 0.0
    for _ in range(n):
        out = am(action)
    return out
mdl = np.array([model_steady(a) for a in a_meas])
mdl_frac = mdl / mdl[-1] if mdl[-1] != 0 else mdl

# ---- a simple deadband+linear fit to the measurement (for comparison) ----
lin = a_meas >= 0.25                    # clearly-linear region
A = np.vstack([a_meas[lin], np.ones(lin.sum())]).T
slope, b = np.linalg.lstsq(A, meas_frac[lin], rcond=None)[0]
x_intercept = -b / slope                # action where the fitted line hits 0

print("breakaway (data, fitted x-intercept): action ~ %.3f  (cmd ~ %.0f)" % (x_intercept, x_intercept * CMD_MAX))
print("model deadband_frac=%.3f (deadband+linear, no creep-jump)\n" % DEFAULT.deadband_frac)
print("%4s %6s  %8s %8s %8s" % ("cmd", "act", "meas", "model", "d(m-d)"))
for c, a, mf, df in zip(cmds, a_meas, meas_frac, mdl_frac):
    print("%4d %6.3f  %8.3f %8.3f %+8.3f" % (c, a, mf, df, df - mf))
rms = float(np.sqrt(np.mean((mdl_frac - meas_frac) ** 2)))
print("\nmodel-vs-measured RMS (normalized): %.4f" % rms)

# ---- plot ----
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    aa = np.linspace(0, 1, 400)
    mm = np.array([model_steady(a) for a in aa]); mm = mm / mm[-1]
    plt.figure(figsize=(7, 5))
    plt.plot(a_meas, meas_frac, "ko-", label="measured (bench, free wheel)", ms=5)
    plt.plot(aa, mm, "r-", label="sim ActuatorModel (as built)")
    plt.plot(aa, np.clip(slope * aa + b, 0, None), "b--", lw=1, label="deadband+linear fit to data")
    plt.axvline(DEFAULT.deadband_frac, color="r", ls=":", alpha=.5, label="model breakaway 0.18")
    plt.axvline(x_intercept, color="k", ls=":", alpha=.5, label="data breakaway ~%.2f" % x_intercept)
    plt.xlabel("action (cmd / %.0f)" % CMD_MAX); plt.ylabel("steady speed / max")
    plt.title("Actuator fidelity: sim model vs bench (LQR fw, free wheel)")
    plt.legend(fontsize=8); plt.grid(alpha=.3); plt.tight_layout()
    out = os.path.join(HERE, "actuator_fidelity.png")
    plt.savefig(out, dpi=110)
    print("plot -> %s" % out)
except ImportError:
    print("(matplotlib not available; skipped plot)")
