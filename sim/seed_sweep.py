#!/usr/bin/env python3
"""Seed sweep in the firmware-faithful sim, screened offline (no robot).

Trains N seeds of one recipe in parallel batches, exports each (npz + policy.h), then screens
every export with the two offline tests that predict on-robot behaviour:
  * policy_symmetry.metrics  -- conditioning around the operating point (a_op, saturation, gain ratio)
  * drive_sim               -- full-stick forward + reverse drive in the calibrated sim
                               (stays up? pitch-rate RMS, dominant frequency, saturation)
Only policies that pass the screen are worth a floor test.

Default recipe = the drive-aware one (ppo_drive34_l4's): domain rand + embedded drive loop +
CAPS lambda 4 + frame_stack 2, 2.5M steps. Logs + summary go to a FIXED path: sim/logs/seed_sweep/.

Run:  ./.venv/bin/python seed_sweep.py [--seeds 0-9] [--parallel 5] [--steps 2500000] [--tag fw]
"""
import argparse, os, subprocess, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.join(HERE, ".venv", "bin", "python")
LOGDIR = os.path.join(HERE, "logs", "seed_sweep")
RECIPE = ["--domain_rand", "--frame_stack", "2", "--drive_train", "--caps_lambda", "4"]


def parse_seeds(s):
    if "-" in s:
        a, b = s.split("-"); return list(range(int(a), int(b) + 1))
    return [int(x) for x in s.split(",")]


def train(name, seed, steps, threads):
    env = dict(os.environ, OMP_NUM_THREADS=str(threads), MKL_NUM_THREADS=str(threads))
    log = open(os.path.join(LOGDIR, name + ".log"), "w")
    cmd = [PY, "train.py", "--steps", str(steps), "--seed", str(seed), "--out", name] + RECIPE
    log.write("# " + " ".join(cmd) + "\n"); log.flush()
    return subprocess.Popen(cmd, cwd=HERE, stdout=log, stderr=subprocess.STDOUT, env=env)


def export(name):
    r = subprocess.run([PY, "export_policy.py", name, "2"], cwd=HERE, capture_output=True, text=True)
    ok = "OK (portable)" in r.stdout
    return ok, r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr.strip()[-200:]


def screen(name):
    sys.path.insert(0, HERE)
    import policy_symmetry as ps
    import drive_sim as ds
    m = ps.metrics(os.path.join(HERE, name + "_export.npz"))
    infer = ds.load_policy(os.path.join(HERE, name))
    rows = {}
    for dirn, tag in ((+1, "fwd"), (-1, "rev")):
        L, fell, dt = ds.simulate(infer, dirn, ds.VMAX, fw_obs=True)
        s = ds.stats(L, 1.0 / dt)
        rows[tag] = dict(up=fell is None, fell=fell, rate=s["rate_rms"], freq=s["freq"], prom=s["prom"],
                         sat=s["act_sat"], pitch=s["pitch_std"])
    return m, rows


def verdict(m, rows):
    reasons = []
    if not (rows["fwd"]["up"] and rows["rev"]["up"]): reasons.append("falls")
    if abs(m["a_op"]) > 0.5: reasons.append("a_op")
    if m["gRatio"] > 2.0: reasons.append("gain-asym")
    if m["sat"] > 0.10: reasons.append("clipped")
    for t in ("fwd", "rev"):
        r = rows[t]
        if r["up"] and r["rate"] > 60.0: reasons.append(t + "-rough")
        if r["up"] and 6.0 <= r["freq"] <= 12.0 and r["prom"] > 8: reasons.append(t + "-ring")
    return "PASS" if not reasons else "fail(" + ",".join(reasons) + ")"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0-9")
    ap.add_argument("--parallel", type=int, default=5)
    ap.add_argument("--threads", type=int, default=4, help="torch/BLAS threads per run")
    ap.add_argument("--steps", type=int, default=2_500_000)
    ap.add_argument("--tag", default="fw")
    args = ap.parse_args()
    os.makedirs(LOGDIR, exist_ok=True)
    seeds = parse_seeds(args.seeds)
    names = ["ppo_%s_s%d" % (args.tag, s) for s in seeds]
    summary = open(os.path.join(LOGDIR, "summary.txt"), "a")

    def say(s):
        print(s, flush=True); summary.write(s + "\n"); summary.flush()

    say("\n=== seed sweep %s  seeds=%s  steps=%d  parallel=%d  recipe=%s ===" %
        (time.strftime("%Y-%m-%d %H:%M"), seeds, args.steps, args.parallel, " ".join(RECIPE)))
    say("%-14s %6s | %6s %5s %6s | %-28s | %-28s | %s" %
        ("policy", "train", "a_op", "sat%", "gRatio", "FWD  rateRMS  freq  sat  up", "REV  rateRMS  freq  sat  up", "verdict"))
    for i in range(0, len(names), args.parallel):
        batch = list(zip(names[i:i + args.parallel], seeds[i:i + args.parallel]))
        t0 = time.time()
        procs = [(n, train(n, s, args.steps, args.threads)) for n, s in batch]
        for n, p in procs:
            p.wait()
        mins = (time.time() - t0) / 60.0
        for n, p in procs:
            if p.returncode != 0:
                say("%-14s FAILED (rc=%d) see %s" % (n, p.returncode, os.path.join(LOGDIR, n + ".log"))); continue
            ok, last = export(n)
            if not ok:
                say("%-14s export MISMATCH: %s" % (n, last)); continue
            m, rows = screen(n)
            f, r = rows["fwd"], rows["rev"]
            say("%-14s %5.0fm | %+6.2f %5.0f %6.1f | %6.1f°/s %5.1fHz x%-3.0f %3.0f%% %-4s | %6.1f°/s %5.1fHz x%-3.0f %3.0f%% %-4s | %s" %
                (n, mins, m["a_op"], 100 * m["sat"], m["gRatio"],
                 f["rate"], f["freq"], f["prom"], f["sat"], "up" if f["up"] else "%.1fs" % f["fell"],
                 r["rate"], r["freq"], r["prom"], r["sat"], "up" if r["up"] else "%.1fs" % r["fell"],
                 verdict(m, rows)))
    say("=== sweep done %s ===" % time.strftime("%H:%M"))


if __name__ == "__main__":
    main()
