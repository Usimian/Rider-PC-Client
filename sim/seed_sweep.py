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

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.join(HERE, ".venv", "bin", "python")
LOGDIR = os.path.join(HERE, "logs", "seed_sweep")
RECIPE = ["--domain_rand", "--frame_stack", "2", "--drive_train", "--caps_lambda", "4", "--drive_lead", "--pitch_dis",
          "--spin_prob", "0.3"]
# two-wheel spins/turns (rider_env_twowheel, sensor noise on, turnrate 1.0 = full stick). Floor: ld8_s6 spins at
# 16-22 deg/s (two-wheel sim 5-7), LQR 2.6-3 (sim 1.4-1.9) -> a policy must spin within ~2x the sim LQR.
TURNS = [("spin+", dict(yaw_cmd=1.0)), ("spin-", dict(yaw_cmd=-1.0)), ("turn-drive", dict(yaw_cmd=1.0, v=0.30))]
SPIN_RATE_MAX = 3.5     # deg/s pitch-rate RMS while spinning in place
LEAN_MAX = 2.5          # deg: the drive loop's lean bias at full speed, clean plant. Sim: ld8_s6 1.3-1.4 (floor: drives well),
                        # tw2_s6 4.9-5.0 (floor 2026-10-20: rocked itself over at full speed, though the sim kept it calm)
FLOOR_SPINS = "floor_spins_ld8s6_2026-10-20.csv"   # ld8_s6 spinning on the floor (see floor_spin_gain)
LEAD4_MIN = 25.0        # deg: its 4 Hz response must LEAD ld8_s6's by this much (see floor_spin_response: +34/+76 halved
                        # the floor shimmy, +4/+10 did not)
DRIVE = "lead"          # the stick-drive structure the screen runs (must match the recipe: --drive_lead -> 'lead')


def parse_seeds(s):
    if "-" in s:
        a, b = s.split("-"); return list(range(int(a), int(b) + 1))
    return [int(x) for x in s.split(",")]


def train(name, seed, steps, threads, add=()):
    env = dict(os.environ, OMP_NUM_THREADS=str(threads), MKL_NUM_THREADS=str(threads))
    log = open(os.path.join(LOGDIR, name + ".log"), "w")
    cmd = [PY, "train.py", "--steps", str(steps), "--seed", str(seed), "--out", name] + RECIPE + list(add)
    log.write("# " + " ".join(cmd) + "\n"); log.flush()
    return subprocess.Popen(cmd, cwd=HERE, stdout=log, stderr=subprocess.STDOUT, env=env)


def export(name):
    r = subprocess.run([PY, "export_policy.py", name, "2"], cwd=HERE, capture_output=True, text=True)
    ok = "OK (portable)" in r.stdout
    return ok, r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr.strip()[-200:]


def screen(name):
    """Offline screen, all with the MEASURED accelerometer coupling and the recipe's stick drive (DRIVE):
      * clean loaded wheel (nominal), loaded STICK-SLIP, and a WEAK wheel (x0.85, a low battery): full stick
        FWD/REV at 0.30 -- s8/s9's floor falls
      * LOW LEGS: balance point 3 deg off (floor 2026-10-19 at the lowest legs: no reverse, forward creep/falls)
      * TURN DRIFT: a 2 deg pitch-sensor drift building mid-drive (floor 2026-10-19: sped up in turns, fell)
      * HOLD 7 s on stick-slip -- ranks the hunt (s5's ~2 Hz see-saw)
    A policy must survive every drive at a speed within 0.05 m/s of the command; 'a99' = 99th-percentile
    |output| while driving (saturation margin: s9 pinned its output before it fell). The firmware LQR passes
    all of these in the same sim."""
    sys.path.insert(0, HERE)
    import numpy as np
    import policy_symmetry as ps
    import drive_sim as ds
    from dataclasses import replace
    from rider_params import DEFAULT
    m = ps.metrics(os.path.join(HERE, name + "_export.npz"))
    infer = ds.load_policy(os.path.join(HERE, name))
    ss = replace(DEFAULT, stick_slip=True)
    weak = replace(DEFAULT, vel_max_rad_s=0.85 * DEFAULT.vel_max_rad_s)
    cases = [("clean", DEFAULT, {}), ("ss", ss, {}), ("weak", weak, {}),
             ("low", DEFAULT, dict(pitch_offset_deg=-3.0)), ("drift", DEFAULT, dict(drift_deg=-2.0))]
    rows = {}
    for plant, p, kw in cases:
        for dirn, tag in ((+1, "fwd"), (-1, "rev")):
            L, fell, dt = ds.simulate(infer, dirn, ds.VMAX, T=9.0, params=p, fw_obs=True, drive=DRIVE, **kw)
            s = ds.stats(L, 1.0 / dt)
            drv = L[:, 0] > ds.HOLD_S
            a99 = float(np.percentile(np.abs(L[drv, 3]), 99)) if drv.any() else 1.0
            spd = abs(float(np.mean(L[L[:, 0] > L[-1, 0] - 3.0, 4])))
            lean = abs(float(np.mean(L[L[:, 0] > L[-1, 0] - 3.0, 5])))     # drive-loop lean bias (deg), last 3 s
            rows[plant + "-" + tag] = dict(up=fell is None, fell=fell, rate=s["rate_rms"], freq=s["freq"],
                                           prom=s["prom"], sat=s["act_sat"], pitch=s["pitch_std"],
                                           travel=abs(s["travel"]), a99=a99, spd=spd, lean=lean)
    L, fell, dt = ds.simulate(infer, +1, 0.0, params=ss, fw_obs=True, drive=DRIVE)
    k = L[:, 0] > 2.0; pr = L[k, 2]
    f, prom = ds.dom_freq(pr, 1.0 / dt, 1.0, 6.0)
    rows["hold"] = dict(up=fell is None, fell=fell, pitch=np.degrees(L[k, 1].std()),
                        rate=np.degrees(np.sqrt((pr ** 2).mean())), freq=f, prom=prom)
    for tag, kw in TURNS:
        kw = dict(kw); v = kw.pop("v", 0.0)
        L, fell, dt = ds.simulate(infer, +1, v, T=8.0, fw_obs=True, drive=DRIVE, twowheel=True, noise=True, **kw)
        k = L[:, 0] > 3.0
        pr = L[k, 2] if k.sum() > 32 else L[:, 2]
        rows[tag] = dict(up=fell is None, fell=fell, rate=np.degrees(np.sqrt((pr ** 2).mean())))
    m["floor4"], m["lead4"] = floor_spin_response(infer)
    return m, rows


def floor_spin_response(infer):
    """FLOOR-REPLAY check (2026-10-20): ld8_s6's floor spins (FLOOR_SPINS, its policy inputs at loop rate) fed to the
    candidate, open loop; returns its response to the 4 Hz shimmy relative to ld8_s6's own recorded output (pitch-rate
    -> output cross-spectrum, 3.5-4.8 Hz) as (gain, phase lead in degrees). The sim's own spin number did not predict
    the floor; the PHASE LEAD does, on four floor results (floor spin pitch-rate RMS; ld8_s6 16-22 deg/s):
      tt25_s4 0.96x +4 deg -> 21     sa5_s1 0.68x +10 deg -> 17     da5_s0 0.62x +34 deg -> 8.9     tw2_s6 0.76x +76 -> 8.5
    The gain alone does not (sa5_s1). Reacting earlier is what breaks a 4 Hz oscillation born of loop delay."""
    import csv
    import numpy as np
    eps = {}
    for r in csv.DictReader(open(os.path.join(HERE, FLOOR_SPINS))):
        eps.setdefault(r["spin"], []).append(r)
    gains = []
    for rs in eps.values():
        g = lambda k, s=1.0: np.array([float(r[k]) for r in rs]) * s
        pp, pr = np.radians(g("pol_pitch_x100", 0.01)), np.radians(g("pol_rate_x10", 0.1))
        xv, ip = g("pol_xvel_mm", 1e-3), g("pol_int_m", 1e-3)
        cols = [pp, pr, 0 * pp, xv, xv / 0.03, ip, 0 * pp]
        if getattr(infer, "turn_obs", False):
            cols.append(np.minimum(2.0, np.abs(g("yrc_x1000", 1e-3))))
        fr = np.stack(cols, 1)
        u1 = np.array([infer(np.concatenate([fr[i - 1], fr[i]])) * 30 * 7.764 for i in range(1, len(fr))])
        u0, pr = g("cmd")[1:], pr[1:]
        f = np.fft.rfftfreq(len(u0), np.median(np.diff(g("t_us", 1e-6))))
        P, U0, U1 = (np.fft.rfft((x - x.mean()) * np.hanning(len(x))) for x in (pr, u0, u1))
        k = (f >= 3.5) & (f <= 4.8)
        gains.append(np.sum(np.conj(P[k]) * U1[k]) / np.sum(np.conj(P[k]) * U0[k]))
    h = np.mean(gains)
    return float(abs(h)), float(np.degrees(np.angle(h)))


def verdict(m, rows):
    reasons = []
    turns = [t for t, _ in TURNS]
    drives = [k for k in rows if k != "hold" and k not in turns]
    falls = [k for k in rows if k != "hold" and not rows[k]["up"]]
    if falls: reasons.append("falls:" + "/".join(falls))
    for t in ("spin+", "spin-"):
        if rows[t]["up"] and rows[t]["rate"] > SPIN_RATE_MAX: reasons.append(t + "-shimmy")
    if not rows["hold"]["up"]: reasons.append("hold-falls")
    if m["lead4"] < LEAD4_MIN: reasons.append("floor-shimmy")
    if abs(m["a_op"]) > 0.5: reasons.append("a_op")
    if m["gRatio"] > 2.0: reasons.append("gain-asym")
    if m["sat"] > 0.10: reasons.append("clipped")
    for t in drives:
        r = rows[t]
        if not r["up"]:
            continue
        if r["rate"] > 60.0: reasons.append(t + "-rough")
        if 6.0 <= r["freq"] <= 12.0 and r["prom"] > 8: reasons.append(t + "-ring")
        if abs(r["spd"] - 0.30) > 0.05: reasons.append(t + "-speed")    # LQR holds the command within 0.01
        if t in ("clean-fwd", "clean-rev") and r["lean"] > LEAN_MAX: reasons.append(t + "-lean")
        if t in ("clean-fwd", "clean-rev", "ss-fwd", "ss-rev") and r["a99"] > 0.90: reasons.append(t + "-nomargin")
    h = rows["hold"]
    if h["up"] and h["prom"] > 4.5 and h["freq"] < 3.5: reasons.append("hold-hunt")   # s5-style 2 Hz see-saw
    return "PASS" if not reasons else "fail(" + ",".join(reasons) + ")"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0-9")
    ap.add_argument("--parallel", type=int, default=5)
    ap.add_argument("--threads", type=int, default=4, help="torch/BLAS threads per run")
    ap.add_argument("--steps", type=int, default=2_500_000)
    ap.add_argument("--tag", default="fw")
    ap.add_argument("--rescreen", action="store_true", help="skip training; re-screen existing exports")
    ap.add_argument("--extra", default="", help="comma-separated extra policy names to include in --rescreen")
    ap.add_argument("--add", default="", help="extra train.py arguments for this sweep, e.g. a fine-tune: "
                                              "'--init ppo_ld8_s6 --anchor_lambda 5 --lr 1e-4'")
    args = ap.parse_args()
    os.makedirs(LOGDIR, exist_ok=True)
    seeds = parse_seeds(args.seeds)
    names = ["ppo_%s_s%d" % (args.tag, s) for s in seeds]
    summary = open(os.path.join(LOGDIR, "summary.txt"), "a")

    def say(s):
        print(s, flush=True); summary.write(s + "\n"); summary.flush()

    def report(n, mins, m, rows):
        def cell(r):
            return "%.2f %.2f %3.1f° %s" % (r["spd"], r["a99"], r["lean"], "up" if r["up"] else "F%.1f" % r["fell"])
        h = rows["hold"]
        turns = " ".join("%4.1f%s" % (rows[t]["rate"], "" if rows[t]["up"] else "F") for t, _ in TURNS) + \
            " fl4 %.2fx%+3.0f" % (m["floor4"], m["lead4"])
        say("%-14s %5.0fm | %+6.2f %5.0f %6.1f | %s | %-22s | %-16s | %s" %
            (n, mins, m["a_op"], 100 * m["sat"], m["gRatio"],
             " | ".join(cell(rows[pl + "-fwd"]) + " / " + cell(rows[pl + "-rev"])
                        for pl in ("clean", "ss", "weak", "low", "drift")),
             "%4.2f° %5.1f°/s %3.1fHz x%-3.0f" % (h["pitch"], h["rate"], h["freq"], h["prom"]),
             turns, verdict(m, rows)))

    header = "%-14s %6s | %6s %5s %6s | %s | %-22s | %-16s | %s" % \
        ("policy", "train", "a_op", "sat%", "gRatio",
         " | ".join("%-25s" % (pl + " FWD/REV spd a99") for pl in ("clean", "stick-slip", "weak x0.85", "low legs -3",
                                                                    "turn drift -2")),
         "ss HOLD std rate f x", "2W spin+/-/turn  floor4", "verdict")
    if args.rescreen:
        say("\n=== rescreen %s ===" % time.strftime("%Y-%m-%d %H:%M")); say(header)
        for n in ([x for x in args.extra.split(",") if x] + names):
            m, rows = screen(n); report(n, 0, m, rows)
        return

    say("\n=== seed sweep %s  seeds=%s  steps=%d  parallel=%d  recipe=%s %s ===" %
        (time.strftime("%Y-%m-%d %H:%M"), seeds, args.steps, args.parallel, " ".join(RECIPE), args.add))
    say(header)
    for i in range(0, len(names), args.parallel):
        batch = list(zip(names[i:i + args.parallel], seeds[i:i + args.parallel]))
        t0 = time.time()
        procs = [(n, train(n, s, args.steps, args.threads, args.add.split())) for n, s in batch]
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
            report(n, mins, m, rows)
    say("=== sweep done %s ===" % time.strftime("%H:%M"))


if __name__ == "__main__":
    main()
