#!/usr/bin/env python3
"""Head-to-head hold comparison: old vs new policy on the SAME corrected sim.

Both policies are pure_balance (they don't SEE position), so both drift as a
random walk -- what differs is HOW MUCH they drift and how much they CHATTER the
wheel command. The hold-hunt limit cycle shows up as action chatter: fine
command dither the policy makes trying to correct sub-breakaway, which the real
wheel can't execute. A stiction-aware policy should chatter LESS.

Run on the SAME nominal corrected sim (DR off, sensor noise on) for a fair A/B:
    sim/.venv/bin/python sim/eval_compare.py
"""
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from rider_env import RiderBalanceEnv

FS = 2
SECONDS = 30.0
N_SEEDS = 20
SETTLE_S = 3.0           # skip initial settling before measuring the hold
# (name, zip_base, pure_balance): pure_balance=False -> position-aware (policy SEES x_err and holds).
# Each policy is evaluated in the env it was TRAINED for, so it behaves as it would deployed.
POLICIES = [("old  mc_l4a (deployed)", "ppo_mc_l4a", True),
            ("pw0.5 seed a", "ppo_posaware_pw05", False),
            ("pw0.5 seed b", "ppo_posaware_pw05b", False),
            ("pw0.5 seed c", "ppo_posaware_pw05c", False),
            ("pw0.5 seed d", "ppo_posaware_pw05d", False)]


def run(name, zip_base, pure_balance=True):
    model = PPO.load(zip_base, device="cpu")
    vn = VecNormalize.load(zip_base + "_vecnorm.pkl",
                           DummyVecEnv([lambda: RiderBalanceEnv(frame_stack=FS, pure_balance=pure_balance)]))
    drift_std, drift_max, chatter, pitch_std, stood = [], [], [], [], 0
    pos_trace = None
    for s in range(N_SEEDS):
        env = RiderBalanceEnv(add_noise=True, domain_rand=False, mirror_aug=False,
                              frame_stack=FS, pure_balance=pure_balance, max_seconds=SECONDS)
        obs, _ = env.reset(seed=200 + s)
        settle = int(SETTLE_S / env.ctrl_dt)
        xs, pits, acts, k = [], [], [], 0
        prev_a = None
        term = False
        while True:
            a, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
            obs, _, term, trunc, _ = env.step(a)
            af = float(np.asarray(a).flatten()[0])
            if k >= settle:
                xs.append(env.data.qpos[env.q_x])
                pits.append(env.data.qpos[env.q_pitch])
                if prev_a is not None:
                    acts.append(abs(af - prev_a))
            prev_a = af
            k += 1
            if term or trunc:
                break
        if not term and len(xs) > 1:                 # survived -> a real hold to measure
            stood += 1
            xs = np.array(xs)
            drift_std.append(np.std(xs))
            drift_max.append(np.max(xs) - np.min(xs))
            chatter.append(np.mean(acts))
            pitch_std.append(np.std(pits) * 180 / np.pi)
            if s == 0:
                pos_trace = xs
        env.close()
    return dict(name=name, stood=stood, n=N_SEEDS,
                drift_std=np.mean(drift_std) if drift_std else float("nan"),
                drift_max=np.mean(drift_max) if drift_max else float("nan"),
                chatter=np.mean(chatter) if chatter else float("nan"),
                pitch_std=np.mean(pitch_std) if pitch_std else float("nan"),
                trace=pos_trace)


results = [run(n, z, pb) for n, z, pb in POLICIES]
print("\n%-26s %6s %10s %10s %10s %10s" %
      ("policy", "stood", "drift_std", "drift_pp", "chatter", "pitch_std"))
print("%-26s %6s %10s %10s %10s %10s" %
      ("", "/%d" % N_SEEDS, "(m)", "(m)", "(|da|)", "(deg)"))
for r in results:
    print("%-26s %4d/%-2d %10.4f %10.4f %10.5f %10.4f" %
          (r["name"], r["stood"], r["n"], r["drift_std"], r["drift_max"],
           r["chatter"], r["pitch_std"]))

base = results[0]
for r in results[1:]:
    print("\n%s vs old:" % r["name"].strip())
    print("  chatter    %+.1f%%" % (100 * (r["chatter"] - base["chatter"]) / base["chatter"]))
    print("  hold-drift %+.1f%%" % (100 * (r["drift_std"] - base["drift_std"]) / base["drift_std"]))

try:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    plt.figure(figsize=(8, 4))
    for r in results:
        if r["trace"] is not None:
            t = np.arange(len(r["trace"])) * (SECONDS / len(r["trace"]))
            plt.plot(t, r["trace"] * 1000, label=r["name"])
    plt.xlabel("time into hold (s)"); plt.ylabel("position (mm)")
    plt.title("Hold drift, seed 0 (same corrected sim)"); plt.legend(fontsize=8); plt.grid(alpha=.3)
    plt.tight_layout(); plt.savefig("eval_compare.png", dpi=110)
    print("plot -> eval_compare.png")
except Exception as e:
    print("(no plot:", e, ")")
