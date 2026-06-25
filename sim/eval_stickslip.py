"""Apples-to-apples hold-hunt comparison under the LOADED stick-slip actuator.

The residual hold-hunt on hardware is a loop-rate-invariant limit cycle = the wheel's
near-zero-speed stick-slip (sim/actuator_bench_lqr_loaded.csv). This script runs each saved
diff-drive policy in the env with stick_slip=True, commands pinned to ZERO (pure hold), and
measures the limit-cycle amplitude (position peak-to-peak), survival, and action chatter.

The question this answers: does a policy TRAINED on stick-slip (v5) hold stiller than one
trained on the clean deadband (v2) when both face the real loaded wheel?

  sim/.venv/bin/python eval_stickslip.py
"""
import pickle
import numpy as np
from stable_baselines3 import PPO
from rider_env_3d import RiderDiffDriveEnv

POLICIES = [
    ("v2  (clean-deadband trained)", "ppo_diffdrive_v2.zip", "ppo_diffdrive_v2_vecnorm.pkl"),
    ("v5  (stick-slip trained)",     "ppo_diffdrive_v5_stickslip.zip", "ppo_diffdrive_v5_stickslip_vecnorm.pkl"),
]
N_SEEDS = 12
FRAME_STACK = 2


def run(name, zip_path, vn_path):
    try:
        model = PPO.load(zip_path, device="cpu")
        with open(vn_path, "rb") as f:
            vn = pickle.load(f)
    except FileNotFoundError:
        print(f"  {name}: MISSING ({zip_path}) -- skipped")
        return
    env = RiderDiffDriveEnv(add_noise=False, domain_rand=False, frame_stack=FRAME_STACK, stick_slip=True)
    # pin the command to a pure hold (cmd_fwd=0, cmd_yaw=0) for the whole episode
    env._sample_cmd = lambda: (setattr(env, "cmd_fwd", 0.0), setattr(env, "cmd_yaw", 0.0))

    stood = 0; p2p = []; drift = []; chatter = []; ywander = []
    for s in range(N_SEEDS):
        obs, _ = env.reset(seed=400 + s)
        x = 0.0; xs = []; yaw = 0.0; yaws = []; prev_a = np.zeros(2); jerk = []
        term = False
        while True:
            act, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
            obs, _, term, trunc, _ = env.step(act)
            x += env._fwd_vel() * env.ctrl_dt; xs.append(x)        # integrate hold position (m)
            yaw += env.data.qvel[env.d_yaw] * env.ctrl_dt; yaws.append(yaw)
            jerk.append(float(np.mean(np.abs(act - prev_a)))); prev_a = act
            if term or trunc:
                break
        if not term:
            stood += 1
        xs = np.array(xs); yaws = np.array(yaws)
        p2p.append(100.0 * (xs.max() - xs.min()))                  # cm
        drift.append(100.0 * abs(xs[-1]))                          # cm net
        ywander.append(np.degrees(yaws.max() - yaws.min()))        # deg
        chatter.append(np.mean(jerk))

    print(f"  {name}")
    print(f"      STOOD {stood}/{N_SEEDS} | hold p2p {np.mean(p2p):5.1f} cm | net drift {np.mean(drift):5.1f} cm | "
          f"yaw wander {np.mean(ywander):5.1f} deg | action chatter {np.mean(chatter):.4f}")


if __name__ == "__main__":
    print("HOLD-HUNT under the LOADED stick-slip actuator (cmd=0, 10 s, 12 seeds):")
    for name, z, v in POLICIES:
        run(name, z, v)
