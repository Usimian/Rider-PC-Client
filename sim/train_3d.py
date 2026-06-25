"""Train the JOINT balance+turn diff-drive policy with PPO (Phase 2 of JOINT_POLICY_PLAN.md).

net_arch stays small ([64,64]) so it still fits the ESP32. Domain randomization is ON by
default -- the per-wheel asymmetry is the whole point (learn to drive the sticky wheel).
"""
import argparse
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

from rider_env_3d import RiderDiffDriveEnv


def make_env(domain_rand=True, frame_stack=2, stick_slip=False):
    return lambda: RiderDiffDriveEnv(add_noise=True, domain_rand=domain_rand,
                                     frame_stack=frame_stack, mirror_aug=True,
                                     stick_slip=stick_slip)


def evaluate(model, vn, fs, n=12, stick_slip=False):
    """Deterministic, noise/DR off: survival + command-tracking error. With stick_slip the
    nominal loaded actuator is active so the eval reflects the real loaded condition."""
    env = RiderDiffDriveEnv(add_noise=False, domain_rand=False, frame_stack=fs, stick_slip=stick_slip)
    stood = 0; ferr = []; yerr = []
    for s in range(n):
        obs, _ = env.reset(seed=200 + s)
        fe, ye = [], []
        term = False
        while True:
            act, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
            obs, _, term, trunc, _ = env.step(act)
            fe.append(abs(env._fwd_vel() - env.cmd_fwd))
            ye.append(abs(env.data.qvel[env.d_yaw] - env.cmd_yaw))
            if term or trunc:
                break
        if not term:
            stood += 1
        ferr.append(np.mean(fe)); yerr.append(np.mean(ye))
    tag = " [LOADED stick-slip]" if stick_slip else ""
    print(f"  STOOD {stood}/{n} | fwd-track err {np.mean(ferr):.3f} m/s | "
          f"yaw-track err {np.mean(yerr):.2f} rad/s{tag}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--nenv", type=int, default=8)
    ap.add_argument("--out", default="ppo_diffdrive_v1")
    ap.add_argument("--no_dr", action="store_true")
    ap.add_argument("--frame_stack", type=int, default=2)
    ap.add_argument("--stick_slip", action="store_true", help="train on the loaded hysteretic stick-slip actuator")
    ap.add_argument("--warmstart", default=None, help="zip of a policy to FINE-TUNE from (curriculum: balance "
                    "is learned on the clean model first, then adapted to stick-slip -- cold-start can't explore it)")
    ap.add_argument("--warmstart_vn", default=None, help="vecnorm pkl matching --warmstart")
    args = ap.parse_args()

    fs = args.frame_stack
    venv = make_vec_env(make_env(not args.no_dr, fs, args.stick_slip), n_envs=args.nenv)
    if args.warmstart:
        venv = VecNormalize.load(args.warmstart_vn, venv)   # continue from the warm-start's obs normalization
        venv.training = True; venv.norm_reward = True
        model = PPO.load(args.warmstart, env=venv, device="cpu")
        print(f"warm-started from {args.warmstart}")
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
        model = PPO("MlpPolicy", venv, verbose=1, device="cpu", n_steps=1024, batch_size=2048,
                    gamma=0.997, gae_lambda=0.95, learning_rate=3e-4,
                    policy_kwargs=dict(net_arch=[64, 64]),
                    tensorboard_log="./tb")
    print(f"training diff-drive PPO: {args.steps} steps, {args.nenv} envs, DR={not args.no_dr}, "
          f"FS={fs}, stick_slip={args.stick_slip}, warmstart={args.warmstart}")
    model.learn(total_timesteps=args.steps, progress_bar=False)
    model.save(args.out)
    venv.save(args.out + "_vecnorm.pkl")
    print(f"saved -> {args.out}.zip (+ vecnorm)\n")

    venv.training = False
    print("eval (deterministic, noise+DR off, 12 seeds):")
    evaluate(model, venv, fs)
    if args.stick_slip:
        print("eval under the LOADED stick-slip actuator:")
        evaluate(model, venv, fs, stick_slip=True)
