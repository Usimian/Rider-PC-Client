"""Train Stage 1 (balance + turn) with PPO. Same hyperparams/net as the working balancer."""
import argparse
import numpy as np
from dataclasses import replace
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

from rider_params import DEFAULT
from rider_env_turn import RiderTurnEnv
from caps_ppo import CAPS_PPO


def make_env(domain_rand=True, frame_stack=2, control_hz=143.0):
    p = replace(DEFAULT, control_hz=control_hz)     # match the REAL ESP32 loop rate (turn policy enabled)
    return lambda: RiderTurnEnv(params=p, add_noise=True, domain_rand=domain_rand, frame_stack=frame_stack)


def evaluate(model, vn, fs, control_hz=143.0, n=12):
    env = RiderTurnEnv(params=replace(DEFAULT, control_hz=control_hz), add_noise=False, domain_rand=False, frame_stack=fs)
    stood = 0; yerr = []
    for s in range(n):
        obs, _ = env.reset(seed=300 + s)
        ye = []; term = False
        while True:
            act, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
            obs, _, term, trunc, _ = env.step(act)
            ye.append(abs(env.data.qvel[env.d_yaw] - env.cmd_yaw))
            if term or trunc:
                break
        if not term:
            stood += 1
        yerr.append(np.mean(ye))
    print(f"  STOOD {stood}/{n} | yaw-track err {np.mean(yerr):.2f} rad/s")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--nenv", type=int, default=8)
    ap.add_argument("--out", default="ppo_turn_v1")
    ap.add_argument("--frame_stack", type=int, default=2)
    ap.add_argument("--caps_lambda", type=float, default=2.0)   # CAPS spatial-smoothness weight
    ap.add_argument("--caps_sigma", type=float, default=0.05)   # state-perturbation std (normalized obs)
    ap.add_argument("--control_hz", type=float, default=143.0)  # match real ESP32 loop (turn policy enabled)
    args = ap.parse_args()

    fs = args.frame_stack
    venv = make_vec_env(make_env(True, fs, args.control_hz), n_envs=args.nenv)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
    model = CAPS_PPO("MlpPolicy", venv, verbose=1, device="cpu", n_steps=1024, batch_size=2048,
                     gamma=0.997, gae_lambda=0.95, learning_rate=3e-4,
                     policy_kwargs=dict(net_arch=[64, 64]), tensorboard_log="./tb",
                     caps_lambda=args.caps_lambda, caps_sigma=args.caps_sigma)
    print(f"training balance+turn CAPS-PPO: {args.steps} steps, {args.nenv} envs, FS={fs}, "
          f"caps_lambda={args.caps_lambda}, caps_sigma={args.caps_sigma}, control_hz={args.control_hz}")
    model.learn(total_timesteps=args.steps, progress_bar=False)
    model.save(args.out)
    venv.save(args.out + "_vecnorm.pkl")
    print(f"saved -> {args.out}.zip\n")
    venv.training = False
    print("eval (deterministic, noise+DR off, 12 seeds):")
    evaluate(model, venv, fs, args.control_hz)
