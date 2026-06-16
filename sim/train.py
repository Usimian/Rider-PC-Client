"""Train a balancing+position-hold policy with PPO, then eval. No robot.

net_arch is intentionally small ([64, 64]) so the policy fits on the ESP32 and
runs in microseconds at 250 Hz.
"""
import argparse
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

from rider_env import RiderBalanceEnv


def make_env(domain_rand=False, frame_stack=1, vel_pen=0.0):
    return lambda: RiderBalanceEnv(add_noise=True, domain_rand=domain_rand,
                                   frame_stack=frame_stack, mirror_aug=True, vel_pen=vel_pen)


def evaluate(model, vecnorm, frame_stack=1, n=20):
    env = RiderBalanceEnv(add_noise=False, frame_stack=frame_stack)
    results = []
    for s in range(n):
        obs, _ = env.reset(seed=100 + s)
        steps, xmax = 0, 0.0
        while True:
            nobs = vecnorm.normalize_obs(obs)            # same scaling as training
            act, _ = model.predict(nobs, deterministic=True)
            obs, _, term, trunc, _ = env.step(act)
            steps += 1
            xmax = max(xmax, abs(float(obs[2])))
            if term or trunc:
                break
        results.append((steps * env.ctrl_dt, xmax, not term))
    return results


def summarize(results):
    stood = sum(1 for _, _, s in results if s)
    full = [t for t, _, s in results if s]
    held = [x for _, x, s in results if s]
    import numpy as _np
    print(f"  STOOD {stood}/{len(results)}  "
          f"mean survive {_np.mean([t for t, _, _ in results]):.2f}s  "
          f"(stood eps: hold max|x| mean {_np.mean(held) if held else float('nan'):.3f} m)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=300_000)
    ap.add_argument("--nenv", type=int, default=8)
    ap.add_argument("--out", default="ppo_rider")
    ap.add_argument("--domain_rand", action="store_true")
    ap.add_argument("--frame_stack", type=int, default=1)
    ap.add_argument("--vel_pen", type=float, default=0.0)   # fwd-velocity^2 weight: pins optimum stationary, BUT
                                                            # >0 fights the wheel motion balancing needs -> breaks
                                                            # balance on HW (tried 2026-06-16). Keep 0; see memory.
    args = ap.parse_args()

    venv = make_vec_env(make_env(args.domain_rand, args.frame_stack, args.vel_pen), n_envs=args.nenv)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
    model = PPO("MlpPolicy", venv, verbose=1, device="cpu", n_steps=1024, batch_size=2048,
                gamma=0.997, gae_lambda=0.95, learning_rate=3e-4,
                policy_kwargs=dict(net_arch=[64, 64]),
                tensorboard_log="./tb")
    print(f"training PPO: {args.steps} steps, {args.nenv} envs, net [64,64] ...")
    model.learn(total_timesteps=args.steps, progress_bar=False)
    model.save(args.out)
    venv.save(args.out + "_vecnorm.pkl")
    print(f"saved -> {args.out}.zip (+ vecnorm)\n")

    venv.training = False
    print("eval (deterministic, noise off, 20 seeds):")
    res = evaluate(model, venv, args.frame_stack)
    summarize(res)
