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
from caps_ppo import CAPS_PPO       # PPO + spatial action-smoothness (used when --caps_lambda > 0)


def make_env(domain_rand=False, frame_stack=1, vel_pen=0.0, mirror_aug=True, rate_dr=True,
             pos_anchor=0.0, pure_balance=True, pos_weight=0.75, rate_pen=0.30):
    return lambda: RiderBalanceEnv(add_noise=True, domain_rand=domain_rand, frame_stack=frame_stack,
                                   mirror_aug=mirror_aug, vel_pen=vel_pen, rate_dr=rate_dr, rate_pen=rate_pen,
                                   pos_anchor=pos_anchor, pure_balance=pure_balance, pos_weight=pos_weight)


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
    ap.add_argument("--caps_lambda", type=float, default=0.0)  # >0 -> CAPS spatial smoothness (kills mirror_aug jitter)
    ap.add_argument("--caps_sigma", type=float, default=0.05)
    ap.add_argument("--no_mirror", action="store_true")   # disable mirror_aug (reproduce ppo_v_pure recipe)
    ap.add_argument("--no_rate_dr", action="store_true")  # disable control-rate DR (ppo_v_pure predates it)
    ap.add_argument("--pos_anchor", type=float, default=0.0)  # small x^2 penalty: break cruise w/o mirror/velpen
    ap.add_argument("--pos_aware", action="store_true")   # position-aware end-to-end (x_err/x_int in obs + objective)
    ap.add_argument("--pos_weight", type=float, default=0.75)  # position-error^2 weight (pos_aware mode)
    ap.add_argument("--rate_pen", type=float, default=0.30)    # temporal action-rate^2 weight (chatter suppressor)
    args = ap.parse_args()

    venv = make_vec_env(make_env(args.domain_rand, args.frame_stack, args.vel_pen,
                                 mirror_aug=not args.no_mirror, rate_dr=not args.no_rate_dr,
                                 pos_anchor=args.pos_anchor, pure_balance=not args.pos_aware,
                                 pos_weight=args.pos_weight, rate_pen=args.rate_pen), n_envs=args.nenv)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
    common = dict(verbose=1, device="cpu", n_steps=1024, batch_size=2048, gamma=0.997,
                  gae_lambda=0.95, learning_rate=3e-4, policy_kwargs=dict(net_arch=[64, 64]),
                  tensorboard_log="./tb")
    if args.caps_lambda > 0.0:
        model = CAPS_PPO("MlpPolicy", venv, caps_lambda=args.caps_lambda, caps_sigma=args.caps_sigma, **common)
        print(f"training CAPS-PPO (lambda={args.caps_lambda}, sigma={args.caps_sigma}): "
              f"{args.steps} steps, {args.nenv} envs, net [64,64] ...")
    else:
        model = PPO("MlpPolicy", venv, **common)
        print(f"training PPO: {args.steps} steps, {args.nenv} envs, net [64,64] ...")
    model.learn(total_timesteps=args.steps, progress_bar=False)
    model.save(args.out)
    venv.save(args.out + "_vecnorm.pkl")
    print(f"saved -> {args.out}.zip (+ vecnorm)\n")

    venv.training = False
    print("eval (deterministic, noise off, 20 seeds):")
    res = evaluate(model, venv, args.frame_stack)
    summarize(res)
