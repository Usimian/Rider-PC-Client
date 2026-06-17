"""Train a SMALL residual on top of the frozen ppo_v_pure base (residual RL).

The residual learns only a slow trim to cancel ppo_v_pure's forward-creep bias; the base keeps
doing the smooth balancing. Eval reports the TOTAL policy (base+residual) signature -- we want
ppo_v_pure's smoothness (jitter ~0.02) but with the cruise killed (final|x| small) i.e. unbiased.
"""
import argparse
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

from rider_env_residual import ResidualBalanceEnv


def make_env(pos_anchor, res_pen, res_rate_pen, domain_rand, frame_stack):
    return lambda: ResidualBalanceEnv(pos_anchor=pos_anchor, res_pen=res_pen, res_rate_pen=res_rate_pen,
                                      domain_rand=domain_rand, frame_stack=frame_stack)


def evaluate(model, vn, fs, n=8):
    env = ResidualBalanceEnv(add_noise=False, domain_rand=False, frame_stack=fs)
    AT = []; RES = []; stood = 0; xf = []
    for s in range(n):
        obs, _ = env.reset(seed=s)
        for i in range(900):
            r, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
            obs, _, term, trunc, info = env.step(r)
            if i > 650:
                AT.append(info["a_total"]); RES.append(abs(info["residual"]))
            if term:
                break
        else:
            stood += 1
        xf.append(abs(float(env.inner.data.qpos[env.inner.q_x])))
    AT = np.array(AT); dA = np.abs(np.diff(AT))
    flip = np.mean(np.sign(AT[1:]) != np.sign(AT[:-1])) * 100
    print(f"  TOTAL(base+res) | stood {stood}/{n} | jitter|da| {dA.mean():.3f} flip% {flip:2.0f} "
          f"| |residual| {np.mean(RES):.3f} | final|x| {np.mean(xf):.2f}m")
    print("  (target: jitter ~0.02 like ppo_v_pure, final|x| small=no cruise, |residual| small)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--nenv", type=int, default=16)
    ap.add_argument("--frame_stack", type=int, default=2)
    ap.add_argument("--pos_anchor", type=float, default=0.5)
    ap.add_argument("--res_pen", type=float, default=0.005)
    ap.add_argument("--res_rate_pen", type=float, default=0.5)
    ap.add_argument("--out", default="ppo_res")
    args = ap.parse_args()

    fs = args.frame_stack
    venv = make_vec_env(make_env(args.pos_anchor, args.res_pen, args.res_rate_pen, True, fs), n_envs=args.nenv)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
    model = PPO("MlpPolicy", venv, verbose=0, device="cpu", n_steps=1024, batch_size=2048,
                gamma=0.997, gae_lambda=0.95, learning_rate=3e-4,
                policy_kwargs=dict(net_arch=[32, 32]), tensorboard_log="./tb")
    print(f"residual RL: {args.steps} steps, pos_anchor={args.pos_anchor}, res_pen={args.res_pen}, net [32,32]")
    model.learn(total_timesteps=args.steps, progress_bar=False)
    model.save(args.out); venv.save(args.out + "_vecnorm.pkl")
    print(f"saved -> {args.out}.zip")
    venv.training = False
    evaluate(model, venv, fs)
