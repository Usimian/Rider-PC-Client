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
from rider_env_twowheel import RiderTwoWheelEnv
from caps_ppo import CAPS_PPO       # PPO + spatial action-smoothness (used when --caps_lambda > 0)


def make_env(domain_rand=False, frame_stack=1, vel_pen=0.0, mirror_aug=True, rate_dr=True,
             pos_anchor=0.0, pure_balance=True, pos_weight=0.75, rate_pen=0.30, drive_train=False,
             fw_obs=True, drive_lead=False, pitch_dis=False, spin_prob=0.0):
    # spin_prob > 0: the TWO-WHEEL env (rider_env_twowheel) -- real turns/spins through the firmware turn controller
    cls, extra = (RiderTwoWheelEnv, dict(spin_prob=spin_prob)) if spin_prob > 0.0 else (RiderBalanceEnv, {})
    return lambda: cls(add_noise=True, domain_rand=domain_rand, frame_stack=frame_stack,
                       mirror_aug=mirror_aug, vel_pen=vel_pen, rate_dr=rate_dr, rate_pen=rate_pen,
                       pos_anchor=pos_anchor, pure_balance=pure_balance, pos_weight=pos_weight,
                       drive_train=drive_train, fw_obs=fw_obs, drive_lead=drive_lead, pitch_dis=pitch_dis, **extra)


def evaluate(model, vecnorm, frame_stack=1, n=20, fw_obs=True):
    env = RiderBalanceEnv(add_noise=False, frame_stack=frame_stack, fw_obs=fw_obs)
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


def anchor_set(model, vecnorm, args):
    """States for the --anchor_lambda penalty: the initial policy (sampled actions, for coverage) holding and driving
    in the two-wheel plant WITHOUT turns, with the training randomization; its mean actions on them are the target."""
    import torch as th
    env = RiderTwoWheelEnv(add_noise=True, domain_rand=args.domain_rand, frame_stack=args.frame_stack,
                           mirror_aug=not args.no_mirror, rate_dr=not args.no_rate_dr, rate_pen=args.rate_pen,
                           drive_train=args.drive_train, drive_lead=args.drive_lead, pitch_dis=args.pitch_dis,
                           spin_prob=0.0)
    obs, _ = env.reset(seed=args.seed)
    states = []
    while len(states) < args.anchor_steps:
        nobs = vecnorm.normalize_obs(obs)
        states.append(nobs)
        act, _ = model.predict(nobs, deterministic=False)
        obs, _, term, trunc, _ = env.step(act)
        if term or trunc:
            obs, _ = env.reset()
    s = th.as_tensor(np.array(states, np.float32))
    with th.no_grad():
        mu = model.policy.get_distribution(s).distribution.mean
    return s, mu


def summarize(results):
    stood = sum(1 for _, _, s in results if s)
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
    ap.add_argument("--drive_train", action="store_true")      # embed the firmware setpoint-tilt DRIVE loop in
                                                               # training -> policy learns forward-drive authority
    ap.add_argument("--legacy_obs", action="store_true")       # CLEAN physics obs (pre-2026-09-16 recipes). Default
                                                               # is the firmware sensing path (FirmwareSensing).
    ap.add_argument("--drive_lead", action="store_true")       # LQR-like stick drive: the target advances at the
                                                               # commanded speed, position P keeps acting (2026-10-19)
    ap.add_argument("--spin_prob", type=float, default=0.0)    # >0: train in the two-wheel env, this share of episodes
                                                               # holding a turnrate (spins / turns, 2026-10-19)
    ap.add_argument("--pitch_dis", action="store_true")        # randomize the floor's pitch errors (moved balance
                                                               # point by leg height, turn drift) into the fed pitch
    ap.add_argument("--seed", type=int, default=None)          # PPO + env + numpy seed (seed sweeps; None = unseeded)
    ap.add_argument("--init", default=None)                    # FINE-TUNE from this saved policy (2026-10-20): its
                                                               # weights and its FROZEN obs normalization (so the
                                                               # export keeps the same input scaling)
    ap.add_argument("--anchor_lambda", type=float, default=0.0)  # with --init: keep the policy's mean action close to
                                                               # the initial policy's on states it visits driving and
                                                               # holding WITHOUT turning (its floor-proven drive style)
    ap.add_argument("--anchor_steps", type=int, default=60_000)  # how many such states to collect
    ap.add_argument("--lr", type=float, default=3e-4)
    args = ap.parse_args()
    if args.seed is not None:
        np.random.seed(args.seed)                              # env sensor noise uses the global numpy RNG

    venv = make_vec_env(make_env(args.domain_rand, args.frame_stack, args.vel_pen,
                                 mirror_aug=not args.no_mirror, rate_dr=not args.no_rate_dr,
                                 pos_anchor=args.pos_anchor, pure_balance=not args.pos_aware,
                                 pos_weight=args.pos_weight, rate_pen=args.rate_pen,
                                 drive_train=args.drive_train, fw_obs=not args.legacy_obs,
                                 drive_lead=args.drive_lead, pitch_dis=args.pitch_dis, spin_prob=args.spin_prob),
                        n_envs=args.nenv, seed=args.seed)
    common = dict(verbose=1, device="cpu", n_steps=1024, batch_size=2048, gamma=0.997,
                  gae_lambda=0.95, learning_rate=args.lr, policy_kwargs=dict(net_arch=[64, 64]),
                  tensorboard_log="./tb", seed=args.seed)
    if args.init:
        venv = VecNormalize.load(args.init + "_vecnorm.pkl", venv)
        venv.training = False                                  # frozen: same input scaling as the initial policy
        model = CAPS_PPO.load(args.init, env=venv, device="cpu", tensorboard_log="./tb",
                              custom_objects=dict(learning_rate=args.lr, lr_schedule=lambda _: args.lr))
        if args.seed is not None:
            model.set_random_seed(args.seed)
        print(f"fine-tuning {args.init} (lr {args.lr}, caps_lambda {model.caps_lambda}): {args.steps} steps")
        if args.anchor_lambda > 0.0:
            model.anchor_obs, model.anchor_mu = anchor_set(model, venv, args)
            model.anchor_lambda = args.anchor_lambda
            print(f"anchor: {len(model.anchor_obs)} drive/hold states, lambda {args.anchor_lambda}")
    elif args.caps_lambda > 0.0:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
        model = CAPS_PPO("MlpPolicy", venv, caps_lambda=args.caps_lambda, caps_sigma=args.caps_sigma, **common)
        print(f"training CAPS-PPO (lambda={args.caps_lambda}, sigma={args.caps_sigma}): "
              f"{args.steps} steps, {args.nenv} envs, net [64,64] ...")
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
        model = PPO("MlpPolicy", venv, **common)
        print(f"training PPO: {args.steps} steps, {args.nenv} envs, net [64,64] ...")
    model.learn(total_timesteps=args.steps, progress_bar=False)
    model.save(args.out)
    venv.save(args.out + "_vecnorm.pkl")
    print(f"saved -> {args.out}.zip (+ vecnorm)\n")

    venv.training = False
    print("eval (deterministic, noise off, 20 seeds):")
    res = evaluate(model, venv, args.frame_stack, fw_obs=not args.legacy_obs)
    summarize(res)
