"""Robustness + sensitivity sweep. No robot.

Runs a trained policy across off-nominal dynamics, one param at a time, and
reports survival time + position excursion. Two payoffs:
  - shows whether domain randomization bought robustness (compare the two models);
  - reveals which params break the policy when wrong = the priority list for the
    bench characterization (measure the sensitive ones precisely; the flat ones
    can be sloppy).
"""
import numpy as np
from dataclasses import replace
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

from rider_params import DEFAULT
from rider_env import RiderBalanceEnv, DR_RANGES

SWEEP_FIELDS = ("latency_s", "actuator_tau_s", "vel_gain",
                "deadband_frac", "mass_scale", "wheel_friction")


def load(name, frame_stack=1):
    model = PPO.load(name, device="cpu")
    vn = VecNormalize.load(name + "_vecnorm.pkl",
                           DummyVecEnv([lambda: RiderBalanceEnv(frame_stack=frame_stack)]))
    vn.training = False
    return model, vn


def rollout(model, vn, params, friction=None, frame_stack=1, seeds=(100, 101)):
    surv, xm = [], []
    for s in seeds:
        env = RiderBalanceEnv(params=params, add_noise=False, frame_stack=frame_stack)
        if friction is not None:
            env.model.geom_friction[env._wheel_geom, 0] = friction
        obs, _ = env.reset(seed=s)
        steps, xmax = 0, 0.0
        while True:
            act, _ = model.predict(vn.normalize_obs(obs), deterministic=True)
            obs, _, term, trunc, _ = env.step(act)
            steps += 1
            xmax = max(xmax, abs(float(obs[2])))
            if term or trunc:
                break
        surv.append(steps * env.ctrl_dt)
        xm.append(xmax)
    return float(np.mean(surv)), float(np.mean(xm))


def sweep(model, vn, field, frame_stack=1, n=5):
    lo, hi = DR_RANGES[field]
    print(f"  {field:16s} [{lo}..{hi}]")
    worst = 99.0
    for val in np.linspace(lo, hi, n):
        if field == "wheel_friction":
            surv, xm = rollout(model, vn, DEFAULT, friction=val, frame_stack=frame_stack)
        elif field == "mass_scale":
            surv, xm = rollout(model, vn, replace(DEFAULT, body_mass_kg=DEFAULT.body_mass_kg * val),
                               frame_stack=frame_stack)
        elif field == "vel_gain":
            surv, xm = rollout(model, vn, replace(DEFAULT, vel_max_rad_s=DEFAULT.vel_max_rad_s * val),
                               frame_stack=frame_stack)
        else:
            surv, xm = rollout(model, vn, replace(DEFAULT, **{field: val}), frame_stack=frame_stack)
        flag = "  <-- FALLS" if surv < 9.5 else ""
        print(f"    {val:8.3f} -> survive {surv:5.2f}s  max|x| {xm:.3f} m{flag}")
        worst = min(worst, surv)
    return worst


# (model name, frame_stack used at training)
MODELS = [("ppo_v_dr_final", 2)]

if __name__ == "__main__":
    import os
    for name, fs in MODELS:
        if not os.path.exists(name + ".zip"):
            continue
        print(f"=== {name} (frame_stack={fs}) : survival across off-nominal dynamics ===")
        model, vn = load(name, fs)
        worsts = {f: sweep(model, vn, f, fs) for f in SWEEP_FIELDS}
        print(f"  most fragile param: {min(worsts, key=worsts.get)} "
              f"(worst survival {min(worsts.values()):.2f}s)\n")
