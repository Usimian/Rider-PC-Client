"""Headless sanity check for the Rider sim. No robot, no training.

Confirms three things that must hold before any RL is meaningful:
  1. the model builds and steps,
  2. zero action -> it falls over (gravity + instability are real),
  3. a hand-tuned PD can keep it up (the plant is controllable and the
     wheel-ground traction actually moves it).
"""
import numpy as np
from rider_env import RiderBalanceEnv


def run(policy, label, seed=0):
    env = RiderBalanceEnv(add_noise=False)
    obs, _ = env.reset(seed=seed)
    steps = 0
    while True:
        obs, r, term, trunc, _ = env.step(policy(obs))
        steps += 1
        if term or trunc:
            break
    print(f"  {label:18s}: survived {steps:4d} ctrl steps "
          f"= {steps * env.ctrl_dt:5.2f}s  (max {env.max_steps}, {'FELL' if term else 'stood'})")
    return steps, env


if __name__ == "__main__":
    env0 = RiderBalanceEnv()
    print(f"model: nq={env0.model.nq} nv={env0.model.nv} nu={env0.model.nu}  "
          f"physics_dt={env0.model.opt.timestep*1e3:.1f}ms  decim={env0.decim}  "
          f"ctrl_dt={env0.ctrl_dt*1e3:.1f}ms ({1/env0.ctrl_dt:.0f}Hz)")
    print(f"total mass = {env0.model.body_subtreemass[1]*1000:.0f} g\n")

    run(lambda o: np.array([0.0], np.float32), "zero action")

    # naive PD on pitch -> wheel torque. Try both signs; one stabilizes.
    for sign in (+1.0, -1.0):
        kp, kd = 8.0, 0.8
        run(lambda o: np.clip(np.array([sign * (kp * o[0] + kd * o[1])], np.float32), -1, 1),
            f"PD (sign {sign:+.0f})")
