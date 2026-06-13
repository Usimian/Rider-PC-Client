"""Diagnostic: can a simple hand controller balance the velocity plant?

The real robot balances (tenuously) with a pitch-feedback velocity command, so
a PD-on-velocity should stabilize the sim too. If the best PD gets high stood%,
the plant is fine and any RL trouble is a training-config issue. If nothing
stabilizes it, the sim actuator model is wrong.

action = clip(-(kp*pitch + kd*pitch_rate + kx*x + kxv*x_vel), -1, 1)
"""
import numpy as np
from rider_env import RiderBalanceEnv


def run(kp, kd, ki, sign=1.0, seeds=range(200, 215)):
    env = RiderBalanceEnv(add_noise=False)
    stood, xmaxes = 0, []
    for s in seeds:
        obs, _ = env.reset(seed=s)
        steps, xmax = 0, 0.0
        while True:
            pitch, prate, xerr, xvel, wvel, pint, xint = obs[:7]   # first frame
            a = sign * (kp * pitch + kd * prate + ki * pint)
            obs, _, term, trunc, _ = env.step(np.array([np.clip(a, -1, 1)], np.float32))
            steps += 1
            xmax = max(xmax, abs(float(obs[2])))
            if term or trunc:
                break
        if not term:
            stood += 1
            xmaxes.append(xmax)
    return stood, len(list(seeds)), (np.mean(xmaxes) if xmaxes else float("nan"))


if __name__ == "__main__":
    print(f"plant: kv={RiderBalanceEnv().p.vel_kv} forcerange={RiderBalanceEnv().p.vel_forcerange_Nm} "
          f"vel_max={RiderBalanceEnv().p.vel_max_rad_s}\n")
    best = None
    # PI on pitch (kp,kd,ki). Try BOTH signs.
    for sign in (1.0, -1.0):
        for kp in (4, 8, 14, 22):
            for kd in (0.5, 1.2, 2.5):
                for ki in (0.0, 20, 60, 150):
                    stood, n, hold = run(kp, kd, ki, sign)
                    score = (stood, -(hold if hold == hold else 9))
                    if best is None or score > (best[0], -(best[4] if best[4] == best[4] else 9)):
                        best = (stood, kp, kd, ki, hold, n, sign)
                    if stood >= n - 1:
                        print(f"  sign={sign:+.0f} kp={kp:3} kd={kd:4} ki={ki:4}: "
                              f"STOOD {stood}/{n}  hold {hold:.3f} m")
    s, kp, kd, ki, hold, n, sign = best
    print(f"\nBEST: stood {s}/{n}  sign={sign:+.0f} kp={kp} kd={kd} ki={ki}  hold {hold:.3f} m")
