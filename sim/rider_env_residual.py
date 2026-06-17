"""Residual-RL env: frozen base policy (ppo_v_pure) + a small LEARNED residual.

Why residual instead of another full retrain: every from-scratch pure_balance policy that
removes the forward-creep bias also loses ppo_v_pure's smoothness -- because that smoothness
came FROM the cruise (a near-constant command is inherently smooth; active position-holding is
intrinsically jittery at the real plant's delay). So we keep ppo_v_pure as the smooth base and
learn only a small, slow trim that cancels the cruise -> no bias, while the base keeps doing the
fast smooth balancing. The total action is a = clip(base(obs) + residual(obs)).

The base is run as a fast numpy forward of its exported weights (ppo_v_pure_export.npz) -- same
math the firmware runs, deterministic, no torch in the hot loop.
"""
import numpy as np
import gymnasium as gym

from rider_env import RiderBalanceEnv


class BaseForward:
    """Deterministic numpy reimplementation of the exported PPO base policy (matches firmware)."""
    def __init__(self, npz_path):
        d = np.load(npz_path)
        self.mean = d["mean"].astype(np.float64)
        self.std = np.sqrt(d["var"].astype(np.float64) + float(d["eps"]))
        self.clip = float(d["clip"])
        self.layers = [(d[f"W{i}"].astype(np.float64), d[f"b{i}"].astype(np.float64)) for i in range(3)]

    def __call__(self, raw_obs):
        x = np.clip((raw_obs - self.mean) / self.std, -self.clip, self.clip)
        for i, (W, b) in enumerate(self.layers):
            x = W @ x + b
            if i < len(self.layers) - 1:
                x = np.tanh(x)
        return float(np.clip(x, -1.0, 1.0)[0])


class ResidualBalanceEnv(gym.Env):
    """Trainer sees the SAME obs as the base; its action is the residual added to base(obs).

    Reward = the inner pure_balance+pos_anchor reward (computed on the TOTAL action, so smoothness
    and anti-cruise are enforced on what actually drives the wheels) minus a small residual-magnitude
    penalty (keeps the trim small so it can't reintroduce a co-equal jittery controller)."""
    metadata = {"render_modes": [], "render_fps": 50}

    def __init__(self, base_npz="ppo_v_pure_export.npz", pos_anchor=0.5, res_pen=0.005,
                 res_rate_pen=0.5, add_noise=True, domain_rand=True, frame_stack=2, rate_dr=True):
        super().__init__()
        self.base = BaseForward(base_npz)
        self.inner = RiderBalanceEnv(add_noise=add_noise, domain_rand=domain_rand,
                                     frame_stack=frame_stack, pure_balance=True,
                                     mirror_aug=False, pos_anchor=pos_anchor, rate_dr=rate_dr)
        self.res_pen = res_pen            # tiny magnitude penalty (just bounds it; cruise-cancel needs a big DC trim)
        self.res_rate_pen = res_rate_pen  # MAIN penalty: residual rate-of-change -> allows large STEADY trim, forbids jitter
        self.observation_space = self.inner.observation_space
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)  # residual; kept small by res_pen
        self._obs = None

    def reset(self, *, seed=None, options=None):
        obs, info = self.inner.reset(seed=seed, options=options)
        self._obs = obs
        self._prev_r = 0.0
        return obs, info

    def step(self, residual):
        r = float(np.asarray(residual).flat[0])
        a_base = self.base(self._obs.astype(np.float64))
        a_total = float(np.clip(a_base + r, -1.0, 1.0))
        obs, rew, term, trunc, info = self.inner.step(np.array([a_total], np.float32))
        rew -= self.res_pen * r * r                         # tiny magnitude bound
        rew -= self.res_rate_pen * (r - self._prev_r) ** 2  # MAIN: keep the trim SMOOTH (slow DC, no jitter)
        self._prev_r = r
        self._obs = obs
        info = dict(info); info["a_base"] = a_base; info["a_total"] = a_total; info["residual"] = r
        return obs, float(rew), term, trunc, info
