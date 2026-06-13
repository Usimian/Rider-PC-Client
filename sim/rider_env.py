"""Gymnasium env for the Rider balancer + an isolated actuator model.

The ActuatorModel is deliberately separate: it maps the policy's normalized
command through the BENCH-measured non-idealities (deadband, saturation,
first-order lag, pure latency). When we characterize the real wheel on the
bench, only this module's params change -- the policy interface stays fixed.
This is also the natural place to hang domain randomization.
"""
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from collections import deque

from rider_params import DEFAULT, RiderParams
from rider_model import build_mjcf


class ActuatorModel:
    """Normalized command [-1, 1] -> applied wheel torque (Nm)."""

    def __init__(self, p: RiderParams, dt: float):
        self.p = p
        self.dt = dt
        self.state = 0.0
        n_lat = max(0, round(p.latency_s / dt))
        self._buf = deque([0.0] * n_lat, maxlen=n_lat) if n_lat > 0 else None

    def reset(self):
        self.state = 0.0
        if self._buf is not None:
            self._buf.clear()
            self._buf.extend([0.0] * self._buf.maxlen)

    def __call__(self, cmd: float) -> float:
        cmd = float(np.clip(cmd, -1.0, 1.0))
        if self._buf is not None:                      # pure latency
            self._buf.append(cmd)
            cmd = self._buf[0]
        db = self.p.deadband_frac                      # deadband
        if db > 0.0:
            if abs(cmd) < db:
                cmd = 0.0
            else:
                cmd = np.sign(cmd) * (abs(cmd) - db) / (1.0 - db)
        target = cmd * self.p.torque_max_Nm            # saturation (cmd already clipped)
        if self.p.actuator_tau_s > 0.0:                # first-order lag
            a = self.dt / (self.p.actuator_tau_s + self.dt)
            self.state += a * (target - self.state)
        else:
            self.state = target
        return self.state


class RiderBalanceEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, params: RiderParams = None, render_mode=None,
                 max_seconds: float = 10.0, target_x: float = 0.0, add_noise: bool = True):
        super().__init__()
        self.p = params or DEFAULT
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(self.p))
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        self._viewer = None

        self.decim = self.p.control_decimation()
        self.ctrl_dt = self.decim * self.model.opt.timestep
        self.act = ActuatorModel(self.p, self.ctrl_dt)
        self.max_steps = int(max_seconds / self.ctrl_dt)
        self.target_x = target_x
        self.add_noise = add_noise

        def qadr(name):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            return self.model.jnt_qposadr[jid], self.model.jnt_dofadr[jid]

        (self.q_x, self.d_x) = qadr("slide_x")
        (self.q_pitch, self.d_pitch) = qadr("pitch")
        (self.q_wheel, self.d_wheel) = qadr("wheel_spin")

        self.action_space = spaces.Box(-1.0, 1.0, (1,), np.float32)
        # obs: [pitch(rad), pitch_rate(rad/s), x_err(m), x_vel(m/s), wheel_vel(rad/s)]
        high = np.array([np.pi, 50.0, 5.0, 10.0, 200.0], np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self._steps = 0

    # ---- helpers ----
    def _raw_state(self):
        pitch = self.data.qpos[self.q_pitch]
        pitch_rate = self.data.qvel[self.d_pitch]
        x = self.data.qpos[self.q_x]
        x_vel = self.data.qvel[self.d_x]
        wheel_vel = self.data.qvel[self.d_wheel]
        return pitch, pitch_rate, x, x_vel, wheel_vel

    def _obs(self):
        pitch, pitch_rate, x, x_vel, wheel_vel = self._raw_state()
        if self.add_noise:
            d2r = np.pi / 180.0
            pitch += np.random.normal(0, self.p.accel_pitch_noise_deg * d2r)
            pitch_rate += np.random.normal(0, self.p.gyro_noise_dps * d2r)
        return np.array([pitch, pitch_rate, x - self.target_x, x_vel, wheel_vel], np.float32)

    # ---- gym API ----
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        # small random initial lean; starts resting on the wheel (chassis pos puts wheel on floor)
        self.data.qpos[self.q_pitch] = self.np_random.uniform(-0.05, 0.05)
        mujoco.mj_forward(self.model, self.data)
        self.act.reset()
        self._steps = 0
        return self._obs(), {}

    def step(self, action):
        torque = self.act(float(np.asarray(action).flat[0]))
        self.data.ctrl[0] = torque
        for _ in range(self.decim):
            mujoco.mj_step(self.model, self.data)
        self._steps += 1

        pitch, pitch_rate, x, x_vel, wheel_vel = self._raw_state()
        fell = abs(pitch) > 0.40                       # ~23 deg
        upright = np.cos(pitch)                        # 1 upright, falls off with lean
        pos_pen = 2.0 * (x - self.target_x) ** 2
        act_pen = 0.01 * float(np.asarray(action).flat[0]) ** 2
        reward = upright - pos_pen - act_pen
        if fell:
            reward -= 10.0
        terminated = bool(fell)
        truncated = self._steps >= self.max_steps
        return self._obs(), float(reward), terminated, truncated, {}

    def render(self):
        if self.render_mode == "human":
            if self._viewer is None:
                import mujoco.viewer
                self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._viewer.sync()

    def close(self):
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
