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
from dataclasses import replace

from rider_params import DEFAULT, RiderParams
from rider_model import build_mjcf

# Domain-randomization ranges. Now that the actuator is MEASURED, these are tight
# around the measured values (the residual uncertainty), not wide guesses -- which
# is exactly the payoff of the bench characterization. Friction stays wide (real
# floor traction is still unknown).
DR_RANGES = {
    "vel_gain":       (0.85, 1.15),    # cmd->velocity gain (vel_max multiplier)
    "actuator_tau_s": (0.010, 0.018),  # tight around measured 13 ms
    "latency_s":      (0.002, 0.006),  # tight around measured 3 ms
    "deadband_frac":  (0.0, 0.03),     # measured negligible
    "mass_scale":     (0.85, 1.15),    # mass measured -> tighter
    "wheel_friction": (0.6, 1.5),      # real-floor traction unknown -> wide
}


class ActuatorModel:
    """Normalized command [-1, 1] -> wheel VELOCITY setpoint (rad/s).

    Models the measured wheel: a velocity source with ~3 ms pure latency and a
    ~13 ms first-order tracking lag. The MuJoCo velocity actuator then tracks the
    setpoint this returns.
    """

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
        target = cmd * self.p.vel_max_rad_s            # velocity setpoint (rad/s)
        if self.p.actuator_tau_s > 0.0:                # first-order tracking lag
            a = self.dt / (self.p.actuator_tau_s + self.dt)
            self.state += a * (target - self.state)
        else:
            self.state = target
        return self.state


class RiderBalanceEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, params: RiderParams = None, render_mode=None,
                 max_seconds: float = 10.0, target_x: float = 0.0, add_noise: bool = True,
                 domain_rand: bool = False, frame_stack: int = 1, pure_balance: bool = True):
        super().__init__()
        self.p = params or DEFAULT
        self.frame_stack = frame_stack
        # pure_balance: position is handled by a CODE loop on the robot (the firmware
        # feeds the policy x_err=0, x_int=0), so train the policy that way -- no position
        # objective, position obs zeroed. Removes the train/deploy mismatch + the wobble.
        self.pure_balance = pure_balance
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
        self.domain_rand = domain_rand

        # cache nominal values that DR perturbs (so each episode scales from nominal)
        self._chassis = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self._wheel_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wheel")
        self._nom_mass = float(self.model.body_mass[self._chassis])
        self._nom_inertia = self.model.body_inertia[self._chassis].copy()
        self._nom_friction = self.model.geom_friction[self._wheel_geom].copy()

        def qadr(name):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            return self.model.jnt_qposadr[jid], self.model.jnt_dofadr[jid]

        (self.q_x, self.d_x) = qadr("slide_x")
        (self.q_pitch, self.d_pitch) = qadr("pitch")
        (self.q_wheel, self.d_wheel) = qadr("wheel_spin")

        self.action_space = spaces.Box(-1.0, 1.0, (1,), np.float32)
        # single-frame obs: [pitch, pitch_rate, x_err, x_vel, wheel_vel, INT_pitch, INT_x]
        # The two integral terms are the key to velocity-mode: the body responds to
        # the derivative of the commanded velocity, so static feedback can't stabilize
        # -- integral (PI-like) state can. Both are trivial to accumulate on the ESP32.
        self._int_clip = (1.0, 2.0)        # clamps (rad*s, m*s) to prevent windup
        high1 = np.array([np.pi, 50.0, 5.0, 10.0, 200.0, 1.0, 2.0], np.float32)
        high = np.tile(high1, self.frame_stack)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self._stack = deque(maxlen=self.frame_stack)
        self._steps = 0

    # ---- helpers ----
    def _raw_state(self):
        pitch = self.data.qpos[self.q_pitch]
        pitch_rate = self.data.qvel[self.d_pitch]
        x = self.data.qpos[self.q_x]
        x_vel = self.data.qvel[self.d_x]
        wheel_vel = self.data.qvel[self.d_wheel]
        return pitch, pitch_rate, x, x_vel, wheel_vel

    def _single_obs(self):
        pitch, pitch_rate, x, x_vel, wheel_vel = self._raw_state()
        if self.add_noise:
            d2r = np.pi / 180.0
            pitch += np.random.normal(0, self.p.accel_pitch_noise_deg * d2r)
            pitch_rate += np.random.normal(0, self.p.gyro_noise_dps * d2r)
        # NOTE: training on quantized-encoder velocity (the 'qnoise' experiment) hurt
        # balance and was abandoned; the deployed policy trains on clean velocity and the
        # FIRMWARE low-passes the velocity obs instead (polvlp) to kill the shimmy.
        xerr = 0.0 if self.pure_balance else (x - self.target_x)   # firmware zeros these (pos in code)
        xint = 0.0 if self.pure_balance else self._x_int
        return np.array([pitch, pitch_rate, xerr, x_vel, wheel_vel,
                         self._pitch_int, xint], np.float32)

    def _obs(self):
        return np.concatenate(self._stack).astype(np.float32)

    # ---- gym API ----
    def _randomize(self):
        """Sample new dynamics for this episode (mass/friction into the model,
        actuator params into a fresh ActuatorModel)."""
        u = self.np_random.uniform
        k = u(*DR_RANGES["mass_scale"])
        self.model.body_mass[self._chassis] = self._nom_mass * k
        self.model.body_inertia[self._chassis] = self._nom_inertia * k     # consistent scale
        self.model.geom_friction[self._wheel_geom] = self._nom_friction
        self.model.geom_friction[self._wheel_geom, 0] = u(*DR_RANGES["wheel_friction"])
        ep = replace(self.p,
                     vel_max_rad_s=self.p.vel_max_rad_s * u(*DR_RANGES["vel_gain"]),
                     actuator_tau_s=u(*DR_RANGES["actuator_tau_s"]),
                     deadband_frac=u(*DR_RANGES["deadband_frac"]),
                     latency_s=u(*DR_RANGES["latency_s"]))
        self.act = ActuatorModel(ep, self.ctrl_dt)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        if self.domain_rand:
            self._randomize()
        # small random initial lean; starts resting on the wheel (chassis pos puts wheel on floor)
        # realistic "let go from a near-still hold": small lean, small pitch rate.
        self.data.qpos[self.q_pitch] = self.np_random.uniform(-0.06, 0.06)
        self.data.qvel[self.d_pitch] = self.np_random.uniform(-0.20, 0.20)
        mujoco.mj_forward(self.model, self.data)
        self.act.reset()
        self._steps = 0
        self._prev_a = 0.0
        self._pitch_int = 0.0
        self._x_int = 0.0
        s = self._single_obs()
        self._stack.clear()
        for _ in range(self.frame_stack):
            self._stack.append(s)
        return self._obs(), {}

    def step(self, action):
        torque = self.act(float(np.asarray(action).flat[0]))
        self.data.ctrl[0] = torque
        for _ in range(self.decim):
            mujoco.mj_step(self.model, self.data)
        self._steps += 1

        a = float(np.asarray(action).flat[0])
        pitch, pitch_rate, x, x_vel, wheel_vel = self._raw_state()
        fell = abs(pitch) > 0.40                       # ~23 deg
        upright = np.cos(pitch)                        # ~1 upright
        # pure_balance: NO position objective (code does position). Heavy command-
        # smoothness to kill the on-robot shimmy (jerky velocity cmd = jerky cart accel).
        pos_pen = 0.0 if self.pure_balance else 0.75 * (x - self.target_x) ** 2
        act_pen = 0.01 * a ** 2
        rate_pen = (0.30 if self.pure_balance else 0.05) * (a - self._prev_a) ** 2
        reward = upright - pos_pen - act_pen - rate_pen
        if fell:
            reward -= 10.0
        self._prev_a = a
        # accumulate integral state (clamped to prevent windup)
        ci, cx = self._int_clip
        self._pitch_int = float(np.clip(self._pitch_int + pitch * self.ctrl_dt, -ci, ci))
        self._x_int = float(np.clip(self._x_int + (x - self.target_x) * self.ctrl_dt, -cx, cx))
        terminated = bool(fell)
        truncated = self._steps >= self.max_steps
        self._stack.append(self._single_obs())
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
