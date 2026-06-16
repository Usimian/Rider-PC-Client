"""Gymnasium env for the JOINT balance + turn policy (Phase 2 of JOINT_POLICY_PLAN.md).

Diff-drive balancer: 2 actions = per-wheel velocity commands (NOT common+differential, so
no "sum that nulls a wheel" -> the one-sided-pivot failure can't form). Command-conditioned:
the policy is told a forward-velocity and a yaw-rate command and must balance while tracking
both. Per-wheel domain randomization (independent gain/stiction/lag) models the real
asymmetry (the sticky right/dead-leg side) so the policy learns to drive the weak wheel.

Position/heading HOLD stays an outer loop (firmware) that sets cmd_fwd/cmd_yaw; the policy's
job is balance + velocity tracking (cmd_fwd=0,cmd_yaw=w -> spin in place; both 0 -> hold).

The 2-D env (rider_env.py) is kept intact as the working balance baseline.
"""
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from collections import deque
from dataclasses import replace

from rider_params import DEFAULT, RiderParams
from rider_model_3d import build_mjcf_3d
from rider_env import ActuatorModel          # reuse the measured latency+lag+deadband model, per wheel

# Per-episode dynamics ranges. Per-wheel params (gain, stiction-deadband, tau, latency) are
# sampled INDEPENDENTLY for L and R -> the policy sees asymmetric wheels every episode.
DR_RANGES = {
    "vel_gain":       (0.80, 1.20),    # per-wheel cmd->vel gain
    "actuator_tau_s": (0.012, 0.022),  # per-wheel first-order lag (bench: ~16 ms, 2026-06-16)
    "latency_s":      (0.006, 0.014),  # per-wheel pure latency (bench actuator ~4.6 ms + the 6 ms loop)
    "stiction_frac":  (0.0, 0.10),     # per-wheel deadband -> models a sticky wheel (one side dead-leg)
    "mass_scale":     (0.85, 1.15),
    "wheel_friction": (0.6, 1.5),
    "yaw_rate_bias":  (-0.20, 0.20),   # constant gyro-Z zero-rate bias (rad/s, ~+/-11 deg/s) added to the
                                       # yaw-rate OBS -> policy must stay stable without per-boot gyro recal
}

CMD_FWD_MAX = 0.30     # m/s forward command at full
CMD_YAW_MAX = 2.0      # rad/s yaw command at full
VEL_OBS_LP  = 0.7      # wheel-velocity obs low-pass, MATCHES firmware gPolVelLP (models the ~17ms sensing lag)


class RiderDiffDriveEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, params: RiderParams = None, render_mode=None, max_seconds: float = 10.0,
                 add_noise: bool = True, domain_rand: bool = True, frame_stack: int = 2,
                 mirror_aug: bool = True):
        super().__init__()
        self.p = params or DEFAULT
        self.frame_stack = frame_stack
        self.mirror_aug = mirror_aug
        self.mirror = 1.0
        self.model = mujoco.MjModel.from_xml_string(build_mjcf_3d(self.p))
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        self._viewer = None

        self.decim = self.p.control_decimation()
        self.ctrl_dt = self.decim * self.model.opt.timestep
        self.actL = ActuatorModel(self.p, self.ctrl_dt)
        self.actR = ActuatorModel(self.p, self.ctrl_dt)
        self.max_steps = int(max_seconds / self.ctrl_dt)
        self.add_noise = add_noise
        self.domain_rand = domain_rand

        self._chassis = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self._wL = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wheelL")
        self._wR = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wheelR")
        self._nom_mass = float(self.model.body_mass[self._chassis])
        self._nom_inertia = self.model.body_inertia[self._chassis].copy()
        self._nom_fricL = self.model.geom_friction[self._wL].copy()
        self._nom_fricR = self.model.geom_friction[self._wR].copy()

        def adr(name):
            j = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            return self.model.jnt_qposadr[j], self.model.jnt_dofadr[j]
        (self.q_x, self.d_x) = adr("slide_x")
        (self.q_y, self.d_y) = adr("slide_y")
        (self.q_yaw, self.d_yaw) = adr("yaw")
        (self.q_pitch, self.d_pitch) = adr("pitch")
        (_, self.d_spinL) = adr("spin_L")
        (_, self.d_spinR) = adr("spin_R")

        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
        # single-frame obs (8): [pitch, pitch_rate, yaw_rate, wL_vel, wR_vel, cmd_fwd, cmd_yaw, pitch_int]
        high1 = np.array([np.pi, 50.0, 20.0, 200.0, 200.0, CMD_FWD_MAX, CMD_YAW_MAX, 1.0], np.float32)
        self.observation_space = spaces.Box(-np.tile(high1, frame_stack), np.tile(high1, frame_stack),
                                             dtype=np.float32)
        self._stack = deque(maxlen=frame_stack)
        self._steps = 0

    # ---- command sampling ----
    def _sample_cmd(self):
        """Pick a maneuver: hold, drive, spin-in-place, or combined."""
        kind = self.np_random.integers(0, 4)
        f = self.np_random.uniform(-CMD_FWD_MAX, CMD_FWD_MAX)
        w = self.np_random.uniform(-CMD_YAW_MAX, CMD_YAW_MAX)
        if kind == 0:   self.cmd_fwd, self.cmd_yaw = 0.0, 0.0          # hold/balance
        elif kind == 1: self.cmd_fwd, self.cmd_yaw = f, 0.0           # drive straight
        elif kind == 2: self.cmd_fwd, self.cmd_yaw = 0.0, w           # spin in place
        else:           self.cmd_fwd, self.cmd_yaw = f, w             # combined

    # ---- state / obs ----
    def _fwd_vel(self):
        yaw = self.data.qpos[self.q_yaw]
        return self.data.qvel[self.d_x] * np.cos(yaw) + self.data.qvel[self.d_y] * np.sin(yaw)

    def _single_obs(self):
        pitch = self.data.qpos[self.q_pitch]
        prate = self.data.qvel[self.d_pitch]
        yrate = self.data.qvel[self.d_yaw]
        wL = self.data.qvel[self.d_spinL]
        wR = self.data.qvel[self.d_spinR]
        if self.add_noise:
            d2r = np.pi / 180.0
            pitch += np.random.normal(0, self.p.accel_pitch_noise_deg * d2r)
            prate += np.random.normal(0, self.p.gyro_noise_dps * d2r)
            yrate += np.random.normal(0, self.p.gyro_noise_dps * d2r)
        yrate += self.yaw_bias          # per-episode constant gyro-Z bias (0 when domain_rand off)
        # low-pass the wheel-velocity obs to match firmware gPolVelLP -> policy trains against the
        # SAME sensing lag it meets on the robot (the main unmodeled delay behind the oscillation).
        self.wLf = VEL_OBS_LP*self.wLf + (1.0-VEL_OBS_LP)*wL
        self.wRf = VEL_OBS_LP*self.wRf + (1.0-VEL_OBS_LP)*wR
        wL, wR = self.wLf, self.wRf
        mr = self.mirror
        # mirror (L<->R reflection): swap wheels, negate yaw-rate & yaw command
        o = np.array([pitch, prate, mr * yrate, (wL if mr > 0 else wR), (wR if mr > 0 else wL),
                      self.cmd_fwd, mr * self.cmd_yaw, self._pitch_int], np.float32)
        return o

    def _obs(self):
        return np.concatenate(self._stack).astype(np.float32)

    # ---- DR ----
    def _randomize(self):
        u = self.np_random.uniform
        k = u(*DR_RANGES["mass_scale"])
        self.model.body_mass[self._chassis] = self._nom_mass * k
        self.model.body_inertia[self._chassis] = self._nom_inertia * k
        self.model.geom_friction[self._wL] = self._nom_fricL
        self.model.geom_friction[self._wR] = self._nom_fricR
        self.model.geom_friction[self._wL, 0] = u(*DR_RANGES["wheel_friction"])
        self.model.geom_friction[self._wR, 0] = u(*DR_RANGES["wheel_friction"])

        def wheel_params():
            return replace(self.p,
                           vel_max_rad_s=self.p.vel_max_rad_s * u(*DR_RANGES["vel_gain"]),
                           actuator_tau_s=u(*DR_RANGES["actuator_tau_s"]),
                           latency_s=u(*DR_RANGES["latency_s"]),
                           deadband_frac=u(*DR_RANGES["stiction_frac"]))
        self.actL = ActuatorModel(wheel_params(), self.ctrl_dt)   # independent per wheel
        self.actR = ActuatorModel(wheel_params(), self.ctrl_dt)   # -> asymmetric (sticky side)

    # ---- gym API ----
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        if self.domain_rand:
            self._randomize()
        self.data.qpos[self.q_pitch] = self.np_random.uniform(-0.06, 0.06)
        self.data.qvel[self.d_pitch] = self.np_random.uniform(-0.20, 0.20)
        mujoco.mj_forward(self.model, self.data)
        self.actL.reset(); self.actR.reset()
        self._steps = 0
        self._prev_a = np.zeros(2, np.float32)
        self._pitch_int = 0.0
        self.mirror = (1.0 if self.np_random.uniform() < 0.5 else -1.0) if self.mirror_aug else 1.0
        self.yaw_bias = self.np_random.uniform(*DR_RANGES["yaw_rate_bias"]) if self.domain_rand else 0.0
        self.wLf = 0.0; self.wRf = 0.0          # wheel-velocity obs low-pass state
        self._sample_cmd()
        s = self._single_obs()
        self._stack.clear()
        for _ in range(self.frame_stack):
            self._stack.append(s)
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, np.float32).flatten(), -1.0, 1.0)
        # un-mirror the action back to the real L/R frame before applying
        aL, aR = (a[0], a[1]) if self.mirror > 0 else (a[1], a[0])
        self.data.ctrl[0] = self.actL(float(aL))
        self.data.ctrl[1] = self.actR(float(aR))
        for _ in range(self.decim):
            mujoco.mj_step(self.model, self.data)
        self._steps += 1

        # occasionally switch the command mid-episode so it learns transitions
        if self.np_random.uniform() < 0.01:
            self._sample_cmd()

        pitch = self.data.qpos[self.q_pitch]
        fwd = self._fwd_vel()
        yrate = self.data.qvel[self.d_yaw]
        wL = self.data.qvel[self.d_spinL]; wR = self.data.qvel[self.d_spinR]
        fell = abs(pitch) > 0.40

        upright = np.cos(pitch)
        track_fwd = 6.0 * (fwd - self.cmd_fwd) ** 2        # tuned up (was 2.0) -> tighter forward tracking
        track_yaw = 1.2 * (yrate - self.cmd_yaw) ** 2      # tuned up (was 0.30) -> tighter yaw tracking
        act_pen = 0.02 * float(np.sum(a ** 2))                     # up (was 0.01) -> discourage bang-bang saturation
        rate_pen = 0.30 * float(np.sum((a - self._prev_a) ** 2))   # up (was 0.08) -> strongly favor SMOOTH control
                                                                   # (bench: policy was bang-bang -> oscillated on hw)
        # anti-stall: while turning, both wheels should contribute -- penalize one wheel idle
        stall_pen = 0.0
        if abs(self.cmd_yaw) > 0.2:
            stall_pen = 0.10 * np.exp(-min(abs(wL), abs(wR)) / 3.0)  # stronger (was 0.05) -> harder on one-sided turns
        reward = upright - track_fwd - track_yaw - act_pen - rate_pen - stall_pen
        if fell:
            reward -= 10.0
        self._prev_a = a

        self._pitch_int = float(np.clip(self._pitch_int + pitch * self.ctrl_dt, -1.0, 1.0))
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
            self._viewer.close(); self._viewer = None
