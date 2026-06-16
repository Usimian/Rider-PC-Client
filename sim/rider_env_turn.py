"""Stage 1 of the INCREMENTAL joint policy: the proven 2-D balancer + ONE new dimension (turning).

Everything about balance is kept identical to the working rider_env.py -- same ActuatorModel, same
DR ranges, same reward shape (incl. rate_pen=0.30, act_pen=0.02), same pure_balance obs (position is
handled by firmware code, so x_err/x_int are fed 0). The ONLY thing added is a second 'turn' action
plus yaw_rate / cmd_yaw in the obs, applied as L = balance + turn, R = -balance + turn (the proven
firmware mix, but with the turn LEARNED and coordinated with balance instead of hand-tuned).

If this transfers (balances AND turns without the coupling), the NEXT single dimension gets added
(per-wheel outputs, or drive-velocity commands). One variable at a time. See JOINT_POLICY_PLAN.md.
"""
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from collections import deque
from dataclasses import replace

from rider_params import DEFAULT, RiderParams
from rider_model_3d import build_mjcf_3d
from rider_env import ActuatorModel, DR_RANGES        # reuse the EXACT proven actuator + DR ranges

CMD_YAW_MAX  = 1.5      # rad/s yaw command at full (modest; matches firmware MAX_YAW_RATE)
YAW_BIAS_MAX = 0.10     # rad/s constant gyro-Z bias on the yaw_rate obs -> robust w/o per-boot recal
# External-force DISTURBANCES (domain_rand only): random pushes so the policy MUST use real control
# authority to recover. Without these the 3-D sim is "too easy" and CAPS collapses the action to ~0.
PUSH_PROB    = 0.03     # per control-step chance to start a push
PUSH_FX_MAX  = 4.0      # fore/aft push force (N) -> forces balance authority
PUSH_TZ_MAX  = 0.25     # yaw push torque (N.m) -> forces turn authority


class RiderTurnEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, params: RiderParams = None, render_mode=None, max_seconds: float = 10.0,
                 add_noise: bool = True, domain_rand: bool = True, frame_stack: int = 2,
                 balance_only: bool = False):
        super().__init__()
        self.p = params or DEFAULT
        self.frame_stack = frame_stack
        self.balance_only = balance_only   # isolation: pure 3-D balance (1 action, no turn obs/objective)
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

        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)  # [balance, turn]
        # single-frame obs (9): the working 7 [pitch,prate,xerr(0),fwd_vel,wheel_vel,pitch_int,xint(0)]
        # + the two NEW turn channels [yaw_rate, cmd_yaw].
        self._int_clip = 1.0
        high1 = np.array([np.pi, 50.0, 5.0, 10.0, 200.0, 1.0, 2.0, 20.0, CMD_YAW_MAX], np.float32)
        self.observation_space = spaces.Box(-np.tile(high1, frame_stack), np.tile(high1, frame_stack),
                                             dtype=np.float32)
        self._stack = deque(maxlen=frame_stack)
        self._steps = 0

    def _fwd_vel(self):
        yaw = self.data.qpos[self.q_yaw]
        return self.data.qvel[self.d_x] * np.cos(yaw) + self.data.qvel[self.d_y] * np.sin(yaw)

    def _single_obs(self):
        pitch = self.data.qpos[self.q_pitch]
        prate = self.data.qvel[self.d_pitch]
        fwd = self._fwd_vel()
        wheel = 0.5 * (self.data.qvel[self.d_spinL] + self.data.qvel[self.d_spinR])  # common wheel vel
        yrate = self.data.qvel[self.d_yaw]
        if self.add_noise:
            d2r = np.pi / 180.0
            pitch += np.random.normal(0, self.p.accel_pitch_noise_deg * d2r)
            prate += np.random.normal(0, self.p.gyro_noise_dps * d2r)
            yrate += np.random.normal(0, self.p.gyro_noise_dps * d2r)
        yrate += self.yaw_bias
        fb, lr = self.mirror_fb, self.mirror_lr
        # balance channels odd under the fore/aft flip; turn channels odd under the L/R flip
        o = np.array([fb*pitch, fb*prate, 0.0, fb*fwd, fb*wheel, fb*self._pitch_int, 0.0,
                      lr*yrate, lr*self.cmd_yaw], np.float32)
        return o

    def _obs(self):
        return np.concatenate(self._stack).astype(np.float32)

    def _randomize(self):
        u = self.np_random.uniform
        k = u(*DR_RANGES["mass_scale"])
        self.model.body_mass[self._chassis] = self._nom_mass * k
        self.model.body_inertia[self._chassis] = self._nom_inertia * k
        self.model.geom_friction[self._wL] = self._nom_fricL
        self.model.geom_friction[self._wR] = self._nom_fricR
        self.model.geom_friction[self._wL, 0] = u(*DR_RANGES["wheel_friction"])
        self.model.geom_friction[self._wR, 0] = u(*DR_RANGES["wheel_friction"])

        def wp():
            return replace(self.p,
                           vel_max_rad_s=self.p.vel_max_rad_s * u(*DR_RANGES["vel_gain"]),
                           actuator_tau_s=u(*DR_RANGES["actuator_tau_s"]),
                           deadband_frac=u(*DR_RANGES["deadband_frac"]),
                           latency_s=u(*DR_RANGES["latency_s"]))
        self.actL = ActuatorModel(wp(), self.ctrl_dt)   # independent per wheel (slight asymmetry)
        self.actR = ActuatorModel(wp(), self.ctrl_dt)

    def _sample_cmd(self):
        # half the time HOLD (cmd_yaw=0 = pure balance, identical to the working policy's job),
        # half the time a turn command -> the policy learns turning as the added dimension.
        self.cmd_yaw = 0.0 if self.np_random.uniform() < 0.5 else \
            self.np_random.uniform(-CMD_YAW_MAX, CMD_YAW_MAX)

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
        self.mirror_fb = 1.0 if self.np_random.uniform() < 0.5 else -1.0
        self.mirror_lr = 1.0 if self.np_random.uniform() < 0.5 else -1.0
        self.yaw_bias = self.np_random.uniform(-YAW_BIAS_MAX, YAW_BIAS_MAX) if self.domain_rand else 0.0
        self._pushN = 0                          # remaining steps of the current external push
        self._sample_cmd()
        s = self._single_obs()
        self._stack.clear()
        for _ in range(self.frame_stack):
            self._stack.append(s)
        return self._obs(), {}

    def step(self, action):
        if self.domain_rand:                     # external-force pushes -> force real control authority
            if self._pushN > 0:
                self._pushN -= 1
                if self._pushN == 0:
                    self.data.xfrc_applied[self._chassis] = 0.0
            elif self.np_random.uniform() < PUSH_PROB:
                f = np.zeros(6)
                f[0] = self.np_random.uniform(-PUSH_FX_MAX, PUSH_FX_MAX)   # fore/aft force (N)
                f[5] = self.np_random.uniform(-PUSH_TZ_MAX, PUSH_TZ_MAX)   # yaw torque (N.m)
                self.data.xfrc_applied[self._chassis] = f
                self._pushN = int(self.np_random.integers(2, 8))
        a = np.clip(np.asarray(action, np.float32).flatten(), -1.0, 1.0)
        bal = self.mirror_fb * a[0]              # un-mirror to the real frame
        turn = self.mirror_lr * a[1]
        # SIM wheels are NOT mirrored (both spin the same way for forward), so balance is COMMON
        # and turn is DIFFERENTIAL here. The firmware mirrors the right wheel, turning this into the
        # real-robot mix L=bal+turn, R=-bal+turn (= the existing u+tn path, u=balance, tn=turn).
        self.data.ctrl[0] = self.actL(bal + turn)      # L = balance + turn
        self.data.ctrl[1] = self.actR(bal - turn)      # R = balance - turn
        for _ in range(self.decim):
            mujoco.mj_step(self.model, self.data)
        self._steps += 1
        if self.np_random.uniform() < 0.01:
            self._sample_cmd()

        pitch = self.data.qpos[self.q_pitch]
        yrate = self.data.qvel[self.d_yaw]
        fell = abs(pitch) > 0.40

        upright = np.cos(pitch)
        act_pen = 0.02 * float(np.sum(a ** 2))                       # SAME as working balancer
        rate_pen = 0.30 * float(np.sum((a - self._prev_a) ** 2))     # SAME heavy smoothness -> proportional
        track_yaw = 0.30 * (yrate - self.cmd_yaw) ** 2               # the one new objective: track the turn cmd
        reward = upright - act_pen - rate_pen - track_yaw
        if fell:
            reward -= 10.0
        self._prev_a = a
        self._pitch_int = float(np.clip(self._pitch_int + pitch * self.ctrl_dt, -self._int_clip, self._int_clip))
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
