"""Two-wheel (3-D) version of the firmware-faithful balance env -- for turning and spinning in place.

The 2-D env (rider_env.RiderBalanceEnv) is one lumped wheel in the sagittal plane, so it cannot spin. This
subclass keeps EVERYTHING the 2-D env validated against the floor (the measured wheel model per wheel, the
firmware's sensing: tilt blending with the accelerometer coupling, encoder speed, LQR-style steering via
drive_sim, the drive loop) and replaces the plant with two independent wheels on a body that can yaw, plus
the firmware's TURN controller (main.cpp): while a yaw-rate command is held, turn = FF*cmd + Kp*(cmd - rate)
from the gyro; otherwise a heading lock (PD on the two wheels' encoder difference). Both through the
firmware's 0.5 low-pass and +/-250 clamp. The balance output u and the turn tn mix as the firmware does:
left wheel u + tn, right wheel u - tn (forward commands). Each wheel has its own ActuatorModel.

Tyres: cylinders 20 mm wide on the floor -- spinning in place, the contact across that width scrubs, so the
physics engine supplies the sideways drag a real spin has (what the 2-D sim cannot).
The policy still outputs ONE number (the balance command): the deployed policies run unchanged.

Run via drive_sim.simulate(..., twowheel=True, yaw_cmd=1.0).
"""
import numpy as np
import mujoco

from rider_env import RiderBalanceEnv, ActuatorModel

# firmware turn controller (main.cpp defaults)
YAW_FF, YAW_KP = 75.0, 40.0          # gYawFF (raw per rad/s commanded), gYawKp (raw per rad/s of error)
TURN_MAX, TURN_LP = 250.0, 0.5       # gTurnMax (raw), gTurnLP (per cycle)
HDG_KP, HDG_KD = 1.0, 2.0            # gHdgKp (raw per count of wheel1-wheel2), gHdgKd (raw per wheel_vx unit)
WV_PER_MPS = 265.0                   # wheel_vx units per m/s (floor, 2026-10-19)
LP_VEL = 0.85                        # firmware per-wheel speed low-pass
CMD_FULL = 30.0 * 7.764              # raw wheel-cmd at action 1
# training randomization (spin_prob > 0, domain_rand)
SPIN_CMD = (0.3, 1.0)                # turnrate held (rad/s; the joystick's full stick = 1.0)
SCRUB_DR = (0.02, 0.06)              # tyre torsional friction about the calibrated 0.04
RIGHT_DEADBAND = (0.05, 0.15)        # the right wheel's own dead zone (the left gets the 2-D DR range too)
WHEEL_HALF_WIDTH = 0.010             # m (tyre ~20 mm wide)
SCRUB = [4, 0.04]                    # wheel contact: condim 4 (+ torsional) and the torsional friction (m). CALIBRATED
                                     # 2026-10-19: at turnrate 1.0 the floor needs turn ~60-67 raw for yaw ~1.2-1.3
                                     # rad/s; 0.04 gives 66 / 1.22 (no torsion: 52 / 1.58 -- the spin was too free)


def build_mjcf_twowheel(p):
    hx, hy, hz = p.body_half_extent_m
    half_track = p.track_width_m / 2.0
    r = p.wheel_radius_m
    vmax = 1.25 * p.vel_max_rad_s * max(1.0, p.cmd_limit)
    frng = p.vel_forcerange_Nm
    arm = p.wheel_armature / 2.0     # per wheel: two spin joints share the lumped value (see rider_model_3d)
    ww = WHEEL_HALF_WIDTH
    return f"""<mujoco model="rider_twowheel">
  <option timestep="{p.physics_timestep_s}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <geom contype="1" conaffinity="1" friction="1.0 0.005 0.0001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.05" rgba="0.3 0.3 0.35 1"/>
    <body name="chassis" pos="0 0 {r}">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <joint name="slide_y" type="slide" axis="0 1 0"/>
      <joint name="slide_z" type="slide" axis="0 0 1"/>
      <joint name="yaw"     type="hinge" axis="0 0 1"/>
      <joint name="pitch"   type="hinge" axis="0 1 0"/>
      <geom name="torso" type="box" pos="0 0 {p.com_height_m}" size="{hx} {hy} {hz}" mass="{p.body_mass_kg}"
            contype="0" conaffinity="0" rgba="0.85 0.5 0.2 1"/>
      <body name="wheel_L" pos="0 {half_track} 0">
        <joint name="spin_L" type="hinge" axis="0 1 0" armature="{arm}"/>
        <geom name="wheelL" type="cylinder" fromto="0 {-ww} 0 0 {ww} 0" size="{r}" mass="{p.wheel_mass_kg}"
              condim="{SCRUB[0]}" friction="1.0 {SCRUB[1]} 0.0001"/>
      </body>
      <body name="wheel_R" pos="0 {-half_track} 0">
        <joint name="spin_R" type="hinge" axis="0 1 0" armature="{arm}"/>
        <geom name="wheelR" type="cylinder" fromto="0 {-ww} 0 0 {ww} 0" size="{r}" mass="{p.wheel_mass_kg}"
              condim="{SCRUB[0]}" friction="1.0 {SCRUB[1]} 0.0001"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="L" joint="spin_L" kv="{p.vel_kv}" ctrlrange="{-vmax} {vmax}" forcerange="{-frng} {frng}"/>
    <velocity name="R" joint="spin_R" kv="{p.vel_kv}" ctrlrange="{-vmax} {vmax}" forcerange="{-frng} {frng}"/>
  </actuator>
</mujoco>
"""


class RiderTwoWheelEnv(RiderBalanceEnv):
    """RiderBalanceEnv with two wheels, yaw and the firmware turn controller. Set .yaw_cmd (rad/s, the
    'turnrate' the joystick sends; full stick = 1.0) at any time."""

    def __init__(self, *args, spin_prob: float = 0.0, **kwargs):
        # spin_prob (training, needs domain_rand): share of episodes that hold a turnrate for part of the episode
        self.spin_prob = spin_prob
        self._yaw_plan = None                                       # (cmd, start step, end step) or None
        super().__init__(*args, **kwargs)
        # swap in the two-wheel plant (the parent built the 1-wheel one; everything else is reused)
        self.model = mujoco.MjModel.from_xml_string(build_mjcf_twowheel(self.p))
        self.data = mujoco.MjData(self.model)
        self.decim = self.p.control_decimation()
        self.ctrl_dt = self.decim * self.model.opt.timestep
        self.max_steps = int(self.max_seconds / self.ctrl_dt)
        self._chassis = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self._wheel_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wheelL")
        self._nom_mass = float(self.model.body_mass[self._chassis])
        self._nom_inertia = self.model.body_inertia[self._chassis].copy()
        self._nom_friction = self.model.geom_friction[self._wheel_geom].copy()

        def qadr(name):
            j = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            return self.model.jnt_qposadr[j], self.model.jnt_dofadr[j]
        (self.q_x, self.d_x) = qadr("slide_x")
        (self.q_y, self.d_y) = qadr("slide_y")
        (self.q_yaw, self.d_yaw) = qadr("yaw")
        (self.q_pitch, self.d_pitch) = qadr("pitch")
        (self.q_wL, self.d_wL) = qadr("spin_L")
        (self.q_wR, self.d_wR) = qadr("spin_R")
        self.q_wheel, self.d_wheel = self.q_wL, self.d_wL      # (unused: _wheel_angle is overridden)
        self.q_flex = self.d_flex = None
        self.right_kw = {}                                          # per-wheel asymmetry: RiderParams fields for R only
        self.actL = ActuatorModel(self.p, self.ctrl_dt)
        self.actR = ActuatorModel(self.p, self.ctrl_dt)
        self.yaw_cmd = 0.0
        self._turn_reset()
        # the encoder model: each wheel quantized on its own, the firmware averages -> override the rounding
        r = self.p.wheel_radius_m
        self.sense._enc_x = lambda a: a * r                       # a = the already-quantized mean angle

    def _turn_reset(self):
        self.turn_f = 0.0; self.turning_prev = False; self.hdg_target = 0.0
        self.vL_f = 0.0; self.vR_f = 0.0
        self.last_turn = 0.0

    # ---- state ----
    def _fwd_vel(self):
        yaw = self.data.qpos[self.q_yaw]
        return self.data.qvel[self.d_x] * np.cos(yaw) + self.data.qvel[self.d_y] * np.sin(yaw)

    def _raw_state(self):
        pitch = self.data.qpos[self.q_pitch]
        pitch_rate = self.data.qvel[self.d_pitch]
        r = self.p.wheel_radius_m
        wl, wr = self.data.qpos[self.q_wL], self.data.qpos[self.q_wR]
        x = r * (0.5 * (wl + wr) + pitch)                          # forward travel from the wheels (no slip)
        x_vel = self._fwd_vel()
        wheel_vel = 0.5 * (self.data.qvel[self.d_wL] + self.data.qvel[self.d_wR])
        return pitch, pitch_rate, x, x_vel, wheel_vel

    def _counts(self):
        k = 1024.0 / (2 * np.pi)
        return np.round(self.data.qpos[self.q_wL] * k), np.round(self.data.qpos[self.q_wR] * k)

    def _wheel_angle(self):
        cl, cr = self._counts()
        return 0.5 * (cl + cr) * (2 * np.pi / 1024.0)            # firmware wheel_x: mean of the two encoders

    def yaw_rate_turn(self):
        """Yaw rate in the firmware's turn sense: positive when the LEFT wheel runs faster (L = u + tn)."""
        return -self.data.qvel[self.d_yaw]

    # ---- the firmware turn controller + mixing ----
    def _turn_output(self):
        r = self.p.wheel_radius_m
        a = 1.0 - LP_VEL
        self.vL_f = LP_VEL * self.vL_f + a * self.data.qvel[self.d_wL] * r * WV_PER_MPS
        self.vR_f = LP_VEL * self.vR_f + a * self.data.qvel[self.d_wR] * r * WV_PER_MPS
        cl, cr = self._counts()
        d = cl - cr
        turning = abs(self.yaw_cmd) > 0.001
        if turning:
            raw = YAW_FF * self.yaw_cmd + YAW_KP * (self.yaw_cmd - self.yaw_rate_turn())
            self.hdg_target = d                                    # latches the new heading on release
        else:
            raw = HDG_KP * (self.hdg_target - d) - HDG_KD * (self.vL_f - self.vR_f)
        raw = float(np.clip(raw, -TURN_MAX, TURN_MAX))
        self.turn_f = TURN_LP * self.turn_f + (1.0 - TURN_LP) * raw
        return self.turn_f

    def _apply_action(self, a):
        if self._yaw_plan is not None:                              # training: the episode's turn schedule
            cmd, s0, s1 = self._yaw_plan
            self.yaw_cmd = cmd if s0 <= self._steps < s1 else 0.0
        tn = self._turn_output() / CMD_FULL                         # raw -> action units
        self.last_turn = tn * CMD_FULL
        self.data.ctrl[0] = self.actL(a + tn)
        self.data.ctrl[1] = self.actR(a - tn)

    # ---- episode ----
    def _randomize(self):
        from dataclasses import replace
        super()._randomize()                                        # this episode's wheel params -> self.act, friction -> L
        p = self.act.p
        u = self.np_random.uniform
        self.actL = ActuatorModel(p, self.ctrl_dt)
        # real wheels are never equal: the right one gets its own dead zone and +/-10 % strength
        pr = replace(p, deadband_frac=u(*RIGHT_DEADBAND), vel_max_rad_s=p.vel_max_rad_s * u(0.9, 1.1))
        self.actR = ActuatorModel(pr, self.ctrl_dt)
        wl = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wheelL")
        wr = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wheelR")
        self.model.geom_friction[wl, 1] = u(*SCRUB_DR)               # tyre scrub (torsional friction)
        self.model.geom_friction[wr] = self.model.geom_friction[wl]

    def set_right_wheel(self, **kw):
        """Make the RIGHT wheel differ (e.g. deadband_frac=0.15, stick_slip=True): real wheels are never equal."""
        from dataclasses import replace
        self.right_kw = kw
        self.actR = ActuatorModel(replace(self.actL.p, **kw), self.ctrl_dt)

    def reset(self, *, seed=None, options=None):
        self._turn_reset()
        self._yaw_plan = None
        out = super().reset(seed=seed, options=options)
        if self.right_kw:
            self.set_right_wheel(**self.right_kw)
        self.actL.reset(); self.actR.reset()
        if self.domain_rand and self.spin_prob > 0.0 and self.np_random.uniform() < self.spin_prob:
            # a turn: full-ish stick either way, starting 0.5-3 s in, held 2-6 s (then released: the stop too).
            # Half of them spin in place (the floor shimmy), half turn while driving.
            u = self.np_random.uniform
            cmd = u(*SPIN_CMD) * (1.0 if u() < 0.5 else -1.0)
            s0 = int(u(0.5, 3.0) / self.ctrl_dt); s1 = s0 + int(u(2.0, 6.0) / self.ctrl_dt)
            self._yaw_plan = (cmd, s0, s1)
            if self.drive_train and u() < 0.5:
                self._drive_cmd = 0.0
        return out
