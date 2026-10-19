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
    "vel_gain":       (0.80, 1.20),    # cmd->velocity gain (vel_max multiplier) about the MEASURED loaded nominal
                                       # 17.5 rad/s (2026-10-19 floor ground speed; per-window scatter 0.86-1.15 of
                                       # it at 7.3-7.7 V, plus battery: a full pack is stronger, a low one weaker)
    "actuator_tau_s": (0.010, 0.018),  # tight around measured 13 ms
    "latency_s":      (0.000, 0.010),  # 0-2 control steps around the 2026-09-16 calibrated ONE step (sim cross-corr
                                       # lag == robot's 30-35 ms) -> ~20-40 ms effective so the policy learns real
                                       # margin. (0.015-0.028 before: double-counted the sim's own ~20 ms -> 64 ms
                                       # effective and a 4 Hz thrash the robot never shows; see rider_params)
    "deadband_frac":  (0.05, 0.15),    # breakaway about the floor-sweep value 0.08 (2026-10-19; was (0.10, 0.22)
                                       # about the 2026-09-16 cruise-fit 0.15). Wide: stick-slip varies.
    "mass_scale":     (0.85, 1.15),    # mass measured -> tighter
    "wheel_friction": (0.6, 1.5),      # real-floor traction unknown -> wide
    "accel_couple":   (-1.1, -0.6),    # accelerometer-reads-acceleration gain (sim frame) about the MEASURED -0.85
                                       # (2026-10-19 recorder fits +0.66..+0.97 robot frame); see rider_params
}

CONTROL_RATE_HZ = (180.0, 260.0)   # per-episode control-rate DR. MEASURED 2026-09-16 after the wheel-read
                                   # timeout fix: 234 Hz idle, ~199 Hz driving (was 143-166 Hz before it).
                                   # Randomize so the policy is rate-ROBUST instead of tuned to one rate.


class FirmwareSensing:
    """Reproduce what esp32_rider_fw/src/main.cpp actually feeds the policy (2026-09-16 delay budget).

    The old sim gave the policy CLEAN physics state; the firmware gives it:
      * velocity  = LP(fw_pol_vel_lp) of d(wheel_x)/dt, where wheel_x is the 10-bit ENCODER position
                    (wheel relative to the body -- NOT ground speed; it omits the r*pitch_rate term)
      * wheel_vel = that same filtered signal / r  (the firmware feeds xvelP/POL_WHEEL_R)
      * pitch rate = LP(fw_rate_lp) of the gyro
      * pitch     = the firmware's complementary filter: accelerometer pitch (which ALSO reads the chassis
                    acceleration, accel_couple * atan(a/g)) blended at 1/(|gyro deg/s|+200) per cycle into the
                    gyro-integrated pitch (2026-10-19; before, the policy saw the true pitch)
    Per-cycle coefficients (they scale with the control rate), exactly as the firmware runs them.
    All ops are linear or odd-symmetric (round), so mirror_aug stays exact.
    """

    def __init__(self, p: RiderParams):
        self.p = p
        self.q_step = 2.0 * np.pi / p.encoder_counts_per_rev
        self.couple = p.accel_couple   # per-episode (env DR overrides it)
        self.reset(0.0)

    def reset(self, wheel_angle: float, pitch: float = 0.0, x_vel: float = 0.0):
        self.prev_x = self._enc_x(wheel_angle)
        self.xvelP = 0.0          # firmware xvelP (polInit zeroes it)
        self.rate_f = 0.0         # firmware dqf
        self.xvel_raw = 0.0       # last differenced encoder velocity (drive loop's input)
        self.theta = pitch        # firmware theta (primed from the accelerometer; still at reset)
        self.prev_xv = x_vel      # chassis velocity at the last read (-> the accelerometer's a)

    def fuse_pitch(self, pitch: float, gyro_rate: float, x_vel: float, dt: float, acc_noise: float = 0.0):
        """The firmware's pitch fusion (main.cpp theta), radians in the sim frame. The accelerometer pitch is
        the true pitch plus couple*atan(a/g) for the chassis acceleration a (+ noise); the gyro rate is the
        raw (unfiltered) rate the firmware integrates."""
        a = (x_vel - self.prev_xv) / dt
        self.prev_xv = x_vel
        pit_acc = pitch + self.couple * np.arctan2(a, 9.81) + acc_noise
        pf = 1.0 / (abs(np.degrees(gyro_rate)) + 200.0)
        self.theta = pf * pit_acc + (1.0 - pf) * (self.theta + gyro_rate * dt)
        return self.theta

    def _enc_x(self, wheel_angle: float) -> float:
        return np.round(wheel_angle / self.q_step) * self.q_step * self.p.wheel_radius_m

    def update(self, wheel_angle: float, gyro_rate: float, dt: float):
        """Advance one control step. Returns (pitch_rate_obs, x_vel_obs, wheel_vel_obs)."""
        x = self._enc_x(wheel_angle)
        self.xvel_raw = (x - self.prev_x) / dt
        self.prev_x = x
        lp = self.p.fw_pol_vel_lp
        self.xvelP = lp * self.xvelP + (1.0 - lp) * self.xvel_raw
        rl = self.p.fw_rate_lp
        self.rate_f = rl * self.rate_f + (1.0 - rl) * gyro_rate
        return self.rate_f, self.xvelP, self.xvelP / self.p.wheel_radius_m

# --- drive-in-training loop (matches esp32_rider_fw main.cpp + drive_sim.py) ---
# When drive_train=True, each episode commands a random cruise velocity and runs the firmware's
# EXACT setpoint-tilt loop, so the policy learns to hold the driving lean at the real 34 ms delay
# (a pure balance-in-place policy has no forward-drive authority -> falls at full stick).
DRIVE_K, DRIVE_KI, DRIVE_KD = 20.0, 1.0, 10.0   # gPosHoldK / gPosHoldKi / gDriveKd (MOVING regime)
DRIVE_POSMAX_DEG = 8.0                           # gPosHoldMax (setpoint-bias clamp, deg); 5 -> 8 with drive_lead (2026-10-19)
DRIVE_AMAX  = 0.4                                 # gPosAmax  (accel cap m/s^2) -- 0.8 -> 0.4 fw default 2026-10-19
DRIVE_VMAX  = 0.30                                # gPosVmax  (full-stick speed cap m/s) -- 0.25 -> 0.30 2026-10-19
DRIVE_TRAIN_VMAX = 0.35                           # training samples cruise speeds up to this: margin over the fw cap
DRIVE_VELLP = 0.9                                 # gPosVelLP (LP on velocity feeding the D term)
# HOLD integral (main.cpp posErrInt): integrates position error while HOLDING (target settled, within
# gPosIntBand, outside the gPosDeadband), clamped to gPosHoldMax/gPosHoldKi; it cancels the policy's own
# command bias at rest. Since 2026-10-18 it is KEPT (frozen) through a stick drive. Before 2026-10-19 the
# training env never integrated it at all (always 0) -- unlike every real drive, which starts from a hold.
DRIVE_INTBAND  = 0.15                             # gPosIntBand (m)
DRIVE_DEADBAND = 0.012                            # gPosDeadband (m)
DRIVE_KILIM    = DRIVE_POSMAX_DEG / DRIVE_KI      # integral clamp (m*s)
DRIVE_HOLD_S   = (1.0, 2.5)                       # drive episodes start with a hold this long (real use: hold, then drive)
DRIVE_LEASH    = 0.25                             # drive_lead: the moving target stays within this of the robot (m)
# pitch_dis: errors in the pitch the controller is fed, as MEASURED on the floor 2026-10-19 -- a balance point moved
# by the leg height (~3 deg at the lowest legs) and a sensor drift that builds during fast turns (~2 deg).
PDIS_OFFSET_DEG = (-4.0, 4.0)                     # per-episode constant offset
PDIS_DRIFT_DEG  = (-3.0, 3.0)                     # turn-style drift amplitude (half of the drive episodes)
PDIS_DRIFT_RAMP_S = 1.5                          # it builds over this, holds 1-3 s, then fades over this
_DEG2RAD    = 0.017453292


class ActuatorModel:
    """Normalized command [-1, 1] -> wheel VELOCITY setpoint (rad/s).

    Models the measured wheel: a velocity source with ~21 ms pure transport
    latency and a ~13 ms first-order tracking lag (~34 ms total cmd->velocity,
    measured on-robot 2026-08-13). The MuJoCo velocity actuator then tracks the
    setpoint this returns.
    """

    def __init__(self, p: RiderParams, dt: float):
        self.p = p
        self.dt = dt
        self.state = 0.0
        n_lat = max(0, round(p.latency_s / dt))
        # A deque of maxlen n returns the value appended n-1 calls ago (append drops the oldest, then
        # [0] is read) -> maxlen must be n_lat+1 for n_lat steps of delay. Fixed 2026-09-16: every
        # earlier run got one step LESS pure delay than its latency_s said (0.021 s -> 16 ms, not 20).
        self._buf = deque([0.0] * (n_lat + 1), maxlen=n_lat + 1) if n_lat > 0 else None

    def reset(self):
        self.state = 0.0
        if self._buf is not None:
            self._buf.clear()
            self._buf.extend([0.0] * self._buf.maxlen)

    def __call__(self, cmd: float) -> float:
        cmd = float(np.clip(cmd, -self.p.cmd_limit, self.p.cmd_limit))
        if self._buf is not None:                      # pure latency
            self._buf.append(cmd)
            cmd = self._buf[0]
        if getattr(self.p, "stick_slip", False):
            # LOADED hysteretic stick-slip, gated on ACTUAL wheel motion (self.state from the prev step),
            # NOT the instantaneous command -> a spinning wheel rides through zero-crossings in the
            # KINETIC regime (momentum) and only re-sticks once it has genuinely stopped. From rest it
            # needs the HIGH static breakaway, then jumps discontinuously onto the kinetic line. This is
            # faithful to the loaded bench AND balanceable: hold = stick/slip pulses, drive = smooth.
            ds = self.p.stiction_static_frac
            dk = self.p.stiction_kinetic_frac
            a = abs(cmd); sgn = 1.0 if cmd >= 0.0 else -1.0
            if abs(self.state) > 0.5:                              # rad/s: wheel still spinning -> kinetic
                cmd = sgn * max(0.0, a - dk) / (1.0 - dk)          # no static wall while in motion
            elif a < ds:
                cmd = 0.0                                          # at rest, below static breakaway -> stuck
            else:
                cmd = sgn * (a - dk) / (1.0 - dk)                  # break free from rest -> jump onto kinetic line
        else:
            db = self.p.deadband_frac                  # STICTION breakaway (measured ~0.09, bench command-ladder)
            if db > 0.0:
                if abs(cmd) < db:
                    cmd = 0.0                          # below breakaway: STUCK
                else:
                    # deadband + linear: 0 at breakaway, linear to full. NO creep-jump -- the UNLOADED bench
                    # ladder (replay_actuator.py) shows a smooth line from breakaway, not a stick-slip step.
                    cmd = np.sign(cmd) * (abs(cmd) - db) / (1.0 - db)
        target = cmd * self.p.vel_max_rad_s            # velocity setpoint (rad/s)
        tau = self.p.actuator_tau_s
        if self.p.actuator_tau_moving_s is not None:   # speed-dependent lag (floor sweeps 2026-10-19, see params)
            k = min(1.0, abs(self.state) / self.p.tau_speed_ref_rad_s)
            tau = tau + k * (self.p.actuator_tau_moving_s - tau)
        if tau > 0.0:                                  # first-order tracking lag
            a = self.dt / (tau + self.dt)
            self.state += a * (target - self.state)
        else:
            self.state = target
        return self.state


class SpinActuator:
    """A SPIN in place on the 2-D sim: the firmware adds a turn command to one wheel and subtracts it from the
    other, each wheel its own actuator; the body's fore-aft motion is the MEAN of the two wheel speeds.
    gain / extra_lat / extra_tau alter the spinning wheels for experiments. NOTE (2026-10-19): a "weaker, later
    spinning wheel" was a misreading -- the floor frequency sweeps show the wheel responds the same spinning or
    straight (it is slower whenever it MOVES; see RiderParams.actuator_tau_moving_s). This 2-D spin does NOT
    reproduce ppo_ld8_s6's floor spin shimmy (4 Hz, 16-22 deg/s); the cause is still open."""

    def __init__(self, p, dt, turn, gain=1.0, extra_lat=0.0, extra_tau=0.0):
        q = replace(p, vel_max_rad_s=p.vel_max_rad_s * gain, latency_s=p.latency_s + extra_lat,
                    actuator_tau_s=p.actuator_tau_s + extra_tau)
        self.a, self.b, self.turn, self.dt = ActuatorModel(q, dt), ActuatorModel(q, dt), turn, dt
        self.spin = 0.0                                         # differential wheel angle (rad): the spin

    def reset(self):
        self.a.reset(); self.b.reset(); self.spin = 0.0

    def __call__(self, cmd):
        va, vb = self.a(cmd + self.turn), self.b(cmd - self.turn)
        self.spin += 0.5 * (va - vb) * self.dt
        return 0.5 * (va + vb)


class RiderBalanceEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, params: RiderParams = None, render_mode=None,
                 max_seconds: float = 10.0, target_x: float = 0.0, add_noise: bool = True,
                 domain_rand: bool = False, frame_stack: int = 1, pure_balance: bool = True,
                 mirror_aug: bool = False, vel_pen: float = 0.0, rate_dr: bool = True,
                 pos_anchor: float = 0.0, pos_weight: float = 0.75, rate_pen: float = 0.30,
                 drive_train: bool = False, fw_obs: bool = True, drive_lead: bool = False,
                 pitch_dis: bool = False):
        super().__init__()
        # fw_obs: feed the policy what the FIRMWARE feeds it (FirmwareSensing: encoder-differenced +
        # low-passed velocity, low-passed pitch rate) instead of clean physics state. Default ON since
        # 2026-09-16; fw_obs=False reproduces the older recipes (ppo_v_pure ... ppo_drive34_l4).
        self.fw_obs = fw_obs
        # drive_train: embed the firmware setpoint-tilt drive loop in training so the policy learns
        # forward/back drive authority (a balance-in-place policy falls at full-stick forward on HW).
        self.drive_train = drive_train
        # drive_lead: the LQR-like stick drive (2026-10-19) -- the target ADVANCES at the commanded speed and the
        # position P keeps acting (leashed at DRIVE_LEASH), instead of the target latching to wheel_x every cycle.
        self.drive_lead = drive_lead
        # pitch_dis: randomize the floor's pitch errors (moved balance point + turn drift) into the fed pitch.
        self.pitch_dis = pitch_dis
        self._dis_off = 0.0; self._dis_amp = 0.0; self._dis_t0 = 0.0; self._dis_hold = 0.0
        self._bias_rad = 0.0
        self.rate_pen = rate_pen   # temporal action-rate^2 weight (chatter suppressor); default 0.30 = deployed
        # pos_weight: position-error^2 penalty when NOT pure_balance (position-aware end-to-end policy).
        # The policy SEES x_err/x_int and learns the full position->lean->torque mapping itself, so it
        # holds its spot WITHOUT a cruise -- the proper way to a smooth non-drifting balancer. Pairs with
        # the high rate_pen below (the old position-aware attempt used 0.05 -> too twitchy on HW).
        self.pos_weight = pos_weight
        self.p = params or DEFAULT
        self.frame_stack = frame_stack
        # vel_pen: penalize chassis fwd-velocity^2. pure_balance had NO velocity term, so a
        # steady forward roll scored the same as standing still -> the policy learned a creep
        # (~0.13 m/s), VecNormalize centered on it (wheel_vel mean ~4), and that baked-in creep
        # became a constant hold-command bias on the robot (-37) that broke fwd/back drive
        # symmetry. A small penalty pins the reward optimum at STATIONARY. Keep it modest: the
        # policy sees RAW velocity (not velocity error), so a large weight makes it fight the
        # firmware's commanded drive. mirror_aug handles the directional symmetry separately.
        self.vel_pen = vel_pen
        self.rate_dr = rate_dr   # per-episode control-rate randomization (added 2026-06-16); ppo_v_pure predates it
        # pos_anchor: SMALL position^2 penalty in pure_balance. Integrated x^2 explodes for a SUSTAINED
        # cruise (the bias source) but is ~free for balance transients (small net displacement) -- so it
        # breaks the creep WITHOUT mirror_aug and WITHOUT the velocity penalty's fight against balancing.
        # The policy still doesn't SEE position (obs stays pure-balance, deployable as-is); it just learns
        # not to sustain a velocity. Replaces the cruise as the smoothness mechanism. 0 = old behavior.
        self.pos_anchor = pos_anchor
        # mirror_aug: the balancer is physically mirror-symmetric, so each episode we
        # randomly run it in a sign-flipped frame (obs and action negated). This makes the
        # training data exactly fwd/back symmetric -> VecNormalize means stay ~0 and the
        # policy learns action(-s)=-action(s). It was the missing symmetry that let the old
        # policies (ppo_v_pure et al.) drift onto the action rail asymmetrically.
        self.mirror_aug = mirror_aug
        self.mirror = 1.0
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
        self.sense = FirmwareSensing(self.p)
        self.last_sensed = (0.0, 0.0, 0.0)   # (pitch_rate_obs, x_vel_obs, wheel_vel_obs) of the latest frame
        self.max_seconds = max_seconds
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
        # leg give (RiderParams.leg_flex_stiffness_Nm_rad): body pitch = legs' pitch + the flex
        self.q_flex = self.d_flex = None
        if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "leg_flex") >= 0:
            (self.q_flex, self.d_flex) = qadr("leg_flex")

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
    # hooks the two-wheel env (rider_env_twowheel.py) overrides: how the command reaches the wheel(s), and
    # the encoder angle the firmware differences
    def _apply_action(self, a):
        self.data.ctrl[0] = self.act(a)

    def _wheel_angle(self):
        return self.data.qpos[self.q_wheel]

    def _raw_state(self):
        pitch = self.data.qpos[self.q_pitch]
        pitch_rate = self.data.qvel[self.d_pitch]
        if self.q_flex is not None:                     # the IMU is on the torso, which flexes on the legs
            pitch = pitch + self.data.qpos[self.q_flex]
            pitch_rate = pitch_rate + self.data.qvel[self.d_flex]
        x = self.data.qpos[self.q_x]
        x_vel = self.data.qvel[self.d_x]
        wheel_vel = self.data.qvel[self.d_wheel]
        return pitch, pitch_rate, x, x_vel, wheel_vel

    def _sense(self):
        """One sensor read per control step, in firmware order: BEFORE the drive loop and the policy
        obs, which both consume it (the firmware reads the wheels once, then derives xvel for the
        drive-loop bias and the policy from that same read). Stores (pitch, rate, x_vel, wheel_vel)."""
        pitch, pitch_rate, x, x_vel, wheel_vel = self._raw_state()
        acc_noise = 0.0
        if self.add_noise:
            d2r = np.pi / 180.0
            acc_noise = np.random.normal(0, self.p.accel_pitch_noise_deg * d2r)
            pitch_rate += np.random.normal(0, self.p.gyro_noise_dps * d2r)
        if self.fw_obs:
            # the accelerometer noise enters through the fusion (heavily filtered), as on the robot
            pitch = self.sense.fuse_pitch(pitch, pitch_rate, x_vel, self.ctrl_dt, acc_noise)
        else:
            pitch += acc_noise
        if self.pitch_dis:
            pitch += self._pitch_dis_now()
        if self.fw_obs:
            # What the firmware feeds the policy: encoder-differenced + LP'd velocity, LP'd gyro rate.
            # (The 2026-06 'qnoise' experiment trained on quantized-but-UNfiltered velocity and hurt
            # balance; this is the filtered signal the robot really uses -- see docs/loop_delay_budget.md.)
            pitch_rate, x_vel, wheel_vel = self.sense.update(self._wheel_angle(), pitch_rate, self.ctrl_dt)
        self.last_sensed = (pitch, pitch_rate, x_vel, wheel_vel)

    def _single_obs(self):
        pitch, pitch_rate, x_vel, wheel_vel = self.last_sensed
        x = self._raw_state()[2]                         # forward travel (2-D: slide_x; two-wheel: wheel odometry)
        xerr = 0.0 if self.pure_balance else (x - self.target_x)   # firmware zeros these (pos in code)
        xint = 0.0 if self.pure_balance else self._x_int
        # drive_train: the pitch obs is setpoint-RELATIVE (pitch + drive-loop bias), exactly like the
        # firmware feeds pr_pitch = biasDeg - pitch. bias_rad is 0 when not drive_train (pure balance).
        o = np.array([pitch + self._bias_rad, pitch_rate, xerr, x_vel, wheel_vel,
                      self._pitch_int, xint], np.float32)
        return (self.mirror * o).astype(np.float32)   # all 7 obs are odd under the mirror

    def _drive_bias(self):
        """Run the firmware setpoint-tilt loop (main.cpp / drive_sim.py) for one control step and
        store the pitch-obs bias. No-op (bias 0) unless drive_train. Called each step post-physics."""
        if not self.drive_train:
            self._bias_rad = 0.0
            return
        dt = self.ctrl_dt
        x = float(self._raw_state()[2])                        # forward travel, not world x (the two-wheel env turns)
        if self._steps >= self._drive_start_step:              # hold first, then the stick goes over
            self._vdes_cmd = self._drive_cmd
        dv = DRIVE_AMAX * dt                                   # accel-cap the commanded velocity ramp
        if self._vdes_cmd > self._vdes_s + dv:   self._vdes_s += dv
        elif self._vdes_cmd < self._vdes_s - dv: self._vdes_s -= dv
        else:                                    self._vdes_s = self._vdes_cmd
        self._tgt_eff += self._vdes_s * dt
        # the firmware's drive loop differences the same ENCODER wheel_x the policy's velocity comes from
        xvel = self.sense.xvel_raw if self.fw_obs else (x - self._prevX) / dt
        self._prevX = x
        self._xvelF = DRIVE_VELLP * self._xvelF + (1.0 - DRIVE_VELLP) * xvel
        if abs(self._vdes_cmd) > 0.001 and self.drive_lead:
            # LQR-LIKE STICK DRIVE: the target advanced at the ramped speed above; the robot is held to it by
            # the position P (dragged along if more than DRIVE_LEASH away); the hold integral stays frozen.
            self._tgt_eff = float(np.clip(self._tgt_eff, x - DRIVE_LEASH, x + DRIVE_LEASH))
            posErr = x - self._tgt_eff
        elif abs(self._vdes_cmd) > 0.001:
            # STICK DRIVE (main.cpp): the target tracks wheel_x every cycle (posErr == 0) and the hold
            # integral is FROZEN (kept, not zeroed -- fw 2026-10-18), so the lean is the velocity-D term
            # plus the integral's bias cancel. (Before 2026-09-16 the sim let the target run away and
            # leaned up to 5 deg on the P term -- authority the firmware never gives.)
            self._tgt_eff = x
            posErr = 0.0
        else:
            posErr = x - self._tgt_eff
            inband = abs(posErr) < DRIVE_DEADBAND
            if abs(posErr) < DRIVE_INTBAND and not inband and abs(self._vdes_s) < 1e-6:
                self._posErrInt = float(np.clip(self._posErrInt + posErr * dt, -DRIVE_KILIM, DRIVE_KILIM))
            if inband:
                posErr = 0.0
        bias_deg = float(np.clip(DRIVE_K * posErr + DRIVE_KI * self._posErrInt
                                 + DRIVE_KD * (self._xvelF - self._vdes_s),
                                 -DRIVE_POSMAX_DEG, DRIVE_POSMAX_DEG))
        self._bias_rad = bias_deg * _DEG2RAD

    def _pitch_dis_now(self):
        """The pitch error (rad, real frame) at this step: the episode's offset + the turn-style drift profile."""
        t = self._steps * self.ctrl_dt - self._dis_t0
        r = PDIS_DRIFT_RAMP_S
        if t <= 0.0:
            k = 0.0
        elif t < r:
            k = t / r
        elif t < r + self._dis_hold:
            k = 1.0
        else:
            k = max(0.0, 1.0 - (t - r - self._dis_hold) / r)
        return (self._dis_off + k * self._dis_amp) * _DEG2RAD

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
        self.sense.couple = u(*DR_RANGES["accel_couple"])

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        if self.domain_rand:
            if self.rate_dr:
                chz = self.np_random.uniform(*CONTROL_RATE_HZ)   # randomize the loop rate this episode
                self.decim = max(1, round((1.0 / chz) / self.model.opt.timestep))
                self.ctrl_dt = self.decim * self.model.opt.timestep
                self.max_steps = int(self.max_seconds / self.ctrl_dt)
            self._randomize()                                    # rebuilds ActuatorModel with the new ctrl_dt
        # small random initial lean; starts resting on the wheel (chassis pos puts wheel on floor)
        # realistic "let go from a near-still hold": small lean, small pitch rate.
        self.data.qpos[self.q_pitch] = self.np_random.uniform(-0.06, 0.06)
        self.data.qvel[self.d_pitch] = self.np_random.uniform(-0.20, 0.20)
        mujoco.mj_forward(self.model, self.data)
        self.act.reset()
        self.sense.reset(float(self._wheel_angle()), float(self.data.qpos[self.q_pitch]),
                         float(self.data.qvel[self.d_x]))
        self._steps = 0
        self._prev_a = 0.0
        self._pitch_int = 0.0
        self._x_int = 0.0
        self._bias_rad = 0.0
        if self.drive_train:
            # 40% HOLD (vdes=0), 60% DRIVE at a random speed up to DRIVE_TRAIN_VMAX (margin over the fw
            # cap), both signs -- so the policy learns balance AND driving. Drive episodes HOLD first for
            # DRIVE_HOLD_S (the hold integral builds, as on the robot), then the stick goes over: the
            # start from a standstill is where the 2026-10-19 policy falls happened.
            x0 = float(self._raw_state()[2])
            self._drive_cmd = 0.0 if self.np_random.uniform() < 0.4 else \
                float(self.np_random.uniform(-DRIVE_TRAIN_VMAX, DRIVE_TRAIN_VMAX))
            self._drive_start_step = int(self.np_random.uniform(*DRIVE_HOLD_S) / self.ctrl_dt)
            self._vdes_cmd = 0.0
            self._tgt_eff = x0; self._vdes_s = 0.0; self._xvelF = 0.0; self._prevX = x0; self._posErrInt = 0.0
        if self.pitch_dis:
            u = self.np_random.uniform
            self._dis_off = u(*PDIS_OFFSET_DEG)
            drive_ep = self.drive_train and abs(self._drive_cmd) > 0.0
            self._dis_amp = u(*PDIS_DRIFT_DEG) if (drive_ep and u() < 0.5) else 0.0
            self._dis_t0 = (self._drive_start_step * self.ctrl_dt if drive_ep else 0.0) + u(0.0, 3.0)
            self._dis_hold = u(1.0, 3.0)
        self.mirror = (1.0 if self.np_random.uniform() < 0.5 else -1.0) if self.mirror_aug else 1.0
        self._sense()
        s = self._single_obs()
        self._stack.clear()
        for _ in range(self.frame_stack):
            self._stack.append(s)
        return self._obs(), {}

    def step(self, action):
        a = float(np.asarray(action).flat[0])          # action in the (possibly mirrored) policy frame
        self._apply_action(self.mirror * a)            # apply in the real frame -> symmetric training data
        for _ in range(self.decim):
            if self.q_flex is not None and self.p.leg_play_deg > 0.0:
                # FREE PLAY in the legs: no spring force within +/-play, the spring only beyond it (backlash)
                q = self.data.qpos[self.q_flex]; b = np.radians(self.p.leg_play_deg)
                self.data.qfrc_applied[self.d_flex] = -self.p.leg_play_stiffness_Nm_rad * (q - np.clip(q, -b, b))
            mujoco.mj_step(self.model, self.data)
        self._steps += 1
        pitch, pitch_rate, x, x_vel, wheel_vel = self._raw_state()
        self._sense()                                  # one sensor read (firmware order: read, then drive loop, then policy)
        self._drive_bias()                             # advance the setpoint-tilt loop from the new state
        fell = abs(pitch) > 0.40                       # ~23 deg (real fall, setpoint-independent)
        upright = np.cos(pitch + self._bias_rad)       # reward HOLDING the (possibly tilted) drive setpoint
        # pure_balance: NO position objective (code does position). Heavy command-
        # smoothness to kill the on-robot shimmy (jerky velocity cmd = jerky cart accel).
        pos_pen = (self.pos_anchor if self.pure_balance else self.pos_weight) * (x - self.target_x) ** 2
        act_pen = 0.02 * a ** 2                        # bumped 0.01->0.02: discourage riding the action rail
        rate_pen = self.rate_pen * (a - self._prev_a) ** 2   # temporal smoothness pressure (tunable; 0.30 = deployed)
        vel_pen = self.vel_pen * x_vel ** 2            # pin optimum at STATIONARY (kills the trained creep)
        reward = upright - pos_pen - act_pen - rate_pen - vel_pen
        if fell:
            reward -= 10.0
        self._prev_a = a
        # accumulate integral state (clamped to prevent windup)
        ci, cx = self._int_clip
        p_fed = self.last_sensed[0] if self.fw_obs else pitch   # the robot's polIpitch integrates the FUSED pitch
        self._pitch_int = float(np.clip(self._pitch_int + (p_fed + self._bias_rad) * self.ctrl_dt, -ci, ci))
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
