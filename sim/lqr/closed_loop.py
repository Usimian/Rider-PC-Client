"""The Rider's LQR as the firmware runs it, on the model (the Hover's sim/closed_loop.py idea, rewritten for
the Rider's firmware: firmware/esp32_rider_fw/src/main.cpp, 2026-10-28).

    python3 sim/lqr/closed_loop.py               # 5 s standing: the model's sway

What is copied, per control cycle (its period and the moments inside it: fw_timing.py, per firmware version --
FW_NOW_LQR by default, the robot as it runs now; replays of old recordings name the timing they ran under):
  * wheels read first (at timing.wheels_s): the encoder position (1024 counts a turn: x - r th, the wheel against
    the body), the servo's speed register (327.6 counts per m/s, the same wheel-against-body speed),
    each speed smoothed LP_VEL 0.85 per cycle, x 0.828 into 'scaled' units (main.cpp odomUpdate)
  * then the IMU (at timing.imu_s; with timing.imu_loop the IMU loop's latest sample, taken on its own clock): the
    accelerometer IMU_H above the axle sees gravity plus its own acceleration; the
    firmware takes off 60 mm of lever arm (imucomp -0.5 x KIMU's 120 mm: ay -= c*alpha_f, az += c*w^2,
    alpha_f the gyro's per-cycle derivative smoothed 0.8/0.2); pit_acc = atan2(ay, az); the pitch is
    the complementary filter, accelerometer weight 1/(|rate| + 200) per cycle; dqf = 0.6 dqf + 0.4 rate.
    'up' is what the IMU reads with the centre of mass over the axle (it varies by day: 3.50 on 10-19,
    ~2.8 on 10-28); the gyro has no offset here
  * the LQR (gCtrl 1, standing): pitch = theta - 3.55 (gSetpoint), x = wheel_x - home, its integral
    (x 1 s, clamped +/-3), v = the smoothed speed (observer off, input filter off):
      u = -(-500 x - 30 Ix - 6 v + 32 pitch + 3 dqf); |u| > 20 gets 10 more; clipped +/-400
  * the command goes out at timing.command_s (wheel reads + IMU + compute + write) and holds until the next;
    the firmware's sweep ('chirp': f0 -> f1 log over dur, 0.5 s fades) is added to it; the law's dt is the
    period the firmware measures at the cycle's start (the previous cycle's)
Sensor noise MEASURED 2026-10-28 lying still (drop_tests.csv): gyro ~0.06 deg/s (its resolution),
accelerometer angle 0.10-0.15 deg.

The plant is model.py's rigid robot, or model_flex.py's (the body sprung on its legs) when run() is given
model_flex.FlexParams: the encoder reads the wheel against whatever carries the motors, the IMU the body.
The model's u is the firmware's (+ forward, model.py). Recorded like the robot's chirp capture (every 2nd
cycle: inj, cmdL, cmdR, wx_01mm, th_x100, rate_x10), so freq_response.py treats both the same.
"""
import numpy as np

from fw_timing import FW_NOW_LQR
from model import G, Params, plant_module

SUB_S = 0.00025          # physics step (fw_timing's periods and moments are whole steps of it)
IMU_H = 0.098            # m, IMU above the axle (Marc's ruler 100, imu_position_fit 96-98)
COMP_M = -0.5 * 0.12     # firmware lever-arm correction, m (imucomp x KIMU)
SET_DEG = 3.55           # gSetpoint
K_TODAY = dict(x=-500.0, xi=-30.0, vx=-6.0, q=32.0, dq=3.0)   # the June LQR (removed from main.cpp 2026-10-28):
                                                              # kept to read the floor recordings made under it
IX_MAX = 3.0
FF, FF_BAND, U_MAX = 10.0, 20.0, 400.0
# Friction compensation (the Hover's Phase 3 step 3, robot.cc kFric*): the measured friction in the direction
# the law pushes, blended through zero by tanh(u / FRIC_WIDTH), the direction from the command smoothed over
# FRIC_TAU; replaces the factory boost in the new law. Rider friction: model.Params fric / ku = 32.5 units.
FRIC_UNITS, FRIC_WIDTH, FRIC_TAU = 32.5, 20.0, 0.030
# Driving (main.cpp): the stick's speed is ramped (driveRamp) at POS_AMAX up / POS_DMAX braking, capped at VMAX
# (gPosVmax, 0.45 since 10-28 evening; drive_design.py explores above it with --vmax); the law's speed reference is
# VREF x vdes, its command feedforward CMD_FF x vdes.
VMAX, POS_AMAX, POS_DMAX = 0.45, 0.4, 0.8
VREF, CMD_FF = 270.0, 360.0
L2_SPEED = 327.6 * 0.828                         # LQR2's scaled units per m/s (271, main.cpp L2_SPEED_PER_MPS)
# The Hover's plan (robot.cc kPlan*): the lead limit and the plan's own position integral (command units)
PLAN_MAX_LEAD_M, PLAN_KI, PLAN_IMAX = 0.15, 100.0, 100.0
HOME_K, HOME_VMAX = 2.0, 0.15                     # main.cpp LQR_HOME_*: the held home's gentle return (Marc, 10-28)
# The Hover's IMU path (its boards/common/imu.cc, run per ~2.5 ms sample), for LQR2 verbatim (hover_sample()).
HOVER_DQ_TAU = -0.0025 / np.log(0.6)             # 4.9 ms, the Hover's rate smoothing in time (for reference)
# The IMU loop (main.cpp imuTask, since 10-28 12:44, like the Hover's imu_task: read, then vTaskDelay 2 ticks): a
# sample every HOVER_IMU_PERIOD_S on its own clock; the LQR's Hover path takes it per sample, the balance loop the
# latest one. ASSUMED 2.36 ms: vTaskDelay(2) of 1 ms ticks waits 1-2 ticks, so the real period is likely 1.4-2.4 ms;
# no telemetry reports it yet.
IMU_READ_S = 0.00036                              # 12 bytes at 400 kHz I2C
HOVER_IMU_PERIOD_S = 0.002 + IMU_READ_S           # the Hover: "read every ~2-3 ms"
LQR2_UP_DEG = 2.8                                 # LQR2's pitch reference (gL2Up): the measured balance point
LP_VEL = 0.85
SPEED_COUNTS = 327.6     # speed register counts per m/s
SCALED_PER_COUNT = 0.828 # 4.5 x K_V
POS_STEP = np.pi * 0.06 / 1024
R_WHEEL = 0.030


class Firmware:
    def __init__(self, k=None, up_deg=2.8, rng=None, gyro_noise=0.06, acc_noise=0.02, corrected=False, a_v=LP_VEL,
                 fric_comp=False, plan=None, hover_imu=False, tau_v=None):
        """corrected: the encoders' body part taken off (the new law, lqr_design.py): position wheel_x + r*pitch,
        speed register + r*rate before its smoothing a_v (per cycle; today LP_VEL 0.85). plan: drive by the
        Hover's plan (drive_design.py: a_max, t_jerk, gain, th_v, th_a, u_v, u_a) instead of the LQR's drive
        handling; it implies corrected readings and uses its own integral (k['xi'] unused). hover_imu: the law
        reads the Hover's IMU path (theta2, dq2: hover_sample(), twice a cycle) against LQR2_UP_DEG and corrects
        the speed with its smoothed rate, as the Hover's robot.cc does; the shared estimate (theta, dqf, against
        SET_DEG) is what the other controllers read."""
        self.k = dict(K_TODAY if k is None else k)
        self.corrected, self.a_v, self.tau_v = corrected, a_v, tau_v   # tau_v (s): smoothing in time, as main.cpp
        self.fric_comp, self.u_lp = fric_comp, 0.0
        self.rate = 0.0
        self.up = up_deg
        self.rng = rng or np.random.default_rng(0)
        self.gn, self.an = gyro_noise, acc_noise
        self.theta = None
        self.dqf = 0.0
        self.om_prev = 0.0
        self.alpha_f = 0.0
        self.vel_s = 0.0
        self.wheel_x = 0.0
        self.ix = 0.0
        self.tgt = self.tgt_eff = None           # the LQR drive handling's home and moving target (m)
        self.vdes, self.was_vel = 0.0, False
        self.plan = plan
        self.hover_imu = hover_imu
        self.theta2, self.dq2 = None, 0.0
        self.plan_x = None                       # the plan's position, speed, acceleration, smoothed speed, integral
        self.v_r = self.a_r = self.v_r_f = self.plan_i = 0.0
        if plan is not None:
            self.corrected = True

    def read_wheels(self, p, s):
        x_enc, v_enc = plant_module(p).encoder(p, s)
        self.wheel_x = np.round(x_enc / POS_STEP) * POS_STEP
        self.raw_v = SCALED_PER_COUNT * np.round(v_enc * SPEED_COUNTS)   # filtered in control(), after the IMU

    def read_imu(self, th, w, acc, dt):
        """The IMU on the body: its angle th and rate w (+ forward lean), acc = (x'', its angular acceleration)."""
        xdd, thdd = acc
        # specific force in body axes (forward, up) at the IMU, IMU_H up the body's axis, forward lean th
        F = -G * np.sin(th) + thdd * IMU_H + xdd * np.cos(th) + self.rng.normal(0, self.an)
        U = G * np.cos(th) - w * w * IMU_H + xdd * np.sin(th) + self.rng.normal(0, self.an)
        rate = np.round((-np.degrees(w) + self.rng.normal(0, self.gn)) * 16.4) / 16.4   # firmware frame, deg/s
        om = np.radians(rate)
        alpha = (om - self.om_prev) / dt
        self.om_prev = om
        self.alpha_f = 0.8 * self.alpha_f + 0.2 * alpha
        # the firmware's frame: ay = F, its alpha = minus ours; ay -= c*alpha_f, az += c*w^2
        ayc = F - COMP_M * self.alpha_f
        azc = U + COMP_M * om * om
        pit_acc = np.degrees(np.arctan2(ayc, azc)) + self.up
        if self.theta is None:
            self.theta = pit_acc
        wgt = 1.0 / (abs(rate) + 200.0)
        if abs(pit_acc) < 100 and abs(rate) < 1000:
            self.theta = wgt * pit_acc + (1 - wgt) * (self.theta + rate * dt)
        self.dqf = 0.6 * self.dqf + 0.4 * rate
        self.rate = rate

    def hover_sample(self, th, w, acc, dts):
        """One sample of the Hover's IMU path (boards/common/imu.cc), verbatim: dts = time since the last."""
        xdd, thdd = acc
        F = -G * np.sin(th) + thdd * IMU_H + xdd * np.cos(th) + self.rng.normal(0, self.an)
        U = G * np.cos(th) - w * w * IMU_H + xdd * np.sin(th) + self.rng.normal(0, self.an)
        g = np.round((-np.degrees(w) + self.rng.normal(0, self.gn)) * 16.4) / 16.4
        acc_pitch = np.degrees(np.arctan(F / U)) + self.up      # plain angle, no lever correction
        if self.theta2 is None:
            self.theta2 = acc_pitch
        self.dq2 = 0.6 * self.dq2 + 0.4 * g
        wgt = 1.0 / (abs(g) + 200.0)
        if abs(acc_pitch) < 100 and abs(g) < 1000:
            self.theta2 = wgt * acc_pitch + (1 - wgt) * (self.theta2 + g * dts)

    def drive_today(self, dt, stick):
        """The LQR's drive handling as main.cpp has it (both LQR and LQR2): the stick speed ramps (driveRamp,
        POS_AMAX up / POS_DMAX braking), the target advances at it; on release the home is where the brake ramp
        stops (releaseStop) and the integral clears; holding, the target brakes onto it. Returns vdes, m/s."""
        if stick != 0.0:
            self.vdes = drive_ramp(self.vdes, float(np.clip(stick, -VMAX, VMAX)), dt)
            self.tgt_eff += self.vdes * dt
            self.tgt = self.tgt_eff
            self.was_vel = True
        else:
            if self.was_vel:
                self.tgt = self.tgt_eff + np.sign(self.vdes) * self.vdes ** 2 / (2 * POS_DMAX)
                self.ix = 0.0
                self.was_vel = False
            dist = self.tgt - self.tgt_eff
            vcap = min(VMAX, np.sqrt(2 * POS_DMAX * abs(dist)))
            self.vdes = drive_ramp(self.vdes, vcap if dist >= 0 else -vcap, dt)
            self.tgt_eff += self.vdes * dt
            if (dist >= 0 and self.tgt_eff > self.tgt) or (dist < 0 and self.tgt_eff < self.tgt):
                self.tgt_eff, self.vdes = self.tgt, 0.0
        return self.vdes

    def control(self, dt, stick=0.0):
        """One cycle of the balance law; stick = the drive command (m/s, the firmware's gDriveVel)."""
        k = self.k
        v = self.raw_v
        pitch = (self.theta2 - LQR2_UP_DEG) if self.hover_imu else (self.theta - SET_DEG)
        self.dq_law = self.dq2 if self.hover_imu else self.dqf
        if self.corrected:                       # firmware frame: forward lean makes pitch and rate negative
            rr = self.dq2 if self.hover_imu else self.rate   # the Hover corrects with its smoothed rate
            v -= R_WHEEL * np.radians(rr) * SPEED_COUNTS * SCALED_PER_COUNT
        if self.tau_v:
            self.a_v = float(np.exp(-dt / self.tau_v))
        self.vel_s = self.a_v * self.vel_s + (1 - self.a_v) * v
        if self.plan is not None:
            return self._control_plan(dt, stick, pitch)
        if self.tgt is None:
            self.tgt = self.tgt_eff = self.wheel_x
        vdes = self.drive_today(dt, stick)
        x = self.wheel_x - self.tgt_eff
        self.ix = float(np.clip(self.ix + x * dt, -IX_MAX, IX_MAX))
        if self.corrected:
            x -= R_WHEEL * np.radians(pitch)
        u = -(k["x"] * x + k["xi"] * self.ix + k["vx"] * (self.vel_s - VREF * vdes) + k["q"] * pitch
              + k["dq"] * self.dq_law) + CMD_FF * vdes
        return self._output(u, dt)

    def _control_plan(self, dt, stick, pitch):
        """The Hover's driving plan (robot.cc plan_step, its sim/closed_loop.py _control_plan), on the corrected
        readings: the planned speed closes on the stick's at up to a_max, its acceleration building over t_jerk;
        its position never gets more than PLAN_MAX_LEAD_M from the axle; the law tracks the plan's position,
        smoothed speed and lean, the plan's command is fed forward, and the plan's own integral always runs."""
        R, k = self.plan, self.k
        x = self.wheel_x - R_WHEEL * np.radians(pitch)                 # the axle (firmware frame)
        if self.plan_x is None:                                        # plan_start: here, at the measured speed
            self.plan_x, self.v_r, self.a_r = x, self.vel_s / L2_SPEED, 0.0
            self.v_r_f, self.plan_i = self.v_r, 0.0
            self.home, self.stick_held = x, False
        xerr = x - self.plan_x                                         # before the plan's step, as the Hover
        held = abs(stick) > 0.001                                      # main.cpp: the home is held (Marc, 10-28)
        if self.stick_held and not held:
            self.home = x                                              # the stick released: hold this spot
        self.stick_held = held
        target = float(np.clip(stick, -VMAX, VMAX))
        if not held:                                                   # released: back home, gently (main.cpp)
            target = float(np.clip(HOME_K * (self.home - self.plan_x), -HOME_VMAX, HOME_VMAX))
        a_des = np.clip(R["gain"] * (target - self.v_r), -R["a_max"], R["a_max"])
        jerk = R["a_max"] / R["t_jerk"]
        self.a_r += np.clip(a_des - self.a_r, -jerk * dt, jerk * dt)
        self.v_r += self.a_r * dt
        self.plan_x = float(np.clip(self.plan_x + self.v_r * dt, x - PLAN_MAX_LEAD_M, x + PLAN_MAX_LEAD_M))
        self.v_r_f = self.a_v * self.v_r_f + (1 - self.a_v) * self.v_r
        lean_fw = -np.degrees(R["th_v"] * self.v_r + R["th_a"] * self.a_r)   # forward lean is negative pitch
        u_ff = R["u_v"] * self.v_r + R["u_a"] * self.a_r
        self.plan_i = float(np.clip(self.plan_i + PLAN_KI * xerr * dt, -PLAN_IMAX, PLAN_IMAX))
        u = -(k["x"] * xerr + k["vx"] * (self.vel_s - L2_SPEED * self.v_r_f) + k["q"] * (pitch - lean_fw)
              + k["dq"] * self.dq_law) + self.plan_i + u_ff
        return self._output(u, dt)

    def _output(self, u, dt):
        if self.fric_comp:
            a = np.exp(-dt / FRIC_TAU)
            self.u_lp = a * self.u_lp + (1 - a) * u
            u += FRIC_UNITS * np.tanh(self.u_lp / FRIC_WIDTH)
        elif u > FF_BAND:
            u += FF
        elif u < -FF_BAND:
            u -= FF
        return float(np.clip(u, -U_MAX, U_MAX))


def drive_ramp(v, tgt, dt):
    """main.cpp driveRamp: POS_DMAX while the speed falls (never through zero), POS_AMAX while it rises."""
    braking = (v > 0 and tgt < v) or (v < 0 and tgt > v)
    d = (POS_DMAX if braking else POS_AMAX) * dt
    n = v + d if tgt > v + d else (v - d if tgt < v - d else tgt)
    if braking and ((v > 0 and n < 0) or (v < 0 and n > 0)):
        n = 0.0
    return n


def chirp(t, a, f0, f1, dur):
    """The firmware's sweep, as a function of time since it started (phase integrated exactly)."""
    if t < 0 or t >= dur:
        return 0.0
    k = np.log(f1 / f0) / dur
    ph = 2 * np.pi * f0 * (np.exp(k * t) - 1) / k
    env = min(1.0, t / 0.5, (dur - t) / 0.5)
    return a * env * np.sin(ph)


class ModelPlant:
    """run()'s default plant: model.py (or model_flex.py) integrated by RK4 at SUB_S. Another plant (the RL sim's
    MuJoCo one, sim/lqr_in_rl_sim.py) offers the same: p, s (axle x, v, then the IMU body's angle and rate at B, B + 1;
    what read_wheels reads), accel_body(u), step(u, dt), kick(rad/s)."""

    def __init__(self, p, th0):
        self.p, self.m = p, plant_module(p)
        self.B = self.m.BODY
        self.s = self.m.initial(th0)

    def accel_body(self, u):
        return self.m.accel_body(self.p, self.s, u)

    def step(self, u, dt):
        self.s = self.m.step(self.p, self.s, u, dt)

    def kick(self, dw):
        self.s[self.B + 1] += dw


def run(p=None, fw=None, T=5.0, sweep=None, sweep_at=3.0, th0_deg=0.5, kick=None, drive=None, log_all=False,
        plant=None, timing=FW_NOW_LQR, seed=0):
    """Simulate T s standing; sweep = (a, f0, f1, dur) starts at sweep_at. Returns the recorded rows (every 2nd
    cycle, as the robot's capture) and whether it fell. kick = (t, deg/s): a shove, the body's pitch rate
    changed at once (+ forward). drive = (times s, m/s): the stick's speed command, held between the times.
    log_all: return every cycle (t, stick, u, axle speed m/s, body lean deg + forward) instead of the capture rows.
    plant: the plant (ModelPlant(p) by default, see there). timing: the firmware's (fw_timing.py; the robot as it is
    now by default -- a replay of an old recording names its own); seed: the draw of each cycle's period."""
    p = p or Params()
    fw = fw or Firmware()
    plant = plant or ModelPlant(p, np.radians(th0_deg))
    rng = np.random.default_rng(seed)
    B = plant.B
    u = 0.0
    rows = []
    hov = getattr(fw, "hover_imu", False)
    imu_clock = hov or timing.imu_loop                  # samples on the IMU loop's own clock
    i_wheels, i_read, i_cmd = (int(round(x / SUB_S)) for x in (timing.wheels_s, timing.imu_s, timing.command_s))
    fixed = len(timing.periods_s) == 1
    next_hover = 0.0
    last_imu = None                                     # the IMU loop's latest sample (body angle, rate, accelerations)
    t, c = 0.0, 0
    prev = timing.period(rng)                           # the period the firmware measures at a cycle's start
    n_fixed = int(T / timing.periods_s[0]) if fixed else None   # (a fixed period: as before 10-29, exactly)
    while True:
        cyc = timing.period(rng)
        if fixed:
            if c >= n_fixed:
                break
            t = c * cyc
        elif t + cyc > T + 1e-9:
            break
        n_sub = int(round(cyc / SUB_S))
        inj = chirp(t + timing.imu_s - sweep_at, *sweep) if sweep else 0.0
        stick = 0.0
        if drive is not None:
            j = np.searchsorted(drive[0], t, side="right") - 1
            stick = float(drive[1][j]) if j >= 0 else 0.0
        if kick and abs(t - kick[0]) < cyc / 2:
            plant.kick(np.radians(kick[1]))
        for k in range(n_sub):
            s = plant.s
            if imu_clock and t + k * SUB_S >= next_hover:   # the IMU loop's sample, on its own clock
                last_imu = (s[B], s[B + 1], plant.accel_body(u))
                if hov:
                    fw.hover_sample(*last_imu, HOVER_IMU_PERIOD_S)
                next_hover += HOVER_IMU_PERIOD_S
            if k == i_wheels:
                fw.read_wheels(plant.p, s)
            if k == i_read:
                if timing.imu_loop:
                    fw.read_imu(*last_imu, prev)
                else:
                    fw.read_imu(s[B], s[B + 1], plant.accel_body(u), prev)
            if k == i_cmd:
                u_fw = fw.control(prev, stick)
                u = u_fw + inj
            plant.step(u, SUB_S)
        s = plant.s
        if abs(s[B]) > np.radians(45):
            return np.array(rows), True
        if log_all:
            rows.append((t, stick, u, s[1], np.degrees(s[B])))
        elif c % 2 == 0:
            rows.append((t * 1e6, inj, u, -u, fw.wheel_x * 1e4, fw.theta * 100, np.degrees(fw.om_prev) * 10))
        prev = cyc
        c += 1
        if not fixed:
            t += cyc
    return np.array(rows), False


if __name__ == "__main__":
    rows, fell = run(T=5.0)
    th = rows[:, 5] / 100
    print(f"5 s standing, today's gains: {'FELL' if fell else 'up'}; pitch mean {th[200:].mean():.2f} deg, "
          f"sd {th[200:].std():.3f} deg; wheel sd {rows[200:, 4].std() / 10:.2f} mm; command sd {rows[200:, 2].std():.1f}")
