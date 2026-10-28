"""The Rider's LQR as the firmware runs it, on the model (the Hover's sim/closed_loop.py idea, rewritten for
the Rider's firmware: firmware/esp32_rider_fw/src/main.cpp, 2026-10-28).

    python3 sim/lqr/closed_loop.py               # 5 s standing: the model's sway

What is copied, per control cycle (CYCLE_S, the loop at ~200 Hz while balancing):
  * wheels read first (both, ~READ_S): the encoder position (1024 counts a turn: x - r th, the wheel against
    the body), the servo's speed register (327.6 counts per m/s, the same wheel-against-body speed),
    each speed smoothed LP_VEL 0.85 per cycle, x 0.828 into 'scaled' units (main.cpp odomUpdate)
  * then the IMU: the accelerometer IMU_H above the axle sees gravity plus its own acceleration; the
    firmware takes off 60 mm of lever arm (imucomp -0.5 x KIMU's 120 mm: ay -= c*alpha_f, az += c*w^2,
    alpha_f the gyro's per-cycle derivative smoothed 0.8/0.2); pit_acc = atan2(ay, az); the pitch is
    the complementary filter, accelerometer weight 1/(|rate| + 200) per cycle; dqf = 0.6 dqf + 0.4 rate.
    'up' is what the IMU reads with the centre of mass over the axle (it varies by day: 3.50 on 10-19,
    ~2.8 on 10-28); the gyro has no offset here
  * the LQR (gCtrl 1, standing): pitch = theta - 3.55 (gSetpoint), x = wheel_x - home, its integral
    (x 1 s, clamped +/-3), v = the smoothed speed (observer off, input filter off):
      u = -(-500 x - 30 Ix - 6 v + 32 pitch + 3 dqf); |u| > 20 gets 10 more; clipped +/-400
  * the command goes out COMMAND_S into the cycle (wheel reads + IMU + compute) and holds until the next;
    the firmware's sweep ('chirp': f0 -> f1 log over dur, 0.5 s fades) is added to it
Sensor noise MEASURED 2026-10-28 lying still (drop_tests.csv): gyro ~0.06 deg/s (its resolution),
accelerometer angle 0.10-0.15 deg.

The model's u is the firmware's (+ forward, model.py). Recorded like the robot's chirp capture (every 2nd
cycle: inj, cmdL, cmdR, wx_01mm, th_x100, rate_x10), so freq_response.py treats both the same.
"""
import numpy as np

from model import G, Params, accel, encoder, step

CYCLE_S = 0.005          # 200 Hz (telemetry lhz 201 balancing, 10-28)
READ_S = 0.0025          # wheel reads before the IMU (both wheels; rdms worst 2.9 ms)
COMMAND_S = 0.0027       # + the compute
SUB_S = 0.00025          # physics step
IMU_H = 0.098            # m, IMU above the axle (Marc's ruler 100, imu_position_fit 96-98)
COMP_M = -0.5 * 0.12     # firmware lever-arm correction, m (imucomp x KIMU)
SET_DEG = 3.55           # gSetpoint
K_TODAY = dict(x=-500.0, xi=-30.0, vx=-6.0, q=32.0, dq=3.0)
IX_MAX = 3.0
FF, FF_BAND, U_MAX = 10.0, 20.0, 400.0
LP_VEL = 0.85
SPEED_COUNTS = 327.6     # speed register counts per m/s
SCALED_PER_COUNT = 0.828 # 4.5 x K_V
POS_STEP = np.pi * 0.06 / 1024


class Firmware:
    def __init__(self, k=None, up_deg=2.8, rng=None, gyro_noise=0.06, acc_noise=0.02):
        self.k = dict(K_TODAY if k is None else k)
        self.up = up_deg
        self.rng = rng or np.random.default_rng(0)
        self.gn, self.an = gyro_noise, acc_noise
        self.theta = None
        self.dqf = 0.0
        self.om_prev = 0.0
        self.alpha_f = 0.0
        self.vel_s = 0.0
        self.wheel_x = 0.0
        self.home = None
        self.ix = 0.0

    def read_wheels(self, p, s):
        x_enc, v_enc = encoder(p, s)
        self.wheel_x = np.round(x_enc / POS_STEP) * POS_STEP
        raw = np.round(v_enc * SPEED_COUNTS)
        self.vel_s = LP_VEL * self.vel_s + (1 - LP_VEL) * SCALED_PER_COUNT * raw

    def read_imu(self, s, acc, dt):
        x, v, th, w = s
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

    def control(self, dt):
        if self.home is None:
            self.home = self.wheel_x
        k = self.k
        x = self.wheel_x - self.home
        self.ix = float(np.clip(self.ix + x * dt, -IX_MAX, IX_MAX))
        pitch = self.theta - SET_DEG
        u = -(k["x"] * x + k["xi"] * self.ix + k["vx"] * self.vel_s + k["q"] * pitch + k["dq"] * self.dqf)
        if u > FF_BAND:
            u += FF
        elif u < -FF_BAND:
            u -= FF
        return float(np.clip(u, -U_MAX, U_MAX))


def chirp(t, a, f0, f1, dur):
    """The firmware's sweep, as a function of time since it started (phase integrated exactly)."""
    if t < 0 or t >= dur:
        return 0.0
    k = np.log(f1 / f0) / dur
    ph = 2 * np.pi * f0 * (np.exp(k * t) - 1) / k
    env = min(1.0, t / 0.5, (dur - t) / 0.5)
    return a * env * np.sin(ph)


def run(p=None, fw=None, T=5.0, sweep=None, sweep_at=3.0, th0_deg=0.5):
    """Simulate T s standing; sweep = (a, f0, f1, dur) starts at sweep_at. Returns the recorded rows (every 2nd
    cycle, as the robot's capture) and whether it fell."""
    p = p or Params()
    fw = fw or Firmware()
    s = np.array([0.0, 0.0, np.radians(th0_deg), 0.0])
    u = 0.0
    rows = []
    n_sub = int(round(CYCLE_S / SUB_S))
    i_read, i_cmd = int(round(READ_S / SUB_S)), int(round(COMMAND_S / SUB_S))
    for c in range(int(T / CYCLE_S)):
        t = c * CYCLE_S
        inj = chirp(t + READ_S - sweep_at, *sweep) if sweep else 0.0
        for k in range(n_sub):
            if k == 0:
                fw.read_wheels(p, s)
            if k == i_read:
                fw.read_imu(s, accel(p, s, u), CYCLE_S)
            if k == i_cmd:
                u_fw = fw.control(CYCLE_S)
                u = u_fw + inj
            s = step(p, s, u, SUB_S)
        if abs(s[2]) > np.radians(45):
            return np.array(rows), True
        if c % 2 == 0:
            rows.append((t * 1e6, inj, u, -u, fw.wheel_x * 1e4, fw.theta * 100, np.degrees(fw.om_prev) * 10))
    return np.array(rows), False


if __name__ == "__main__":
    rows, fell = run(T=5.0)
    th = rows[:, 5] / 100
    print(f"5 s standing, today's gains: {'FELL' if fell else 'up'}; pitch mean {th[200:].mean():.2f} deg, "
          f"sd {th[200:].std():.3f} deg; wheel sd {rows[200:, 4].std() / 10:.2f} mm; command sd {rows[200:, 2].std():.1f}")
