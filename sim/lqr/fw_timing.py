"""The balance loop's timing, per firmware version -- the one place the simulators take it from (2026-10-29).

A control cycle on the ESP32 (main.cpp balanceTask): read both wheels (left, then right), take the IMU (the IMU
loop's latest sample since 10-28 12:44; before, a fresh read), compute, write the command, then wait whole 1 ms
ticks -- so the period varies from cycle to cycle. Times are from the cycle's start; each simulated cycle draws its
period from `periods_s` with `shares`. The rate belongs to the firmware a recording ran under: replays of old
recordings name FW_2026_10_19, anything about the robot as it is now uses FW_NOW_LQR / FW_NOW_POLICY.

Regenerate the measured numbers with tools/balance/loop_timing.py (recordings + the Pi's riderlog).
"""
from dataclasses import dataclass

import numpy as np


# The IMU loop (main.cpp imuTask, since 10-28 12:44, like the Hover's imu_task: read, then vTaskDelay 2 ticks): a
# sample every IMU_LOOP_PERIOD_S on its own clock. ASSUMED 2.36 ms (the 0.36 ms read + 2 ms): vTaskDelay(2) of 1 ms
# ticks waits 1-2 ticks, so the real period is likely 1.4-2.4 ms; no telemetry reports it yet.
IMU_READ_S = 0.00036                              # 12 bytes at 400 kHz I2C
IMU_LOOP_PERIOD_S = 0.002 + IMU_READ_S

# The wheels act ACTUATOR_DELAY_S after the command goes out (the servo bus and the motor driver; not in the cycle's
# timing). DERIVED 2026-10-29 (fit_tf.py): the plant's transfer function back-calculated from the two driving
# sweeps carries 7.7 ms of delay on the robot against 3.6 ms on the simulated sweeps (the cycle, the IMU sample's
# age, the filters), the poles alike (tipping +11.5 vs +11.6 /s, -36.5 vs -36.6 /s): 4.1 ms the model lacked.
ACTUATOR_DELAY_S = 0.0041


@dataclass(frozen=True)
class Timing:
    name: str
    periods_s: tuple          # the loop periods cycles take (s; whole closed_loop.SUB_S steps)
    shares: tuple             # how often each occurs
    wheels_s: float           # when the wheel readings are taken (s after the cycle's start)
    imu_s: float              # when the balance loop takes the IMU reading
    command_s: float          # when the new command reaches the wheels (until then they run the previous one)
    imu_loop: bool            # the IMU read by its own loop (the reading is its latest sample, 0-2.4 ms old)

    @property
    def mean_period_s(self):
        return float(np.dot(self.periods_s, self.shares) / np.sum(self.shares))

    def period(self, rng):
        """One cycle's period (s), drawn from the measured distribution."""
        if len(self.periods_s) == 1:
            return self.periods_s[0]
        p = np.asarray(self.shares, float)
        return float(rng.choice(self.periods_s, p=p / p.sum()))


# What sim/lqr modelled until 2026-10-29 for the firmware up to 10-28 12:44 (telemetry lhz 201 balancing; the IMU
# read in the cycle after the wheels): a fixed 5 ms, wheels at the start, IMU 2.5 ms, command 2.7 ms. The model's
# fits and freq_response replay recordings of that firmware (the 10-19/20/28 frequency sweeps) with it.
FW_2026_10_19 = Timing("firmware to 2026-10-28 12:44", (0.005,), (1.0,), 0.0, 0.0025, 0.0027, False)

# The firmware since the single IMU loop (e387674, 10-28 12:44), the LQR (ctrl 1). MEASURED periods: 123991 cycles
# of balance_watch recordings 10-28 12:45 - 10-29 07:00 (mean 4.132 ms, 242 Hz; 2 stalls of 49 and 79 ms not
# drawn). DERIVED in-cycle times from the Pi's riderlog, medians of the per-window worst cases (18253 windows):
# wheel reads 2.93 ms (2.82-3.12), compute + write 0.72 (0.68-0.78) -> IMU at 2.93, command at 3.65. ASSUMED: each
# wheel's position is sampled mid-way through its read -> the two wheels' mean at 2.93 / 2 = 1.46 ms.
FW_NOW_LQR = Timing(
    "firmware since 2026-10-28 12:44, LQR",
    (0.003, 0.00325, 0.0035, 0.00375, 0.004, 0.00425, 0.0045, 0.00475, 0.005, 0.00525, 0.0055),
    (0.0036, 0.0065, 0.0011, 0.0136, 0.7515, 0.0641, 0.0593, 0.0335, 0.0349, 0.0276, 0.0043),
    0.00146, 0.00293, 0.00365, True)

# The same firmware running the policy (ctrl 0): the network adds ~0.4 ms. MEASURED periods: 29351 cycles (mean
# 4.687 ms, 213 Hz). DERIVED: wheel reads 2.93 ms, compute + write 1.02 (network 0.40; 14887 windows) -> command
# at 3.95. ASSUMED: the wheels' sample as above.
FW_NOW_POLICY = Timing(
    "firmware since 2026-10-28 12:44, policy",
    (0.00325, 0.0035, 0.00375, 0.004, 0.00425, 0.0045, 0.00475, 0.005, 0.00525, 0.0055, 0.00575),
    (0.0022, 0.0461, 0.0097, 0.1279, 0.0542, 0.114, 0.1459, 0.4164, 0.0583, 0.018, 0.0072),
    0.00146, 0.00293, 0.00395, True)
