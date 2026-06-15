#!/usr/bin/env python3
"""Driving test bed: reproduce (or not) the on-robot forward shimmy IN SIM.

The training env only balances in place. The shimmy is a DRIVING phenomenon, so
this harness wraps the sim physics with the firmware's exact drive loop
(esp32_rider_fw/src/main.cpp): trapezoidal motion profile -> position-hold
setpoint-tilt (P + velocity-D) -> tilt the policy's pitch obs -> run the EXPORTED
policy (bit-faithful to the ESP32) -> step the actuator.

Sign note: on hardware forward lean makes measured pitch DECREASE, so the firmware
feeds the policy pitch in the sim frame; here the policy's pitch obs = sim_pitch +
bias_rad (a negative bias => the policy leans the body +pitch => drives +x).

Run:  ./.venv/bin/python drive_sim.py [policy]   (default ppo_v_pure = flashed)
"""
import sys
import numpy as np
from collections import deque
from rider_env import RiderBalanceEnv

# --- firmware drive gains (main.cpp defaults) ---
K, KI, KD = 5.0, 2.5, 28.0          # position-hold P / I / velocity-D (deg, deg*s, deg per m/s)
POSMAX_DEG = 5.0                    # setpoint-bias clamp (deg)
AMAX = 0.8                          # accel cap (m/s^2)
VEL_LP = 0.8                        # LP on velocity feeding the D term
DEG2RAD = 0.017453292
WHEEL_R = 0.03


def load_policy(name):
    d = np.load(name + "_export.npz")
    mean, var, eps, clip = d["mean"], d["var"], float(d["eps"]), float(d["clip"])
    W0, b0, W1, b1, W2, b2 = d["W0"], d["b0"], d["W1"], d["b1"], d["W2"], d["b2"]

    def infer(o):
        z = np.clip((o - mean) / np.sqrt(var + eps), -clip, clip)
        x = np.tanh(W0 @ z + b0); x = np.tanh(W1 @ x + b1)
        return float(np.clip((W2 @ x + b2)[0], -1.0, 1.0))
    return infer


G = 9.81


def simulate(infer, direction, vmax, T=7.0, seed=0, accel_couple=0.0, latency_s=None):
    """accel_couple: gain on the accelerometer-during-acceleration pitch corruption.
    latency_s: override the actuator pure-delay (the MEASURED on-robot value is ~0.04 s,
    vs the sim default 0.003 s -- the unmodeled delay that costs phase margin)."""
    from dataclasses import replace
    from rider_params import DEFAULT
    p = DEFAULT if latency_s is None else replace(DEFAULT, latency_s=latency_s)
    env = RiderBalanceEnv(params=p, add_noise=False, frame_stack=1)   # we build the stacked obs ourselves
    env.reset(seed=seed)
    dt = env.ctrl_dt
    pitch, prate, x, x_vel, wheel_vel = env._raw_state()
    x0 = x
    tgt = x0 + direction * 5.0                              # far target => cruise at vmax the whole run
    tgt_eff = x0; vdes_s = 0.0; xvelF = 0.0; prevX = x0; pint = 0.0
    theta_filt = pitch; prev_xv = x_vel                    # complementary-filter state + accel calc
    stack = deque(maxlen=2)
    log = []
    n = int(T / dt)
    for i in range(n):
        pitch, prate, x, x_vel, wheel_vel = env._raw_state()
        # --- sensing model: optionally corrupt pitch like the real IMU ---
        if accel_couple != 0.0:
            a_x = (x_vel - prev_xv) / dt                    # cart horizontal accel (m/s^2)
            pit_acc = pitch + accel_couple * np.arctan2(a_x, G)   # accelerometer can't separate tilt from accel
            pf = 1.0 / (abs(np.degrees(prate)) + 200.0)     # firmware rate-adaptive accel weight (~0.005)
            theta_filt = pf * pit_acc + (1 - pf) * (theta_filt + prate * dt)  # gyro = drift-free true rate
            pitch_obs = theta_filt
        else:
            pitch_obs = pitch
        prev_xv = x_vel
        xvel = (x - prevX) / dt; prevX = x
        xvelF = VEL_LP * xvelF + (1 - VEL_LP) * xvel
        # trapezoidal profile (brake-distance capped velocity)
        dist = tgt - tgt_eff
        vstop = np.sqrt(2 * AMAX * abs(dist))
        vcap = min(vmax, vstop)
        vcmd = vcap if dist >= 0 else -vcap
        damax = AMAX * dt
        if vcmd > vdes_s + damax: vdes_s += damax
        elif vcmd < vdes_s - damax: vdes_s -= damax
        else: vdes_s = vcmd
        tgt_eff += vdes_s * dt
        vdes = vdes_s
        pos_err = x - tgt_eff
        bias_deg = np.clip(K * pos_err + KD * (xvelF - vdes), -POSMAX_DEG, POSMAX_DEG)
        bias_rad = bias_deg * DEG2RAD
        pr_pitch = pitch_obs + bias_rad                     # setpoint-tilted (possibly accel-corrupted) pitch obs
        pint = float(np.clip(pint + pr_pitch * dt, -1.0, 1.0))
        frame = np.array([pr_pitch, prate, 0.0, x_vel, wheel_vel, pint, 0.0], np.float32)
        if i == 0:
            for _ in range(2): stack.append(frame)
        else:
            stack.append(frame)
        obs = np.concatenate(stack)
        a = infer(obs)
        env.step(np.array([a], np.float32))
        log.append((i * dt, pitch, prate, a, x_vel, bias_deg, x - x0))
    return np.array(log)


def stats(L, settle=2.0):
    m = L[:, 0] > settle                                    # after settling
    pitch, prate, a = L[m, 1], L[m, 2], L[m, 3]
    return dict(pitch_std=np.degrees(np.std(pitch)),
                rate_rms=np.degrees(np.sqrt(np.mean(prate ** 2))),
                act_rms=np.sqrt(np.mean(a ** 2)),
                act_sat=100 * np.mean(np.abs(a) > 0.95),
                travel=L[-1, 6])


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "ppo_v_pure"
    infer = load_policy(name)
    print("=== driving test bed: %s ===" % name)
    for ac in (0.0, 1.0, -1.0):
        lbl = "perfect sensing" if ac == 0.0 else "accel-coupled pitch (gain %+.0f)" % ac
        print("\n-- %s --" % lbl)
        print("%-22s %9s %9s %8s %7s %8s" % ("condition", "pitchStd", "rateRMS", "actRMS", "sat%", "travel"))
        for vmax in (0.22, 0.35):
            for dirn, tag in ((+1, "FWD"), (-1, "REV")):
                L = simulate(infer, dirn, vmax, accel_couple=ac)
                s = stats(L)
                print("%-22s %8.2f° %8.1f° %8.3f %6.0f%% %+7.2fm"
                      % ("%s vmax=%.2f" % (tag, vmax), s["pitch_std"], s["rate_rms"],
                         s["act_rms"], s["act_sat"], s["travel"]))


if __name__ == "__main__":
    main()
