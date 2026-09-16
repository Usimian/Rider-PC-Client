#!/usr/bin/env python3
"""Driving test bed: reproduce (or not) the on-robot forward shimmy IN SIM.

The training env only balances in place. The shimmy is a DRIVING phenomenon, so
this harness wraps the sim physics with the firmware's exact drive loop
(esp32_rider_fw/src/main.cpp): trapezoidal motion profile -> position-hold
setpoint-tilt (P + velocity-D) -> tilt the policy's pitch obs -> run the EXPORTED
policy (bit-faithful to the ESP32) -> step the actuator.

Sensing: by default the policy sees what the FIRMWARE feeds it (rider_env.FirmwareSensing:
encoder-differenced + low-passed velocity, low-passed gyro rate -- 2026-09-16 delay budget).
`--legacy_obs` gives it clean physics state, the pre-2026-09-16 sim. Run both to see what the
firmware filters do to a policy.

Sign note: on hardware forward lean makes measured pitch DECREASE, so the firmware
feeds the policy pitch in the sim frame; here the policy's pitch obs = sim_pitch +
bias_rad (a negative bias => the policy leans the body +pitch => drives +x).

Run:  ./.venv/bin/python drive_sim.py [policy ...] [--legacy_obs] [--both] [--accel]
      (default policy ppo_mc_l4a = flashed; --both runs fw and legacy sensing side by side)
"""
import sys
import argparse
import numpy as np
from collections import deque
from dataclasses import replace
from rider_env import RiderBalanceEnv
from rider_params import DEFAULT

# --- firmware drive gains (main.cpp POLICY defaults, reconciled 2026-08-13) ---
K, KI, KD = 20.0, 1.0, 10.0         # gPosHoldK (poshold) / gPosHoldKi (poshi) / gDriveKd (drivekd, MOVING regime)
POSMAX_DEG = 5.0                    # gPosHoldMax: setpoint-bias clamp (deg)
AMAX = 0.8                          # gPosAmax: accel cap (m/s^2)
VEL_LP = DEFAULT.fw_drive_vel_lp    # gPosVelLP: LP on velocity feeding the D term
INTBAND = 0.15                      # gPosIntBand: integrate only within this of the FINAL target
VMAX = 0.25                         # gPosVmax = gPosVmaxBack (full stick, both directions, main.cpp)
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


def simulate(infer, direction, vmax, T=7.0, seed=0, accel_couple=0.0, latency_s=None, fw_obs=True, params=None,
             stick=True):
    """accel_couple: gain on the accelerometer-during-acceleration pitch corruption.
    latency_s: override the actuator pure-delay (default = rider_params, the measured on-robot value).
    fw_obs: firmware sensing path (default) vs clean physics state (legacy).
    params: full RiderParams override (sweeps); latency_s still applies on top of it.
    stick: model the joystick drive (D-only lean, firmware default) vs a 'ptgt' distance move (P-term lean)."""
    p = params or DEFAULT
    if latency_s is not None:
        p = replace(p, latency_s=latency_s)
    env = RiderBalanceEnv(params=p, add_noise=False, frame_stack=1, fw_obs=fw_obs)   # stacked obs built here
    env.reset(seed=seed)
    dt = env.ctrl_dt
    pitch, prate, x, x_vel, wheel_vel = env._raw_state()
    x0 = x
    tgt = x0 + direction * 5.0                              # far target => cruise at vmax the whole run
    tgt_eff = x0; vdes_s = 0.0; xvelF = 0.0; prevX = x0; pint = 0.0; pint_pos = 0.0
    theta_filt = pitch; prev_xv = x_vel                    # complementary-filter state + accel calc
    stack = deque(maxlen=2)
    log = []
    n = int(T / dt)
    fell_t = None
    for i in range(n):
        pitch, prate_true, x, x_vel_true, wheel_vel_true = env._raw_state()
        # sensed values: env._sense() ran at the end of the previous step (firmware order)
        _, prate, x_vel, wheel_vel = env.last_sensed
        # --- sensing model: optionally corrupt pitch like the real IMU ---
        if accel_couple != 0.0:
            a_x = (x_vel_true - prev_xv) / dt               # cart horizontal accel (m/s^2)
            pit_acc = pitch + accel_couple * np.arctan2(a_x, G)   # accelerometer can't separate tilt from accel
            pf = 1.0 / (abs(np.degrees(prate_true)) + 200.0)      # firmware rate-adaptive accel weight (~0.005)
            theta_filt = pf * pit_acc + (1 - pf) * (theta_filt + prate_true * dt)
            pitch_obs = theta_filt
        else:
            pitch_obs = pitch
        prev_xv = x_vel_true
        # drive loop's velocity: the firmware differences the ENCODER wheel_x (same read as the policy's)
        xvel = env.sense.xvel_raw if fw_obs else (x - prevX) / dt
        prevX = x
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
        if stick and vmax > 0.0:
            # STICK DRIVE (main.cpp): while the stick is held the target tracks wheel_x every cycle
            # (posErr == 0) and the hold integral is zeroed -> lean = KD*(xvelF - vdes) ONLY (<= 2.5 deg
            # at full stick). The far-target P-term cruise below (stick=False) is a 'ptgt' distance
            # move, which leans up to 5 deg -- it hid a policy's command bias (ppo_fw_s2, 2026-09-16).
            tgt_eff = x; pint_pos = 0.0
        pos_err = x - tgt_eff
        if tgt_eff == tgt and abs(pos_err) < INTBAND:
            kiLim = POSMAX_DEG / KI if KI > 0.01 else 1e6
            pint_pos = float(np.clip(pint_pos + pos_err * dt, -kiLim, kiLim))
        bias_deg = np.clip(K * pos_err + KI * pint_pos + KD * (xvelF - vdes), -POSMAX_DEG, POSMAX_DEG)
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
        _, _, term, _, _ = env.step(np.array([a], np.float32))
        log.append((i * dt, pitch, prate_true, a, x_vel_true, bias_deg, x - x0))
        if term:
            fell_t = (i + 1) * dt
            break
    return np.array(log), fell_t, dt


def dom_freq(x, fs, lo=3.0, hi=20.0):
    """Dominant frequency of x in [lo, hi] Hz and its peak/mean prominence in that band."""
    if len(x) < 32:
        return float("nan"), float("nan")
    x = x - x.mean()
    X = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), 1 / fs)
    m = (fr >= lo) & (fr <= hi)
    band = X[m]
    k = int(np.argmax(band))
    return float(fr[m][k]), float(band[k] / (band.mean() + 1e-9))


def stats(L, fs, settle=2.0):
    m = L[:, 0] > settle                                    # after settling
    if m.sum() < 32:                                        # fell before/around settle -> use what we have
        m = np.ones(len(L), bool)
    pitch, prate, a = L[m, 1], L[m, 2], L[m, 3]
    f, prom = dom_freq(prate, fs)
    return dict(pitch_std=np.degrees(np.std(pitch)),
                rate_rms=np.degrees(np.sqrt(np.mean(prate ** 2))),
                act_rms=np.sqrt(np.mean(a ** 2)),
                act_sat=100 * np.mean(np.abs(a) > 0.95),
                travel=L[-1, 6], freq=f, prom=prom)


def run_table(infer, fw_obs, accel_couples):
    for ac in accel_couples:
        lbl = "perfect pitch" if ac == 0.0 else "accel-coupled pitch (gain %+.0f)" % ac
        print("  -- %s sensing, %s --" % ("FIRMWARE" if fw_obs else "LEGACY (clean)", lbl))
        print("  %-14s %9s %9s %8s %7s %8s %9s %6s" %
              ("condition", "pitchStd", "rateRMS", "actRMS", "sat%", "travel", "domFreq", "fell@"))
        for dirn, tag in ((+1, "FWD"), (-1, "REV")):
            L, fell_t, dt = simulate(infer, dirn, VMAX, accel_couple=ac, fw_obs=fw_obs)
            s = stats(L, 1.0 / dt)
            print("  %-14s %8.2f° %8.1f° %8.3f %6.0f%% %+7.2fm %5.1fHz x%-3.0f %6s"
                  % ("%s vmax=%.2f" % (tag, VMAX), s["pitch_std"], s["rate_rms"], s["act_rms"],
                     s["act_sat"], s["travel"], s["freq"], s["prom"],
                     "%.1fs" % fell_t if fell_t else "up"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policies", nargs="*", default=["ppo_mc_l4a"])
    ap.add_argument("--legacy_obs", action="store_true", help="clean physics obs (pre-2026-09-16 sim)")
    ap.add_argument("--both", action="store_true", help="run firmware AND legacy sensing")
    ap.add_argument("--accel", action="store_true", help="also run the accel-coupled pitch variants")
    args = ap.parse_args()
    modes = [True, False] if args.both else [not args.legacy_obs]
    acs = (0.0, 1.0, -1.0) if args.accel else (0.0,)
    for name in args.policies:
        infer = load_policy(name)
        print("=== driving test bed: %s  (full stick %.2f m/s, %d s) ===" % (name, VMAX, 7))
        for fw in modes:
            run_table(infer, fw, acs)
        print()


if __name__ == "__main__":
    main()
