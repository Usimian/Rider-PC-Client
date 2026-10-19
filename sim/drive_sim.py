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
import argparse
import numpy as np
from collections import deque
from dataclasses import replace
from rider_env import RiderBalanceEnv, SpinActuator
from rider_env_twowheel import RiderTwoWheelEnv
from rider_params import DEFAULT

# --- firmware drive gains (main.cpp POLICY defaults, reconciled 2026-08-13) ---
K, KI, KD = 20.0, 1.0, 10.0         # gPosHoldK (poshold) / gPosHoldKi (poshi) / gDriveKd (drivekd, MOVING regime)
POSMAX_DEG = 8.0                    # gPosHoldMax: setpoint-bias clamp (deg). 5 -> 8 with the LQR-like drive (2026-10-19):
                                    # at 5 it pinned in low-legs reverse (3 deg offset + reverse lean) -> 0.25 m/s
AMAX = 0.4                          # gPosAmax: accel cap (m/s^2) -- 0.8 -> 0.4 fw default 2026-10-19
VEL_LP = DEFAULT.fw_drive_vel_lp    # gPosVelLP: LP on velocity feeding the D term
INTBAND = 0.15                      # gPosIntBand: integrate only within this of the FINAL target
DEADBAND = 0.012                    # gPosDeadband: no P term / no integration this close to the hold target
VMAX = 0.30                         # gPosVmax = gPosVmaxBack (full stick, both directions) -- 0.25 -> 0.30 2026-10-19
HOLD_S = 1.5                        # stick runs start from a HOLD this long (the hold integral builds, as on the robot)
DEG2RAD = 0.017453292
WHEEL_R = 0.03
LEASH = 0.25                        # 'lead' drive: the moving target may run at most this far from the robot (m)
POL_FULL_CMD = 30.0 * 7.764         # raw wheel-cmd at policy action 1 (main.cpp POL_VELMAX_RADS x POL_RAW_PER_RADS)
# --- firmware LQR (main.cpp defaults, floor-tuned 2026-06-19 + drive rework 2026-10-18) ---
LQR_X, LQR_XI, LQR_VX, LQR_Q, LQR_DQ = -500.0, -30.0, -6.0, 32.0, 3.0   # gLqrX / gLqrXi / gLqrVx / gLqrQ / gLqrDq
LQR_IXMAX = 3.0                     # gLqrIxMax (m*s)
LQR_VREF = 270.0                    # gLqrVRef: speed units per m/s of vdes
LQR_CMDFF = 360.0                   # gLqrCmdFF: wheel-cmd per m/s of vdes
LQR_FF, LQR_FFBAND = 10.0, 20.0     # gFF / gFFband: stiction boost
LQR_UMAX = 400.0                    # gUMax: output clamp (raw wheel-cmd)
LQR_VEL_LP = 0.85                   # LP_VEL: the servo-speed low-pass behind wheel_vx
LQR_VEL_PER_MPS = 265.0             # MEASURED 2026-10-19: wheel_vx units per m/s of ground speed (floor cruise)
LQR_SPEED_STEP = 0.0031             # MEASURED 2026-10-19: one raw unit of the servos' reported speed = 3.1 mm/s
                                    # (floor sweeps: 3.09-3.17 mm/s per unit vs the encoder speed; no delay)


def load_policy(name):
    d = np.load(name + "_export.npz")
    mean, var, eps, clip = d["mean"], d["var"], float(d["eps"]), float(d["clip"])
    W0, b0, W1, b1, W2, b2 = d["W0"], d["b0"], d["W1"], d["b1"], d["W2"], d["b2"]

    def infer(o):
        z = np.clip((o - mean) / np.sqrt(var + eps), -clip, clip)
        x = np.tanh(W0 @ z + b0); x = np.tanh(W1 @ x + b1)
        return float(np.clip((W2 @ x + b2)[0], -1.0, 1.0))
    return infer


def spin_encoders(env):
    """The firmware's wheel_x while spinning: the MEAN of the two wheels' own 10-bit encoders, each turning
    (forward +/- spin). Each quantizes on its own, so the mean steps in half-counts at irregular times -- velocity
    noise a still or straight-driving robot never has. Replaces the single-encoder model in env.sense."""
    s = env.sense
    q, r = s.q_step, s.p.wheel_radius_m
    s._enc_x = lambda wa: 0.5 * (np.round((wa + env.act.spin) / q) + np.round((wa - env.act.spin) / q)) * q * r


def lqr_controller(params):
    """The firmware's LQR (main.cpp, floor-tuned gains + 2026-10-18 drive rework), in the sim frame.
    Firmware: u = -(X*x + Xi*Ix + Vx*vx + Q*q + Dq*dq) + CmdFF*vdes, with q = theta - set (robot frame, forward lean
    makes it DECREASE) = -pitch_sim, dq = -rate_sim, x = wheel_x - target (m), vx = wheel speed (scaled units,
    ~265 per m/s) - VRef*vdes; then the stiction boost and the gUMax clamp. Returns u in raw wheel-cmd units."""
    def u_of(x_err, ix, vx_err, pitch_deg, rate_deg, vdes):
        u = -(LQR_X * x_err + LQR_XI * ix + LQR_VX * vx_err - LQR_Q * pitch_deg - LQR_DQ * rate_deg) + LQR_CMDFF * vdes
        if u > LQR_FFBAND: u += LQR_FF
        elif u < -LQR_FFBAND: u -= LQR_FF
        return float(np.clip(u, -LQR_UMAX, LQR_UMAX))
    return u_of


def simulate(infer, direction, vmax, T=7.0, seed=0, accel_couple=None, latency_s=None, fw_obs=True, params=None,
             stick=True, hold_s=HOLD_S, pitch_offset_deg=0.0, drift_deg=0.0, controller="policy", drive="latch",
             leash=LEASH, drive_int=False, reverse_at=None, turn_cmd=0.0, spin_gain=1.0, spin_enc=True,
             spin_lat=0.0, spin_tau=0.0, lqr_speed="reported", inject=None, rate_offset_dps=0.0,
             twowheel=False, yaw_cmd=0.0, yaw_from_s=None, right_wheel=None, noise=False):
    """accel_couple: override the accelerometer-reads-acceleration gain (default rider_params, MEASURED);
    the firmware's pitch fusion lives in rider_env.FirmwareSensing, so it applies with fw_obs only.
    latency_s: override the actuator pure-delay (default = rider_params, the measured on-robot value).
    fw_obs: firmware sensing path (default) vs clean physics state (legacy).
    params: full RiderParams override (sweeps); latency_s still applies on top of it.
    stick: model the joystick drive vs a 'ptgt' distance move (P-term lean).
    hold_s: stick runs HOLD this long first (hold integral builds), then full stick.
    pitch_offset_deg: constant error in the controller's pitch (sim frame, deg): the balance point moved, e.g. legs
      lowered (floor 2026-10-19: ~-3 at the lowest height -> forward creep, no reverse).
    drift_deg: a pitch-sensor drift (sim frame, deg) that builds over 1.5 s starting 1 s into the stick drive, as in
      a fast turn (floor 2026-10-19: ~-2 -> the robot sped up and fell forward).
    controller: 'policy' (infer) or 'lqr' (the firmware LQR; infer unused).
    drive: the policy's stick-drive structure -- 'latch' (firmware now: target = wheel_x every cycle, only the
      velocity-D acts, hold integral frozen) or 'lead' (LQR-like: the target advances at the commanded speed and
      the position P keeps acting; it is dragged along if the robot falls more than `leash` m behind/ahead).
      drive_int: with 'lead', also integrate the position error while driving (as the LQR does).
    reverse_at: flip the stick to the opposite direction at this time (s) -- full forward straight into full
      reverse (floor 2026-10-19: where ppo_fw_s9 fell).
    turn_cmd: spin in place with this turn output (raw wheel-cmd, floor spins ~60) -- SpinActuator.
      spin_gain: the spinning wheels' strength vs straight driving; spin_lat / spin_tau: their extra pure
      delay / extra first-order lag (s);
      spin_enc: model the two encoders' quantization.
    lqr_speed: the LQR's speed signal -- 'reported' (the servos' own speed, as the firmware uses) or 'encoder'
      (the per-cycle encoder difference; the sim LQR before 2026-10-19).
    inject: t -> raw wheel-cmd added to the controller's output (the firmware's frequency sweep, 'chirp').
    twowheel: the two-wheel plant (rider_env_twowheel) with the firmware turn controller; yaw_cmd = the
      'turnrate' (rad/s, full stick 1.0) held from yaw_from_s (default hold_s) -- a real spin."""
    p = params or DEFAULT
    if latency_s is not None:
        p = replace(p, latency_s=latency_s)
    if accel_couple is not None:
        p = replace(p, accel_couple=accel_couple)
    if controller == "lqr":
        p = replace(p, cmd_limit=LQR_UMAX / POL_FULL_CMD)      # the LQR may use the whole gUMax range
        lqr = lqr_controller(p)
    env_cls = RiderTwoWheelEnv if twowheel else RiderBalanceEnv
    np.random.seed(seed)                                    # the sensor noise (noise=True) uses numpy's global RNG
    env = env_cls(params=p, add_noise=noise, frame_stack=1, fw_obs=fw_obs)   # stacked obs built here
    yaw_from = hold_s if yaw_from_s is None else yaw_from_s
    if twowheel and right_wheel:
        env.right_kw = dict(right_wheel)                    # the right wheel differs (see RiderTwoWheelEnv)
    env.reset(seed=seed)
    if turn_cmd:
        env.act = SpinActuator(p, env.ctrl_dt, turn_cmd / POL_FULL_CMD, spin_gain, spin_lat, spin_tau)
        if spin_enc:
            spin_encoders(env)
    dt = env.ctrl_dt
    pitch, prate, x, x_vel, wheel_vel = env._raw_state()
    x0 = x
    tgt = x0 + direction * 5.0                              # far target => cruise at vmax the whole run
    tgt_eff = x0; vdes_s = 0.0; xvelF = 0.0; prevX = x0; pint = 0.0; pint_pos = 0.0
    vest = 0.0; lqr_ix = 0.0
    stack = deque(maxlen=2)
    log = []
    n = int(T / dt)
    fell_t = None
    for i in range(n):
        t = i * dt
        pitch, prate_true, x, x_vel_true, wheel_vel_true = env._raw_state()
        # sensed values: env._sense() ran at the end of the previous step (firmware order); with fw_obs the
        # pitch is the firmware's fused one (accelerometer reads the chassis acceleration too)
        pitch_obs, prate, x_vel, wheel_vel = env.last_sensed
        dis = pitch_offset_deg + drift_deg * float(np.clip((t - hold_s - 1.0) / 1.5, 0.0, 1.0))
        pitch_obs += dis * DEG2RAD
        prate = prate + rate_offset_dps * DEG2RAD           # a gyro offset (e.g. yaw leaking in while spinning)
        # drive loop's velocity: the firmware differences the ENCODER wheel_x (same read as the policy's)
        xvel = env.sense.xvel_raw if fw_obs else (x - prevX) / dt
        prevX = x
        xvelF = VEL_LP * xvelF + (1 - VEL_LP) * xvel
        damax = AMAX * dt
        kiLim = POSMAX_DEG / KI if KI > 0.01 else 1e6
        driving = stick and vmax > 0.0 and t >= hold_s
        sdir = -direction if (reverse_at is not None and t >= reverse_at) else direction
        if controller == "lqr":
            # LQR drive management (main.cpp): stick held -> target ADVANCES at the ramped speed, the position
            # integral runs throughout; held still -> target latched at the start.
            vcmd = sdir * vmax if driving else 0.0
            if vcmd > vdes_s + damax: vdes_s += damax
            elif vcmd < vdes_s - damax: vdes_s -= damax
            else: vdes_s = vcmd
            tgt_eff += vdes_s * dt
            vdes = vdes_s
            lqr_ix = float(np.clip(lqr_ix + (x - tgt_eff) * dt, -LQR_IXMAX, LQR_IXMAX))
            # the LQR's speed is the SERVOS' reported speed (wheel relative to body): MEASURED 2026-10-19 (floor
            # sweeps) on time with the encoder speed (< 1 ms) but in fine LQR_SPEED_STEP steps each cycle -- not
            # the per-cycle encoder difference (~43 mm/s steps), which made the sim LQR ring at 10-15 Hz.
            if lqr_speed == "reported":
                vrep = np.round(wheel_vel_true * WHEEL_R / LQR_SPEED_STEP) * LQR_SPEED_STEP
            else:
                vrep = xvel
            vest = LQR_VEL_LP * vest + (1 - LQR_VEL_LP) * vrep * LQR_VEL_PER_MPS   # firmware LP_VEL, scaled units
            u = lqr(x - tgt_eff, lqr_ix, vest - LQR_VREF * vdes, np.degrees(pitch_obs), np.degrees(prate), vdes)
            a = u / POL_FULL_CMD
            bias_deg = 0.0
        else:
            if stick:
                # STICK (main.cpp): HOLD until hold_s (target latched at x0; the hold integral builds, with
                # the deadband), then full stick: vdes ramps at AMAX.
                vcmd = sdir * vmax if driving else 0.0
                if vcmd > vdes_s + damax: vdes_s += damax
                elif vcmd < vdes_s - damax: vdes_s -= damax
                else: vdes_s = vcmd
                vdes = vdes_s
                if driving and drive == "latch":
                    # target tracks wheel_x every cycle (posErr 0), integral FROZEN: lean = KD*(xvelF - vdes) + cancel
                    tgt_eff = x
                    pos_err = 0.0
                elif driving:
                    # 'lead': the target advances at the commanded speed; the robot is held to it (LQR-like)
                    tgt_eff += vdes_s * dt
                    tgt_eff = float(np.clip(tgt_eff, x - leash, x + leash))   # dragged, never further than the leash
                    pos_err = x - tgt_eff
                    if drive_int:
                        pint_pos = float(np.clip(pint_pos + pos_err * dt, -kiLim, kiLim))
                else:
                    pos_err = x - tgt_eff
                    inband = abs(pos_err) < DEADBAND
                    if abs(pos_err) < INTBAND and not inband:
                        pint_pos = float(np.clip(pint_pos + pos_err * dt, -kiLim, kiLim))
                    if inband:
                        pos_err = 0.0
            else:
                # 'ptgt' distance move: trapezoidal profile (brake-distance capped) to a far target, P-term lean
                dist = tgt - tgt_eff
                vstop = np.sqrt(2 * AMAX * abs(dist))
                vcap = min(vmax, vstop)
                vcmd = vcap if dist >= 0 else -vcap
                if vcmd > vdes_s + damax: vdes_s += damax
                elif vcmd < vdes_s - damax: vdes_s -= damax
                else: vdes_s = vcmd
                tgt_eff += vdes_s * dt
                vdes = vdes_s
                pos_err = x - tgt_eff
                if tgt_eff == tgt and abs(pos_err) < INTBAND:
                    pint_pos = float(np.clip(pint_pos + pos_err * dt, -kiLim, kiLim))
            bias_deg = np.clip(K * pos_err + KI * pint_pos + KD * (xvelF - vdes), -POSMAX_DEG, POSMAX_DEG)
            bias_rad = bias_deg * DEG2RAD
            pr_pitch = pitch_obs + bias_rad                 # setpoint-tilted (possibly accel-corrupted) pitch obs
            pint = float(np.clip(pint + pr_pitch * dt, -1.0, 1.0))
            frame = np.array([pr_pitch, prate, 0.0, x_vel, wheel_vel, pint, 0.0], np.float32)
            if i == 0:
                for _ in range(2): stack.append(frame)
            else:
                stack.append(frame)
            obs = np.concatenate(stack)
            a = infer(obs)
        inj = inject(t) if inject else 0.0                  # firmware 'chirp': known signal on the wheel command
        a = a + inj / POL_FULL_CMD
        if twowheel:
            env.yaw_cmd = yaw_cmd if t >= yaw_from else 0.0         # the joystick's 'turnrate' (rad/s)
        _, _, term, _, _ = env.step(np.array([a], np.float32))
        log.append((t, pitch, prate_true, a, x_vel_true, bias_deg, x - x0, vdes_s, xvel, inj,   # xvel: encoder speed
                    env.last_turn if twowheel else 0.0, env.yaw_rate_turn() if twowheel else 0.0))
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
        lbl = "measured accel coupling" if ac is None else "accel coupling %+.2f" % ac
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
    ap.add_argument("--accel", action="store_true", help="also run with NO accelerometer coupling (contrast)")
    args = ap.parse_args()
    modes = [True, False] if args.both else [not args.legacy_obs]
    acs = (None, 0.0) if args.accel else (None,)
    for name in args.policies:
        infer = load_policy(name)
        print("=== driving test bed: %s  (full stick %.2f m/s, %d s) ===" % (name, VMAX, 7))
        for fw in modes:
            run_table(infer, fw, acs)
        print()


if __name__ == "__main__":
    main()
