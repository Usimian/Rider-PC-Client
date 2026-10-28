"""Driving: the Hover's plan (its sim/drive_design.py, robot.cc plan_step) on the Rider, against today's drive
handling (2026-10-28: LQR2 fell twice within a second of a full-stick start, the command pinned at +400).

    python3 sim/lqr/drive_design.py                  # speeds 0.30 0.45 0.60, motors x1 and x0.85
    python3 sim/lqr/drive_design.py --speeds 0.3 --a-max 1.0 0.5

Today (main.cpp, both LQR and LQR2) the stick's speed is ramped at 0.4 m/s^2 and the target runs ahead at it,
with 360 units per m/s fed straight to the wheels: the robot is asked for a speed before it has leaned into it
(to speed up it must first roll back), and LQR2's 3.4x stiffer position gain turns that into a yank. The
Hover's plan hands the law a path it can follow: the planned speed closes on the stick's at up to a_max, its
acceleration building over t_jerk; the plan's lean and command are the model's steady values for that speed and
acceleration (feedforward(), the linear model, friction off); the plan never runs more than 15 cm from the
robot; its own integral always runs. The Hover's numbers: a_max 1.0, t_jerk 0.12 s, gain 8 /s.

Per case, on the full model (friction, sensor noise, the firmware cycle): FELL, or start (time to 90 % of the
speed, lean range in the first 1.5 s, the share of it at the +/-400 limit), stop after release (how far it rolls,
lean range), reversal (lean range, share at the limit). The motors x0.85 case stands in for a low battery.
"""
import argparse
from dataclasses import replace

import numpy as np

import linear_model as L
from closed_loop import CYCLE_S, K_TODAY, U_MAX, Firmware, run
from model import Params

LQR2 = dict(x=-1694.0, xi=-30.0, vx=-6.78, q=53.3, dq=1.32)     # the flashed defaults (the Hover way)
A_V = float(np.exp(-CYCLE_S / 0.100))
UP = 2.8


def feedforward(p):
    """The steady lean (rad, + forward) and command for a speed v and an acceleration a, th = th_v v + th_a a,
    u = u_v v + u_a a: the linear model with th'' = 0 (friction off)."""
    A, B = L.continuous(p)
    M = np.array([[A[1, 2], B[1, 0]], [A[3, 2], B[3, 0]]])
    th_v, u_v = np.linalg.solve(M, [-A[1, 1], -A[3, 1]])
    th_a, u_a = np.linalg.solve(M, [1.0, 0.0])
    return dict(th_v=th_v, th_a=th_a, u_v=u_v, u_a=u_a)


def case(p, mk, drive, T):
    rows, fell = run(p, mk(), T=T, th0_deg=0.0, drive=drive, log_all=True)
    return np.array(rows), fell


def describe(log, t_go, v_cmd, window=1.5):
    t, u, v, lean = log[:, 0], log[:, 2], log[:, 3], log[:, 4]
    a = np.exp(-CYCLE_S / 0.05)
    vf = v.copy()
    for i in range(1, len(vf)):
        vf[i] = a * vf[i - 1] + (1 - a) * vf[i]
    w = (t >= t_go) & (t < t_go + window)
    hit = np.sign(v_cmd) * vf[w] > 0.9 * abs(v_cmd)
    t90 = (t[w][np.argmax(hit)] - t_go) if hit.any() else np.inf
    return t90, lean[w].min(), lean[w].max(), 100 * np.mean(np.abs(u[w]) >= U_MAX - 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.30, 0.45, 0.60])
    ap.add_argument("--a-max", type=float, nargs="+", default=[1.0])
    ap.add_argument("--t-jerk", type=float, default=0.12)
    ap.add_argument("--gain", type=float, default=8.0)
    ap.add_argument("--motors", type=float, nargs="+", default=[1.0, 0.85],
                    help="motor drive scale; the 10-28 floor drives at a 34 %% battery needed ~460 units per m/s "
                         "to cruise (the model: 333, 10-19 floor: 360) -> ~0.78")
    a = ap.parse_args()
    p0 = Params()
    ff = feedforward(p0)
    print(f"plan per m/s^2: lean {np.degrees(ff['th_a']):.2f} deg, command {ff['u_a']:.0f}; per m/s: lean "
          f"{np.degrees(ff['th_v']):.2f} deg, command {ff['u_v']:.0f}  (Hover: 14 deg per m/s^2, 333 per m/s)")
    ctrls = [("LQR, today's drive", lambda: Firmware(up_deg=UP)),
             ("LQR2, today's drive", lambda: Firmware(k=LQR2, up_deg=UP, corrected=True, a_v=A_V, fric_comp=True))]
    for am in a.a_max:
        plan = dict(ff, a_max=am, t_jerk=a.t_jerk, gain=a.gain)
        ctrls.append((f"LQR2, Hover plan {am:g} m/s^2",
                      lambda plan=plan: Firmware(k=LQR2, up_deg=UP, a_v=A_V, fric_comp=True, plan=plan)))
    for motor in a.motors:
        p = replace(p0, ku_over_b=p0.ku_over_b * motor)
        print(f"\nmotors x{motor:g}" + ("  (a weaker battery)" if motor < 1 else ""))
        for V in a.speeds:
            print(f"  full stick {V:.2f} m/s:      start: 90% in  lean (deg)     at limit | stop: rolls on  lean (deg)"
                  f"     | reversal: lean (deg)     at limit")
            for name, mk in ctrls:
                log, fell = case(p, mk, (np.array([0.0, 2.0, 5.0]), np.array([0.0, V, 0.0])), 9.0)
                if fell:
                    print(f"    {name:30s} FELL at {log[-1, 0]:.2f} s ({'start' if log[-1, 0] < 5 else 'stop'})")
                    continue
                t90, l0, l1, lim = describe(log, 2.0, V)
                t = log[:, 0]
                st = (t >= 5.0) & (t < 8.0)
                x = np.cumsum(log[:, 3]) * CYCLE_S
                roll = (x[st][-1] - x[st][0]) * 1000
                rl, fr = case(p, mk, (np.array([0.0, 2.0, 5.0, 7.5]), np.array([0.0, V, -V, 0.0])), 9.0)
                if fr:
                    rev = f"FELL at {rl[-1, 0]:.2f} s"
                else:
                    _, r0, r1, rlim = describe(rl, 5.0, -V, window=2.0)
                    rev = f"{r0:+5.1f}..{r1:+5.1f}   {rlim:4.0f}%"
                print(f"    {name:30s} {t90:5.2f} s  {l0:+5.1f}..{l1:+5.1f}  {lim:5.0f}%  | {roll:5.0f} mm  "
                      f"{log[st, 4].min():+5.1f}..{log[st, 4].max():+5.1f}  | {rev}")


if __name__ == "__main__":
    main()
