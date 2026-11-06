"""The MPC drive planner exported for the firmware (2026-11-03): its tables, a Python mirror of the firmware's
arithmetic, and the checks.

    python3 sim/lqr/mpc_export.py            # check the mirror, run the hard cases on it, write the header
    python3 sim/lqr/mpc_export.py --no-cases # check and write only

Writes firmware/esp32_rider_fw/src/mpc_plan_data.h (GENERATED: rerun after changing mpc_plan.py's QP or SHAPE).

The firmware planner (main.cpp, MPC PLANNER) is mpc_plan.MpcFirmware at the ESP32 size (SHAPE: 25 steps of 10
cycles, ~1.03 s; the limits checked once a step; 50 ADMM iterations; a step late), with:
  * the motor's strength from the battery, vbat / VREF (tools/balance/motor_strength.py), the nearest of BINS -- a
    table set per bin (told 5 % wrong the planner's cases did not change);
  * the lean limit aimed at LEAN_AIM_DEG (Marc's 12 less ~1: checked once a step, the peak lands between);
  * the pitch estimate corrected (ACC_COMP, flashed): its error e stays ~0 and is taken as 0;
  * ADMM's multiplier reset each solve, x warm-started from the last plan shifted a step.
Per bin and mode (held: track the stick's speed; released: back to the held home) the solve is
    xi = Phi xi0 + Gam j_applied            (the state a step on: the plan arrives a step late)
    l, u = -1 - Y0 xi, 1 - Y0 xi            (the limit rows, each scaled to its limit)
    q = Qxi xi + Qt target                  (target: the stick's speed, or the home's position)
    x = ADMM(Minv, G, q, l, u)  ->  the plan's jerks J_SCALE x (m/s^3)
xi = (axle x, its speed, lean, rate, the last command, the law's smoothed speed, the smoothed rate, e = 0, the
plan's x, v, a, smoothed v), the model's frame (+ forward), m, m/s, rad, rad/s, command units.
FwPlanner is that arithmetic in Python; check() holds it to mpc_plan's planner; cases() runs the hard cases with
FwMirror (the simulated robot planning exactly so).
"""
import argparse
import os
from dataclasses import replace

import numpy as np

import closed_loop as CL
import drive_design as DD
import mpc_plan as M
from model import Params

SHAPE = M.Shape(horizon=25, step_cycles=10, check_every=10, iters=50, late=True)
BINS = (1.0, 0.9, 0.8, 0.75)          # motor strengths with a table set each
VREF = 8.29                           # V: motor = vbat / VREF (MEASURED 11-03, tools/balance/motor_strength.py)
LEAN_AIM_DEG = 11.0
VMAX = 0.45                           # m/s, the firmware's gPosVmax / gPosVmaxBack
SAMPLE = (0.78, 0.45, 8)              # the bench's sample: motors, full-stick speed, planner step into the start
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "firmware", "esp32_rider_fw", "src",
                   "mpc_plan_data.h")


def planner(motor):
    # walls=False: the firmware has no wall limit yet (its tables are the 75 limit rows; the wall rows are the HANDOFF's
    # next step: export them, M 75 -> 100, with the firmware's 'wall <mm>')
    return M.MpcPlanner(M.Predictor(motor=motor, acc_comp="plan", shape=SHAPE), lean_max_deg=LEAN_AIM_DEG, v_max=VMAX,
                        walls=False)


def tables(motor):
    """The firmware's tables for one bin, from mpc_plan's planner."""
    mp = planner(motor)
    pr, lim, ws = mp.p, mp.lim, mp.ws
    Y0 = np.vstack([pr.U0 / lim["u"], pr.Th0 / lim["th"], pr.S0["v"] / lim["v"]])
    Sv, Sx, Sa = pr.Sj["v"], pr.Sj["x"], pr.Sj["a"]
    ones = np.ones(SHAPE.horizon)
    t = dict(G=mp.G, Y0=Y0, Phi=pr.Phi, Gam=pr.Gam)
    t["Qxi"] = [2 * ws * (M.W_V * Sv.T @ pr.S0["v"] + M.W_A * Sa.T @ pr.S0["a"]),
                2 * ws * (M.W_X * Sx.T @ pr.S0["x"] + M.W_V_REL * Sv.T @ pr.S0["v"] + M.W_A * Sa.T @ pr.S0["a"])]
    t["Qt"] = [-2 * ws * M.W_V * Sv.T @ ones, -2 * ws * M.W_X * Sx.T @ ones]
    qp0 = mp.qps["held"]
    t["Minv"] = [np.linalg.inv(mp.qps[m].P + qp0.sigma * np.eye(SHAPE.horizon) + qp0.rho * mp.G.T @ mp.G)
                 for m in ("held", "released")]
    t["qp"] = dict(rho=qp0.rho, sigma=qp0.sigma, alpha=qp0.alpha, soft=qp0.soft)
    t["mp"] = mp
    return t


def admm(Minv, A, q, l, u, x, rho, sigma, alpha, soft, iters):
    """main.cpp mpcAdmm (mpc_admm.h), line for line: z = clip(Ax), y = 0, then the iterations."""
    x = x.copy()
    z, y = np.clip(A @ x, l, u), np.zeros(A.shape[0])
    t = soft / rho
    for _ in range(iters):
        xt = Minv @ (sigma * x - q + A.T @ (rho * z - y))
        x = alpha * xt + (1 - alpha) * x
        zr = alpha * (A @ xt) + (1 - alpha) * z
        v = zr + y / rho
        zn = np.where(v > u + t, v - t, np.where(v < l - t, v + t, np.clip(v, l, u)))
        y = y + rho * (zr - zn)
        z = zn
    return x


class FwPlanner:
    """The firmware planner's arithmetic (main.cpp mpcSolve)."""

    def __init__(self):
        self.tabs = [tables(m) for m in BINS]
        self.x = np.zeros(SHAPE.horizon)

    @staticmethod
    def bin_for(vbat):
        m = vbat / VREF
        return int(np.argmin([abs(m - b) for b in BINS]))

    def solve(self, xi0, held, target, j_applied, vbat):
        t = self.tabs[self.bin_for(vbat)]
        mode = 0 if held else 1
        xi = t["Phi"] @ xi0 + t["Gam"] * j_applied
        free = t["Y0"] @ xi
        q = t["Qxi"][mode] @ xi + t["Qt"][mode] * target
        warm = np.r_[self.x[1:], self.x[-1]]
        self.x = admm(t["Minv"][mode], t["G"], q, -1.0 - free, 1.0 - free, warm, iters=SHAPE.iters, **t["qp"])
        return self.x * M.J_SCALE


class FwMirror(M.MpcFirmware):
    """The simulated robot planning with the firmware's arithmetic (FwPlanner), its battery read as motor x VREF."""

    def __init__(self, motor, **kw):
        super().__init__(motor=motor, shape=SHAPE, lean_max_deg=LEAN_AIM_DEG, **kw)
        self.fp, self.vbat = FwPlanner(), motor * VREF

    def _plan_acc(self, dt, stick, held, x, pitch):
        if self.cycle % SHAPE.step_cycles == 0:
            xi0 = self.xi(x, pitch)
            self.jerk = self.plan_j[1]
            target = float(np.clip(stick, -VMAX, VMAX)) if held else self.home
            self.plan_j = np.r_[0.0, self.fp.solve(xi0, held, target, self.jerk, self.vbat)]
        self.cycle += 1
        self.a_r += self.jerk * dt
        self.track_bias()


def check(n=8, seed=3):
    """The mirror's q, limits and answer against mpc_plan's planner (the same QP, ADMM from the same start)."""
    rng = np.random.default_rng(seed)
    fp = FwPlanner()
    worst = 0.0
    for i in range(n):
        b = i % len(BINS)
        t = fp.tabs[b]
        mp, pr = t["mp"], t["mp"].p
        xi0 = np.zeros(pr.N)
        xi0[1] = xi0[pr.iv] = xi0[pr.ivf] = xi0[5] = rng.uniform(-0.4, 0.4)
        xi0[2] = np.radians(rng.uniform(-3, 3))
        j_ap, held = rng.uniform(-3, 3), i % 2 == 0
        target = rng.uniform(-0.45, 0.45) if held else rng.uniform(-0.1, 0.1)
        xi = pr.Phi @ xi0 + pr.Gam * j_ap
        mp.J = np.zeros(SHAPE.horizon)
        mp.qps = {m: M.Qp(q.P, q.A, rho=q.rho, sigma=q.sigma, alpha=q.alpha, iters=q.iters, soft=q.soft)
                  for m, q in mp.qps.items()}
        J_ref = mp.solve(xi, v_target=target) if held else mp.solve(xi, home=target)
        fp.x = np.zeros(SHAPE.horizon)
        J_fw = fp.solve(xi0, held, target, j_ap, BINS[b] * VREF)
        worst = max(worst, float(np.max(np.abs(J_fw - J_ref))))
    print(f"the firmware's arithmetic (FwPlanner) against mpc_plan's planner, {n} states over the {len(BINS)} bins: "
          f"largest jerk difference {worst:.2e} m/s^3")


def cases():
    CL.VMAX = VMAX
    p0 = Params()
    print(f"the hard cases with the firmware's planner (FwMirror: bins {BINS}, lean aimed at {LEAN_AIM_DEG:g}, "
          f"top speed {VMAX}):")
    for motor in (1.0, 0.85, 0.78):
        p = replace(p0, ku_over_b=p0.ku_over_b * motor)
        for V in (0.30, 0.45):
            for name, mk in (("plan 0.6 (flashed)", lambda: CL.flashed(up_deg=DD.UP)),
                             ("MPC (firmware's)", lambda: FwMirror(motor, up_deg=DD.UP))):
                log, fell = DD.case(p, mk, (np.array([0.0, 2.0, 5.0]), np.array([0.0, V, 0.0])), 8.0)
                if fell:
                    print(f"  x{motor:<5g}{V:.2f} m/s  {name:20s} FELL at {log[-1, 0]:.2f} s")
                    continue
                t90, l0, l1, lim = DD.describe(log, 2.0, V)
                t = log[:, 0]
                st = (t >= 5.0) & (t < 8.0)
                xx = np.cumsum(log[:, 3] * np.gradient(t))
                rl, fr = DD.case(p, mk, (np.array([0.0, 2.0, 5.0, 7.5]), np.array([0.0, V, -V, 0.0])), 9.0)
                rev = "FELL" if fr else "{:+5.1f}..{:+5.1f} {:3.0f}%".format(*DD.describe(rl, 5.0, -V, window=2.0)[1:])
                print(f"  x{motor:<5g}{V:.2f} m/s  {name:20s} start {t90:4.2f} s {l0:+5.1f}..{l1:+5.1f} {lim:3.0f}% | "
                      f"stop {(xx[st][-1] - xx[st][0]) * 1000:+4.0f} mm {log[st, 4].min():+5.1f}..{log[st, 4].max():+5.1f}"
                      f" | reversal {rev}", flush=True)


def sample():
    """The bench's sample: a real planner state from a simulated start (the firmware's planner, SAMPLE), with the
    mirror's answer from a cold start."""
    motor, V, step = SAMPLE
    CL.VMAX = VMAX
    p = replace(Params(), ku_over_b=Params().ku_over_b * motor)
    caught = []

    class Cap(FwMirror):
        def _plan_acc(self, dt, stick, held, x, pitch):
            if self.cycle % SHAPE.step_cycles == 0 and held:
                caught.append((self.xi(x, pitch), self.plan_j[1], float(np.clip(stick, -VMAX, VMAX))))
            super()._plan_acc(dt, stick, held, x, pitch)

    CL.run(p, Cap(motor, up_deg=DD.UP), T=3.0, th0_deg=0.0, drive=(np.array([0.0, 2.0]), np.array([0.0, V])),
           log_all=True)
    xi0, j_ap, target = caught[step]
    fp = FwPlanner()
    J = fp.solve(xi0, True, target, j_ap, motor * VREF)
    return dict(xi0=xi0, j_ap=j_ap, target=target, vbat=motor * VREF, J=J, bin=fp.bin_for(motor * VREF))


def fl(v):
    """A C++ float literal (always with a '.' or an exponent)."""
    t = f"{float(v):.9g}"
    return (t if ("." in t or "e" in t or "n" in t) else t + ".0") + "f"


def carr(name, dims, rows):
    """A const float array; rows nested as dims says (the last dimension flat)."""
    def body(a, d):
        if len(d) == 1:
            return "{" + ", ".join(fl(v) for v in np.asarray(a).ravel()) + "}"
        return "{" + ",\n  ".join(body(r, d[1:]) for r in a) + "}"
    return f"static const float {name}{''.join(f'[{d}]' for d in dims)} = {body(rows, dims)};\n"


def write(smp):
    tabs = [tables(m) for m in BINS]
    H, N = SHAPE.horizon, tabs[0]["mp"].p.N
    Mrows = tabs[0]["G"].shape[0]
    qp = tabs[0]["qp"]
    with open(OUT, "w") as f:
        f.write("// GENERATED by sim/lqr/mpc_export.py -- do not edit: rerun it after changing sim/lqr/mpc_plan.py's QP or\n"
                f"// its SHAPE ({SHAPE.name}). The MPC drive planner's tables, per battery bin and mode (0 held, 1\n"
                f"// released): see mpc_export.py. Lean aimed at {LEAN_AIM_DEG:g} deg, top speed {VMAX} m/s.\n"
                "// xi: axle x, v, lean, rate, last command, smoothed v, smoothed rate, e (0), plan x, v, a, smoothed v\n"
                "// (the model's frame, + forward; m, m/s, rad, rad/s, command units).\n#pragma once\n")
        f.write(f"#define MPC_N {H}\n#define MPC_M {Mrows}\n#define MPC_X {N}\n#define MPC_BINS {len(BINS)}\n"
                f"#define MPC_ITERS {SHAPE.iters}\n#define MPC_STEP_CYCLES {SHAPE.step_cycles}\n")
        f.write(f"static const float MPC_RHO = {fl(qp['rho'])}, MPC_SIGMA = {fl(qp['sigma'])}, MPC_ALPHA = "
                f"{fl(qp['alpha'])}, MPC_SOFT = {fl(qp['soft'])}, MPC_J_SCALE = {fl(M.J_SCALE)}, MPC_VREF = {fl(VREF)}, "
                f"MPC_VMAX = {fl(VMAX)};\n")
        f.write(carr("MPC_BIN_MOTOR", [len(BINS)], BINS))
        f.write(carr("MPC_G", [len(BINS), Mrows * H], [t["G"] for t in tabs]))
        f.write(carr("MPC_Y0", [len(BINS), Mrows * N], [t["Y0"] for t in tabs]))
        f.write(carr("MPC_PHI", [len(BINS), N * N], [t["Phi"] for t in tabs]))
        f.write(carr("MPC_GAM", [len(BINS), N], [t["Gam"] for t in tabs]))
        f.write(carr("MPC_MINV", [len(BINS), 2, H * H], [t["Minv"] for t in tabs]))
        f.write(carr("MPC_QXI", [len(BINS), 2, H * N], [t["Qxi"] for t in tabs]))
        f.write(carr("MPC_QT", [len(BINS), 2, H], [t["Qt"] for t in tabs]))
        f.write(f"// 'mpcbench' sample: {SAMPLE[2]} planner steps into a full-stick start to {SAMPLE[1]} m/s at motors "
                f"x{SAMPLE[0]} (bin {smp['bin']}), held; MPCB_J the mirror's plan from a cold start (m/s^3)\n")
        f.write(carr("MPCB_XI0", [N], smp["xi0"]))
        f.write(f"static const float MPCB_JAP = {fl(smp['j_ap'])}, MPCB_TARGET = {fl(smp['target'])}, MPCB_VBAT = "
                f"{fl(smp['vbat'])};\n")
        f.write(carr("MPCB_J", [H], smp["J"]))
    floats = len(BINS) * (Mrows * H + Mrows * N + N * N + N + 2 * H * H + 2 * H * N + 2 * H)
    print(f"wrote {os.path.relpath(OUT)}: {floats} floats ({floats * 4 / 1024:.0f} kB in flash), sample bin "
          f"{smp['bin']}, first jerk {smp['J'][0]:+.3f} m/s^3")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-cases", action="store_true")
    a = ap.parse_args()
    check()
    if not a.no_cases:
        cases()
    write(sample())


if __name__ == "__main__":
    main()
