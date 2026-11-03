"""An MPC drive planner on top of the flashed LQR (2026-11-03): the plan's acceleration chosen by a small quadratic
program, looking ahead, within the motor's authority and a body-lean limit.

    python3 sim/lqr/mpc_plan.py --check          # the QP solver against scipy on sample problems
    python3 sim/lqr/mpc_plan.py                  # driving cases: the MPC against today's plan (drive_design.py's)
    python3 sim/lqr/mpc_plan.py --predcheck      # the predictor against the full simulation, a start
    python3 sim/lqr/mpc_plan.py --shapes         # the QP's size (an ESP32's) against the hard cases

Why (drive_design.py, 10-29 / 11-03): no fixed plan cap is right for every battery. At 0.6 m/s^2 (flashed) the LQR
never reaches its +/-400 limit, but starts take 0.45-0.9 s; at 1.0-1.5 m/s^2 a full battery starts 1.5-2x faster,
while motors at x0.78 (a ~34 % battery) fall -- the 1.0 m/s^2 fall at 0.60 m/s is the floor's of 10-28 evening.
The falls come from the LQR's own transient (its correction as the body leans in), not the steady push, so the
planner predicts with the whole closed loop.

The LQR, its readings and filters stay exactly as flashed (closed_loop.flashed); only the plan's acceleration a_r
(closed_loop.Firmware._plan_acc) is replaced. Every shape.step_cycles control cycles the planner solves for the jerk
of a_r over the next shape.horizon steps (each held over a step), predicting with linear_model's cycle model (the plant, the
firmware's readings and filters) closed by the flashed law and the plan's references (position, speed,
acceleration, the speed as the law sees it, smoothed 0.1 s) and the pitch estimate's error (Predictor), 12 states.
Stick held: the planned speed toward the
stick's; released: the plan back to where it was let go (the held home, as main.cpp). Within, at every cycle
ahead: the command |u| <= 400 - U_MARGIN (the friction compensation rides on top), the body's lean |th| <=
LEAN_MAX_DEG (Marc 11-03: 12 to start), the planned speed <= the top speed. The plan's lead limit and its integral
stay as main.cpp has them (the prediction leaves both out).

RESULTS 11-03 (the cases, full model; with the pitch estimate corrected by the plan's acceleration, ACC_COMP): every
case held at both batteries, the motor never pinned (<= 1 % in a reversal), the lean within ~0.5-0.8 deg of its 12;
starts to 90 % 0.53-0.63 s (plan 0.6 with the same correction 0.48-0.98, plan 1.0 0.30-0.58 but 15-16 deg of lean and
16 % pinned at 0.60 m/s on the weak battery); stops roll on 1-5 mm (plan 0.6: up to 60). Weaker: at 0.30 m/s slower
than the flashed plan (0.53 vs 0.46 s, the jerk weight), a 5-6 deg lean-back after a fast stop. Without the
correction the MPC fell (the drifting estimate leaned the body past the plan, the robot outran it).
The predictor MEASURED against the full simulation (--predcheck): 0.2 deg rms of lean, ~10 units of command once the
friction compensation (~32, the margin) is taken out.
The battery (tools/balance/motor_strength.py): motor = vbat / 8.29 V on the robot (2.7 % rms); told 5 % wrong either
way the planner's cases do not change; predicting with the full-strength model and only scaling the command limit
falls (x0.78, 0.60 m/s): the strength has to be in the model -- on a microcontroller, a QP per ~10 % of strength.
The size (--shapes, 11-03): every trimmed shape held every case; 25 steps x 41 ms (1.03 s), the limits checked once a
step, 30-50 ADMM iterations, solved a step late (a planner task): ~6-11 M multiply-adds a second, starts ~0.04 s
slower, the lean up to ~13.4 deg against the 12 (its peak falls between the checked points: aim ~1 deg lower).

Not modelled by the predictor: friction, cogging, the 4.1 ms actuator delay, the plan's lead limit -- the full
simulation (closed_loop.run) has them all, so the cases test the planner against them. The motor strength (the
battery) is GIVEN to the planner here (motor=); on the robot it would have to be estimated (battery voltage, or the
command per m/s while cruising).

The QP: min 1/2 J'HJ + f'J, l <= GJ <= h, solved by ADMM (OSQP's iteration, fixed step): H and G do not change with
the state, so the one matrix to factor is factored once per mode (held / released); each solve is then matrix-vector
work, warm-started from the last plan shifted by a step -- the shape a microcontroller can run.
"""
import argparse
from dataclasses import dataclass, replace

import numpy as np
from scipy.linalg import cho_factor, cho_solve

import closed_loop as CL
import linear_model as L
from closed_loop import FLASHED_K, FLASHED_PLAN, FLASHED_UP_DEG, L2_SPEED, U_MAX, Firmware
from model import Params



@dataclass(frozen=True)
class Shape:
    """The QP's size: steps looked ahead, control cycles per step, cycles between the checked points of the command
    and the lean, ADMM iterations; late: the plan used one step after the state it was solved from (a planner task
    slower than the control cycle, as on the ESP32) -- solved from the state predicted a step on instead."""
    horizon: int = 50
    step_cycles: int = 5
    check_every: int = 1
    iters: int = 200
    late: bool = False

    @property
    def name(self):
        T = L.T * self.step_cycles
        return (f"{self.horizon} x {T * 1000:.0f} ms ({self.horizon * T:.2f} s), checks every "
                f"{self.check_every} cycle{'s' if self.check_every > 1 else ''}, {self.iters} iters"
                + (", a step late" if self.late else ""))


FULL = Shape()                                            # the prototype: ~48 Hz, 1.03 s, every cycle checked
LEAN_MAX_DEG = 12.0        # the body's lean limit, deg (Marc, 11-03)
U_MARGIN = 40.0            # command units kept free for the friction compensation (32.5) and slack
J_SCALE = 10.0             # m/s^3 per unit of the QP's variable (keeps it near 1)
E_WEIGHT = 1.0 / 210.0     # the IMU path's accelerometer weight a sample, 1/(|rate| + 200) at ~10 deg/s
SOFT = 1e4                 # the price of a limit overstepped (per limit's width): steep, but never impossible
ACC_COMP = CL.FLASHED_ACC_COMP   # the pitch estimate corrected by the plan's acceleration (as flashed since 11-03):
                           # MEASURED 11-03, uncorrected the estimate drifts ~3 deg through a start, the body outruns
                           # the plan and the weak-battery 0.60 m/s start falls; "wheels" holds too but leans 13-14
# cost weights, per planner step: speed error (m/s)^2, position error (m)^2 when released, jerk (m/s^3)^2,
# acceleration (m/s^2)^2
# W_J MEASURED 11-03 (the cases): 2e-4..2e-2 re-plans bang-bang (the jerk flips sign each step, the body swings back
# 9-13 deg in a start) and the weak-battery 0.60 m/s start falls; 2e-1 holds every case, ~0.1 s slower
W_V, W_X, W_V_REL, W_J, W_A = 100.0, 400.0, 25.0, 2e-1, 0.5


class Predictor:
    """The closed loop at the control cycle: xi = (linear_model's z (7): the axle x, v, the lean th, w, u_prev, vf_prev,
    dqf_prev; e, the firmware's pitch estimate less the true lean; the plan's x_r, v_r, a_r, v_rf), the input the jerk
    j (m/s^3) of a_r. Units: m, m/s, rad, rad/s, the model's command (+ forward = the firmware's).

    e: the LQR's pitch (the Hover's IMU path, closed_loop.hover_sample) blends the accelerometer's angle into the gyro's
    at 1/(|rate| + 200) a sample, and the accelerometer reads the axle's and the body's acceleration as tilt -- the
    lean minus (x'' + IMU_H th'')/g, the model's frame. So e decays by that weight and is pushed by the accelerations;
    the law reads th + e. MEASURED 11-03 (predcheck): left out, the predictor's start peaked 0.9 deg and ~35 units
    under the full simulation's; with friction, the delay and that left out it matched to 0.16 deg and 4 units."""

    def __init__(self, p=None, motor=1.0, plan=FLASHED_PLAN, tau_v=0.100, acc_comp=None, shape=FULL):
        """acc_comp: as closed_loop.Firmware's -- the accelerometer corrected by the plan's acceleration ("plan") or the
        measured one ("wheels", taken as the axle's own): e is then pushed only by what the correction misses."""
        p = p or Params()
        T = L.T
        a_v = float(np.exp(-T / tau_v))
        Az7, Bz7, C7 = L.plant(p, a_v=a_v, corrected=True, motor=motor, v_corr_by_dqf=True, lqr2=True)
        # the estimate's error e, an 8th state: per cycle the accelerometer's weight over ~1.75 IMU samples, the
        # accelerations at the IMU's moment (the previous command still running)
        A_c, B_c = L.continuous(p)
        B_c = B_c * motor
        PhI, GI = L.flow(A_c, B_c, L.TI)
        s_ti = np.hstack([PhI, GI, np.zeros((4, 2))])              # the plant's state at TI, on z (7)
        acc = A_c[[1, 3]] @ s_ti
        acc[:, 4] += B_c[[1, 3], 0]                                  # (x'', th'') at TI
        self.w_c = 1.0 - (1.0 - E_WEIGHT) ** (T / CL.HOVER_IMU_PERIOD_S)
        Az, Bz, C = np.zeros((8, 8)), np.zeros((8, 1)), np.zeros((4, 8))
        Az[:7, :7], Bz[:7], C[:, :7] = Az7, Bz7, C7
        Az[7, :7] = -self.w_c * ((0.0 if acc_comp == "wheels" else acc[0]) + CL.IMU_H * acc[1]) / CL.G
        Az[7, 7] = 1.0 - self.w_c
        C[2, 7] = 1.0                                                # the law's lean = th + e
        K = L.from_fw(FLASHED_K)                         # u = K y; y = (axle x, vf, th, dqf)
        n = Az.shape[0]
        self.n, self.N, self.T, self.ie = n, n + 4, T, 7
        ix, iv, ia, ivf = n, n + 1, n + 2, n + 3
        # the plan's step (closed_loop._control_plan: a_r, then v_r, then x_r, then v_rf), j held over the cycle
        Rr = np.zeros((4, 4))
        Rj = np.zeros(4)
        Rr[2, 2], Rj[2] = 1.0, T                         # a1 = a + T j
        Rr[1, 1], Rr[1, 2], Rj[1] = 1.0, T, T * T        # v1 = v + T a1
        Rr[0, 0], Rr[0, 1], Rr[0, 2], Rj[0] = 1.0, T, T * T, T ** 3   # x1 = x + T v1
        Rr[3] = (1 - a_v) * Rr[1]
        Rr[3, 3] += a_v
        Rj[3] = (1 - a_v) * Rj[1]                        # vf1 = a_v vf + (1 - a_v) v1
        # the law this cycle: u = K (y - y_ref) + u_ff, y_ref = (x_r before the step, v_rf, the plan's lean), u_ff
        # and the lean from the stepped a_r, v_r (as _control_plan)
        th_v, th_a, u_v, u_a = plan["th_v"], plan["th_a"], plan["u_v"], plan["u_a"]
        c_new = np.zeros(4)                              # on the stepped references
        c_new[3] = -K[1]
        c_new[1] = -K[2] * th_v + u_v
        c_new[2] = -K[2] * th_a + u_a
        self.Ku = np.zeros(self.N)                       # u = Ku xi + Kuj j
        self.Ku[:n] = K @ C
        self.Ku[ix] = -K[0]
        self.Ku[n:] += c_new @ Rr
        self.Kuj = float(c_new @ Rj)
        self.A = np.zeros((self.N, self.N))
        self.B = np.zeros(self.N)
        self.A[:n, :n] = Az
        self.A[:n] += np.outer(Bz[:, 0], self.Ku)
        self.B[:n] = Bz[:, 0] * self.Kuj
        self.A[n:, n:] = Rr
        self.B[n:] = Rj
        if acc_comp == "plan":                           # the accelerometer less the plan's (stepped) acceleration
            self.A[7, n:] += self.w_c / CL.G * Rr[2]
            self.B[7] += self.w_c / CL.G * Rj[2]
        self.ix, self.iv, self.ia, self.ivf, self.ith = ix, iv, ia, ivf, 2
        self.shape = shape
        self._lift()

    def _lift(self):
        """Every output over the horizon as Y0 xi0 + Yj J (J = the horizon's jerks / J_SCALE): the command and the lean
        at every shape.check_every-th cycle, per step the plan's position, speed and acceleration; and one step's
        flow (Phi, Gam: xi a step on, the step's jerk held)."""
        H, S, ce, N = self.shape.horizon, self.shape.step_cycles, self.shape.check_every, self.N
        Mx, Mj = np.eye(N), np.zeros((N, H))
        U0, Uj, Th0, Thj = [], [], [], []
        S0 = {k: np.zeros((H, N)) for k in ("x", "v", "a")}
        Sj = {k: np.zeros((H, H)) for k in ("x", "v", "a")}
        for c in range(H * S):
            s = c // S
            chk = c % ce == ce - 1
            if chk:
                U0.append(self.Ku @ Mx)
                Uj.append(self.Ku @ Mj)
                Uj[-1][s] += self.Kuj
            Mx, Mj = self.A @ Mx, self.A @ Mj
            Mj[:, s] += self.B
            if chk:
                Th0.append(Mx[self.ith])
                Thj.append(Mj[self.ith])
            if c % S == S - 1:
                for k, i in (("x", self.ix), ("v", self.iv), ("a", self.ia)):
                    S0[k][s], Sj[k][s] = Mx[i], Mj[i]
            if c == S - 1:
                self.Phi, self.Gam = Mx.copy(), Mj[:, 0].copy()
        sc = J_SCALE
        self.U0, self.Uj, self.Th0, self.Thj = np.array(U0), np.array(Uj) * sc, np.array(Th0), np.array(Thj) * sc
        self.S0, self.Sj = S0, {k: v * sc for k, v in Sj.items()}


class Qp:
    """min 1/2 x'Px + q'x, l <= Ax <= u by ADMM (OSQP's iteration with a fixed rho): P and A fixed, factored once.
    soft: the limits softened -- soft x the distance outside them is added to the cost (an exact penalty: with the
    limits reachable the answer is the hard one; from a state already past one, the least-bad plan instead of none).
    In ADMM it only changes the projection step: past a limit by more than soft/rho, step back by soft/rho."""

    def __init__(self, P, A, rho=10.0, sigma=1e-6, alpha=1.6, iters=200, soft=None):
        self.P, self.A, self.rho, self.sigma, self.alpha, self.iters = P, A, rho, sigma, alpha, iters
        self.soft = soft
        self.F = cho_factor(P + sigma * np.eye(P.shape[0]) + rho * A.T @ A)
        self.x, self.z, self.y = np.zeros(P.shape[0]), np.zeros(A.shape[0]), np.zeros(A.shape[0])

    def solve(self, q, l, u, warm=None):
        A, rho, sg, al = self.A, self.rho, self.sigma, self.alpha
        x = self.x if warm is None else warm
        z, y = np.clip(A @ x, l, u), self.y
        for _ in range(self.iters):
            xt = cho_solve(self.F, sg * x - q + A.T @ (rho * z - y))
            zt = A @ xt
            x = al * xt + (1 - al) * x
            zr = al * zt + (1 - al) * z
            v = zr + y / rho
            if self.soft is None:
                zn = np.clip(v, l, u)
            else:
                t = self.soft / rho
                zn = np.where(v > u + t, v - t, np.where(v < l - t, v + t, np.clip(v, l, u)))
            y = y + rho * (zr - zn)
            z = zn
        self.x, self.z, self.y = x, z, y
        self.prim = float(np.max(np.abs(A @ x - z)))
        self.dual = float(np.max(np.abs(self.P @ x + q + A.T @ y)))
        return x


class MpcPlanner:
    """The QP per mode (stick held: track a speed; released: return to a position), built on a Predictor."""

    def __init__(self, pred, lean_max_deg=LEAN_MAX_DEG, u_max=U_MAX - U_MARGIN, v_max=None):
        self.p = pred
        self.lim = dict(u=u_max, th=np.radians(lean_max_deg), v=CL.VMAX if v_max is None else v_max)
        G = np.vstack([pred.Uj / self.lim["u"], pred.Thj / self.lim["th"], pred.Sj["v"] / self.lim["v"]])
        self.G = G
        Sv, Sx, Sa = pred.Sj["v"], pred.Sj["x"], pred.Sj["a"]
        H, it = pred.shape.horizon, pred.shape.iters
        # the weights are per step: a step of another length weighs in proportion (FULL's 5 cycles = 1)
        ws = pred.shape.step_cycles / FULL.step_cycles
        reg = ws * (W_J * J_SCALE ** 2 * np.eye(H) + W_A * Sa.T @ Sa)
        self.ws = ws
        self.qps = {"held": Qp(2 * ws * W_V * Sv.T @ Sv + 2 * reg, G, soft=SOFT, iters=it),
                    "released": Qp(2 * ws * (W_X * Sx.T @ Sx + W_V_REL * Sv.T @ Sv) + 2 * reg, G, soft=SOFT, iters=it)}
        self.J = np.zeros(H)

    def bounds(self, xi0):
        pr, lim = self.p, self.lim
        free = np.concatenate([pr.U0 @ xi0 / lim["u"], pr.Th0 @ xi0 / lim["th"], pr.S0["v"] @ xi0 / lim["v"]])
        return -1.0 - free, 1.0 - free

    def solve(self, xi0, v_target=None, home=None):
        """The jerk plan (m/s^3, a horizon's steps) from the state xi0: toward v_target (held) or back to home (released)."""
        pr = self.p
        lo, hi = self.bounds(xi0)
        Sv, Sx, Sa = pr.Sj["v"], pr.Sj["x"], pr.Sj["a"]
        fa = W_A * Sa.T @ (pr.S0["a"] @ xi0)
        if home is None:
            mode, q = "held", 2 * self.ws * (W_V * Sv.T @ (pr.S0["v"] @ xi0 - v_target) + fa)
        else:
            mode = "released"
            q = 2 * self.ws * (W_X * Sx.T @ (pr.S0["x"] @ xi0 - home) + W_V_REL * Sv.T @ (pr.S0["v"] @ xi0) + fa)
        warm = np.r_[self.J[1:], self.J[-1]]
        self.J = self.qps[mode].solve(q, lo, hi, warm=warm)
        self.qp, self.q, self.lo, self.hi = self.qps[mode], q, lo, hi
        return self.J * J_SCALE


class MpcFirmware(Firmware):
    """The flashed firmware (closed_loop.flashed) with the plan's acceleration from the MPC planner."""

    def __init__(self, motor=1.0, lean_max_deg=LEAN_MAX_DEG, p=None, up_deg=FLASHED_UP_DEG, acc_comp=ACC_COMP,
                 pred_motor=None, shape=FULL, **kw):
        """motor: the strength the planner believes (on the robot: tools/balance/motor_strength.py, vbat / 8.29 V).
        pred_motor: predict with a motor of this strength instead and scale the command limit by motor / pred_motor
        (pred_motor 1: one QP for every battery, as a microcontroller would keep it)."""
        super().__init__(k=FLASHED_K, up_deg=up_deg, fric_comp=True, plan=dict(FLASHED_PLAN), hover_imu=True,
                         tau_v=0.100, acc_comp=acc_comp, **kw)
        pm = motor if pred_motor is None else pred_motor
        self.pred = Predictor(p, motor=pm, acc_comp=acc_comp, shape=shape)
        self.shape, self.plan_j = shape, np.zeros(shape.horizon)
        self.mpc = MpcPlanner(self.pred, lean_max_deg=lean_max_deg, u_max=(U_MAX - U_MARGIN) * motor / pm)
        self.cycle, self.jerk, self.u_last, self.e_est = 0, 0.0, 0.0, 0.0

    def _output(self, u, dt):
        self.u_last = super()._output(u, dt)
        return self.u_last

    def track_bias(self):
        """The pitch estimate's error e, run alongside each cycle as the predictor has it, its acceleration taken as
        the plan's (the firmware cannot see its own e; the axle follows the plan closely enough)."""
        a_seen = 0.0 if self.acc_comp else self.a_r      # the acceleration the accelerometer still reads as tilt
        self.e_est = (1.0 - self.pred.w_c) * self.e_est - self.pred.w_c * a_seen / CL.G

    def xi(self, x, pitch):
        """The planner's state from what the firmware knows (its frame -> the model's: forward lean is + here). The
        axle's speed: the speed register corrected by the rate, unsmoothed (the law's vel_s lags ~0.1 s)."""
        pr = self.pred
        xi = np.zeros(pr.N)
        v_raw = (self.raw_v - CL.R_WHEEL * np.radians(self.dq2) * CL.SPEED_COUNTS * CL.SCALED_PER_COUNT) / L2_SPEED
        th_est, w = -np.radians(pitch), -np.radians(self.dq_law)
        xi[:8] = (x, v_raw, th_est - self.e_est, w, self.u_last, self.vel_s / L2_SPEED, w, self.e_est)
        xi[pr.ix], xi[pr.iv], xi[pr.ia], xi[pr.ivf] = self.plan_x, self.v_r, self.a_r, self.v_r_f
        return xi

    def _plan_acc(self, dt, stick, held, x, pitch):
        if self.cycle % self.shape.step_cycles == 0:
            xi0 = self.xi(x, pitch)
            if self.shape.late:                          # this step runs the last plan's next jerk; solve from a step on
                self.jerk = self.plan_j[1]
                xi0 = self.pred.Phi @ xi0 + self.pred.Gam * self.jerk
            if held:
                J = self.mpc.solve(xi0, v_target=float(np.clip(stick, -CL.VMAX, CL.VMAX)))
            else:
                J = self.mpc.solve(xi0, home=self.home)
            if not self.shape.late:
                self.jerk = J[0]
            self.plan_j = np.r_[0.0, J]                  # late: the step after this one starts the new plan
        self.cycle += 1
        self.a_r += self.jerk * dt
        self.track_bias()


def check(n=6, seed=1):
    """The ADMM solver against scipy's SLSQP on the planner's own problems, from sample states."""
    from scipy.optimize import minimize
    rng = np.random.default_rng(seed)
    pred = Predictor()
    mpc = MpcPlanner(pred)
    print("ADMM vs SLSQP on the planner's QP (cost, worst constraint, largest jerk difference m/s^3):")
    for i in range(n):
        xi0 = np.zeros(pred.N)
        xi0[1] = xi0[pred.iv] = xi0[pred.ivf] = xi0[5] = rng.uniform(-0.4, 0.4)    # cruising at some speed
        xi0[2] = np.radians(rng.uniform(-3, 3))
        held = i % 2 == 0
        vt = rng.uniform(-0.45, 0.45)
        J = mpc.solve(xi0, v_target=vt) if held else mpc.solve(xi0, home=rng.uniform(-0.1, 0.1))
        qp, q, lo, hi = mpc.qp, mpc.q, mpc.lo, mpc.hi
        Jn = J / J_SCALE
        cost = lambda x: 0.5 * x @ qp.P @ x + q @ x
        cons = [dict(type="ineq", fun=lambda x: hi - qp.A @ x, jac=lambda x: -qp.A),
                dict(type="ineq", fun=lambda x: qp.A @ x - lo, jac=lambda x: qp.A)]
        ref = minimize(cost, np.zeros(mpc.J.size), jac=lambda x: qp.P @ x + q, constraints=cons, method="SLSQP",
                       options=dict(maxiter=500, ftol=1e-10))
        viol = lambda x: float(max(0.0, np.max(qp.A @ x - hi), np.max(lo - qp.A @ x)))
        print(f"  {'held ' if held else 'home '}  ADMM {cost(Jn):10.4f} (viol {viol(Jn):.4f})   SLSQP {ref.fun:10.4f} "
              f"(viol {viol(ref.x):.4f}, {ref.message[:20]})   |dJ| {np.max(np.abs(Jn - ref.x)) * J_SCALE:6.2f}   "
              f"ADMM residuals {qp.prim:.1e} / {qp.dual:.1e}")


class _Recorder(MpcFirmware):
    """The flashed plan (its own acceleration rule), recording per cycle the planner's state and the jerk applied."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.rec = []

    def _plan_acc(self, dt, stick, held, x, pitch):
        a0, xi0 = self.a_r, self.xi(x, pitch)
        Firmware._plan_acc(self, dt, stick, held, x, pitch)
        self.track_bias()
        self.rec.append((xi0, (self.a_r - a0) / dt, self.u_last))


def predcheck(V=0.30, T0=2.0, T=1.5):
    """The predictor against the full simulation: the flashed plan's start at V, its per-cycle jerks fed to the
    predictor from the state the firmware knew at the stick's step; the command and the lean compared, cycle by cycle."""
    fw = _Recorder()
    log, fell = CL.run(Params(), fw, T=T0 + T, th0_deg=0.0, drive=(np.array([0.0, T0]), np.array([0.0, V])),
                       log_all=True)
    pred = fw.pred
    t = log[:, 0]
    k0 = int(np.searchsorted(t, T0))
    xi = fw.rec[k0][0].copy()
    print(f"the flashed plan's start to {V} m/s on the full model, against the predictor (from the state at the step):")
    print("   t (s)   jerk m/s^3   command: sim  predicted   lean deg: sim  predicted   plan speed: sim  predicted")
    for k in range(k0, min(len(fw.rec), len(log)) - 1):
        xi_k, j, _ = fw.rec[k]
        j *= (t[k + 1] - t[k]) / pred.T                  # the firmware's step over this cycle, as one of the mean length
        u_pred = pred.Ku @ xi + pred.Kuj * j
        u_sim = fw.rec[k + 1][2]                         # the command this cycle sent (read at the next one)
        if (k - k0) % 12 == 0:
            print(f"  {t[k] - T0:5.2f}   {j:8.1f}      {u_sim:7.1f}  {u_pred:7.1f}         {log[k, 4]:+6.2f}  "
                  f"{np.degrees(xi[2]):+6.2f}         {xi_k[pred.iv]:5.3f}  {xi[pred.iv]:5.3f}")
        xi = pred.A @ xi + pred.B * j


def cases(motors, speeds):
    import drive_design as DD
    ctrls = [("LQR, plan 0.6, uncorrected (to 11-03)", lambda m: CL.flashed(up_deg=DD.UP, acc_comp=None)),
             ("LQR, plan 0.6 (flashed)", lambda m: CL.flashed(up_deg=DD.UP)),
             ("LQR, plan 1.0", lambda m: CL.flashed(up_deg=DD.UP, a_max=1.0)),
             (f"LQR, MPC {LEAN_MAX_DEG:g} deg, pitch corr.", lambda m: MpcFirmware(motor=m, up_deg=DD.UP))]
    p0 = Params()
    for motor in motors:
        p = replace(p0, ku_over_b=p0.ku_over_b * motor)
        print(f"\nmotors x{motor:g}" + ("  (a weaker battery; the MPC is told)" if motor < 1 else ""))
        for V in speeds:
            print(f"  full stick {V:.2f} m/s:          start: 90% in  lean (deg)     at limit | stop: rolls on  "
                  f"lean (deg)     | reversal: lean (deg)     at limit")
            for name, mk in ctrls:
                log, fell = DD.case(p, lambda: mk(motor), (np.array([0.0, 2.0, 5.0]), np.array([0.0, V, 0.0])), 9.0)
                if fell:
                    print(f"    {name:34s} FELL at {log[-1, 0]:.2f} s ({'start' if log[-1, 0] < 5 else 'stop'})")
                    continue
                t90, l0, l1, lim = DD.describe(log, 2.0, V)
                t = log[:, 0]
                st = (t >= 5.0) & (t < 8.0)
                x = np.cumsum(log[:, 3] * np.gradient(t))
                roll = (x[st][-1] - x[st][0]) * 1000
                rl, fr = DD.case(p, lambda: mk(motor),
                                 (np.array([0.0, 2.0, 5.0, 7.5]), np.array([0.0, V, -V, 0.0])), 9.0)
                if fr:
                    rev = f"FELL at {rl[-1, 0]:.2f} s"
                else:
                    _, r0, r1, rlim = DD.describe(rl, 5.0, -V, window=2.0)
                    rev = f"{r0:+5.1f}..{r1:+5.1f}   {rlim:4.0f}%"
                print(f"    {name:34s} {t90:5.2f} s  {l0:+5.1f}..{l1:+5.1f}  {lim:5.0f}%  | {roll:5.0f} mm  "
                      f"{log[st, 4].min():+5.1f}..{log[st, 4].max():+5.1f}  | {rev}")


SHAPES = (FULL,
          Shape(horizon=50, step_cycles=5, check_every=5),
          Shape(horizon=25, step_cycles=10, check_every=10, iters=50),
          Shape(horizon=25, step_cycles=10, check_every=10, iters=50, late=True),
          Shape(horizon=25, step_cycles=5, check_every=5, iters=50, late=True),
          Shape(horizon=25, step_cycles=10, check_every=10, iters=30, late=True))


def shapes_compare(shapes=SHAPES):
    """The QP's size against the hard cases (full / near-empty battery, 0.45 / 0.60 m/s; starts and their stops), and
    one solve's arithmetic (multiply-adds, roughly what a microcontroller does)."""
    import time
    import drive_design as DD
    p0 = Params()
    cases = [(1.0, 0.30), (1.0, 0.45), (1.0, 0.60), (0.78, 0.45), (0.78, 0.60)]
    print("each: time to 90 % | lean range | % at the limit | lean range after the release")
    for sh in shapes:
        H, rows = sh.horizon, 3 * (sh.horizon * sh.step_cycles // sh.check_every)
        flops = sh.iters * (2 * H * H + 4 * rows * H)
        out = []
        t_solve = []
        for motor, V in cases:
            p = replace(p0, ku_over_b=p0.ku_over_b * motor)
            fws = []
            mk = lambda: fws.append(MpcFirmware(motor=motor, up_deg=DD.UP, shape=sh)) or fws[-1]
            t0 = time.perf_counter()
            log, fell = DD.case(p, mk, (np.array([0.0, 2.0, 5.0]), np.array([0.0, V, 0.0])), 7.0)
            t_solve.append((time.perf_counter() - t0) / max(1, fws[-1].cycle / sh.step_cycles))
            if fell:
                out.append(f"x{motor:g} {V}: FELL {log[-1, 0]:.2f} s")
                continue
            t90, l0, l1, lim = DD.describe(log, 2.0, V)
            st = (log[:, 0] >= 5.0) & (log[:, 0] < 7.0)
            out.append(f"x{motor:g} {V}: {t90:4.2f} s {l0:+5.1f}..{l1:+5.1f} {lim:2.0f}% | {log[st, 4].min():+5.1f}.."
                       f"{log[st, 4].max():+5.1f}")
        print(f"\n{sh.name}: {H} variables, {rows} limit rows, ~{flops / 1e3:.0f}k multiply-adds a solve "
              f"({flops / (L.T * sh.step_cycles) / 1e6:.1f} M/s)")
        for o in out:
            print("   " + o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="the QP solver against scipy")
    ap.add_argument("--predcheck", action="store_true", help="the predictor against the full simulation")
    ap.add_argument("--shapes", action="store_true", help="the QP's size (SHAPES) against the hard cases")
    ap.add_argument("--motors", type=float, nargs="+", default=[1.0, 0.78])
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.30, 0.45, 0.60])
    ap.add_argument("--vmax", type=float, default=0.6, help="the firmware's top speed (closed_loop.VMAX)")
    a = ap.parse_args()
    CL.VMAX = a.vmax
    if a.check:
        check()
        return
    if a.predcheck:
        predcheck()
        return
    if a.shapes:
        shapes_compare()
        return
    cases(a.motors, a.speeds)


if __name__ == "__main__":
    main()
