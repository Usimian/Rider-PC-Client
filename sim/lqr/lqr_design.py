"""An LQR balance controller on the readings the Rider's firmware has (the Hover's sim/lqr_design.py, e2fa373,
on the Rider's linear model).

    python3 sim/lqr/lqr_design.py                        # the Hover's Tightest weights
    python3 sim/lqr/lqr_design.py --x 0.05 --v 0.1 --th 2 --w 20 --u 100 --tau-v 0.031 0.015 0.008

The plant is linear_model.plant (friction and cogging out), with the encoders' body part taken off as on
the Hover: x = x_enc + r th, v_f = smoothed(v_enc + r w), th, dq_f -- and u_fw = -(kx x + kv v_scaled +
kq q + kdq dq), the firmware's form, so only the gains and the correction change on the robot. The gains
minimise the LQR cost

    J = sum over cycles of  (x/X)^2 + (v/V)^2 + (th/TH)^2 + (w/W)^2 + (u/U)^2

(Bryson's rule) from a spread of starting states, computed exactly with a discrete Lyapunov equation, over
the four gains directly: LQR restricted to the readings the robot has, smoothing and the in-cycle command
delay included. The speed smoothing (tau_v) is a design choice (today 0.85 per 5 ms cycle = 31 ms).

Robust, as on the Hover: the robot answers 1.1-1.6x more than the model above 2 Hz (freq_response.py) and
its loop timing varies, so the gains minimise the summed cost over four plants, motors x1 and x2, each with
and without one more cycle of delay; gains that destabilise any of them are rejected.
"""
import argparse
import warnings

import numpy as np
from scipy.linalg import solve_discrete_are, solve_discrete_lyapunov
from scipy.optimize import minimize

import linear_model as L
from closed_loop import K_TODAY
from model import Params

MOTOR_SCALES = (1.0, 2.0)
# the instant filter states make the Lyapunov solve ill-conditioned (rcond ~1e-16); the costs are unaffected
warnings.filterwarnings("ignore", message="Ill-conditioned matrix")
PLANTS = tuple((s, d) for s in MOTOR_SCALES for d in (False, True))


def closed(K, Az, Bz, C, delay):
    """The closed loop; with delay, the command applied one cycle later (an extra state holds it). Returns
    Acl and the row giving the applied u from the state."""
    n = Az.shape[0]
    if not delay:
        return Az + Bz @ K[None, :] @ C, K[None, :] @ C
    A = np.zeros((n + 1, n + 1))
    A[:n, :n] = Az
    A[:n, n] = Bz[:, 0]
    A[n, :n] = K @ C
    G = np.zeros((1, n + 1))
    G[0, n] = 1.0
    return A, G


def cost(K, plants, Q, R, S0):
    total = 0.0
    for Az, Bz, C, delay in plants:
        Acl, G = closed(K, Az, Bz, C, delay)
        if max(abs(np.linalg.eigvals(Acl))) >= 0.9999:
            return np.inf
        n = Acl.shape[0]
        Qa = np.zeros((n, n))
        Qa[:4, :4] = Q
        Sa = np.zeros((n, n))
        Sa[:4, :4] = S0
        P = solve_discrete_lyapunov(Acl.T, Qa + G.T * R @ G)
        total += float(np.trace(P @ Sa))
    return total / len(plants)


def plants_for(p, a_v, corrected=True):
    out = []
    for scale, delay in PLANTS:
        Az, Bz, C = L.plant(p, a_v=a_v, corrected=corrected, motor=scale)
        out.append((Az, Bz, C, delay))
    return out


def design(p, a_v, X, V, TH, W, U, starts=()):
    pl = plants_for(p, a_v)
    Q = np.diag([1 / X ** 2, 1 / V ** 2, 1 / TH ** 2, 1 / W ** 2])
    R = 1 / U ** 2
    S0 = np.diag([X ** 2, V ** 2, TH ** 2, W ** 2])
    # start from full-state LQR on the bare plant (perfect readings, no delay), applied to the readings
    A, B = L.continuous(p)
    Ad, Bd = L.flow(A, B, L.T)
    P = solve_discrete_are(Ad, Bd, Q, np.array([[R]]))
    Kf = -np.linalg.solve(Bd.T @ P @ Bd + R, Bd.T @ P @ Ad)[0]
    best = None
    for K0 in [Kf * f for f in (1.0, 0.5, 0.3, 0.2)] + [L.from_fw(K_TODAY)] + list(starts):
        if not np.isfinite(cost(K0, pl, Q, R, S0)):
            continue
        r = minimize(cost, K0, args=(pl, Q, R, S0), method="Nelder-Mead",
                     options=dict(xatol=1e-3, fatol=1e-6, maxiter=6000))
        if best is None or r.fun < best.fun:
            best = r
    return (None, np.inf) if best is None else (best.x, best.fun), (pl, Q, R, S0)


def report(label, K, p, a_v, J, corrected=True):
    k = L.to_fw(K)
    print(f"  {label}: gains [{k['x']:.0f} per m, {k['vx']:.2f} per speed unit, {k['q']:.1f} per deg, "
          f"{k['dq']:.2f} per deg/s], cost {J:.2f}")
    Az, Bz, C = L.plant(p, a_v=a_v, corrected=corrected)
    for f, z, tau, mag in L.modes(Az + Bz @ K[None, :] @ C):
        if f > 1e-6:
            print(f"      oscillation {f:5.2f} Hz  damping {z:+.2f}")
        elif tau < 10:
            print(f"      decay       time constant {tau * 1000:6.0f} ms")
    for (Az, Bz, C, delay), (scale, _) in zip(plants_for(p, a_v, corrected), PLANTS):
        Acl, _ = closed(K, Az, Bz, C, delay)
        osc = [m for m in L.modes(Acl) if m[0] > 1e-6]
        z, f = min(((m[1], m[0]) for m in osc), default=(1.0, 0.0))
        ok = max(abs(np.linalg.eigvals(Acl))) < 1
        print(f"      motors x{scale:g}{' + one cycle late' if delay else '':18} "
              f"{'stable' if ok else 'UNSTABLE'}, least damped {z:+.2f} at {f:.1f} Hz")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.015, help="position, m (Hover Tightest 15 mm)")
    ap.add_argument("--v", type=float, default=0.03, help="speed, m/s")
    ap.add_argument("--th", type=float, default=1.0, help="lean, deg")
    ap.add_argument("--w", type=float, default=20.0, help="pitch rate, deg/s")
    ap.add_argument("--u", type=float, default=100.0, help="command, units")
    ap.add_argument("--tau-v", type=float, nargs="+", default=[0.031, 0.015, 0.008])
    a = ap.parse_args()
    p = Params()
    X, V, TH, W, U = a.x, a.v, np.radians(a.th), np.radians(a.w), a.u
    print(f"weights: position {a.x * 1000:.0f} mm, speed {a.v:.2f} m/s, lean {a.th:g} deg, rate {a.w:g} deg/s, "
          f"command {a.u:g} units")
    found = []
    for tv in a.tau_v:
        a_v = float(np.exp(-L.T / tv))
        (K, J), (pl, Q, R, S0) = design(p, a_v, X, V, TH, W, U, starts=found)
        print(f"speed smoothing {tv * 1000:.0f} ms ({a_v:.3f} per cycle):")
        if K is None:
            print("  no gains found stable on all four plants")
        else:
            found.append(K)
            report("LQR on the readings", K, p, a_v, J)
        Kt = L.from_fw(K_TODAY)
        report("today's gains, corrected readings", Kt, p, a_v, cost(Kt, pl, Q, R, S0))
    Kt = L.from_fw(K_TODAY)
    print("today, as flashed (raw encoder, 0.85):")
    report("today's gains", Kt, p, L.A_V_TODAY, cost(Kt, plants_for(p, L.A_V_TODAY, False), Q, R, S0), corrected=False)


if __name__ == "__main__":
    main()
