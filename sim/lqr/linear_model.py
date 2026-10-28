"""The Rider's model made linear about upright, at its control cycle (the Hover's sim/linear_model.py,
for the Rider's firmware timing).

    python3 sim/lqr/linear_model.py        # today's gains on it: modes, frequency, damping

model.accel is linearised numerically about rest with friction and cogging left out (friction gets its own
compensation, as on the Hover). Discretised over one control cycle T (closed_loop.CYCLE_S) with the
firmware's timing inside it: the wheels are read at the start, the IMU at TI (closed_loop.READ_S), and the
new command goes out at TAU (closed_loop.COMMAND_S) -- until then the wheels still run the previous one.

State, at the start of cycle k:  z = (x, v, th, w, u_prev, vf_prev, dqf_prev)
  x, v, th, w   the axle and the body (model.py), u_prev the command running since the last cycle,
  vf_prev       the firmware's smoothed speed after the last cycle (m/s; 'scaled units' / 271),
  dqf_prev      its smoothed gyro rate after the last cycle (rad/s)
Readings at cycle k (each linear in z):
  wheels at the start: x_enc = x - r th, v_enc = v - r w (the wheel against the body)
  IMU at TI: th_i, w_i (the state then, the previous command still running)
  vf  = a_v vf_prev + (1 - a_v) * (v_enc [+ r w_i if corrected])   (per cycle: a_v = 0.85 today)
  dqf = 0.6 dqf_prev + 0.4 w_i
  y = (x_enc [+ r th_i if corrected], vf, th_i, dqf)
The controller u = K y in the model's units (u + forward = the firmware's u; firmware gains: to_fw()).
The pitch is taken as the true lean (the complementary filter is ~gyro over a cycle; its accelerometer
part acts over ~1 s); the position integral is left out (slow).
"""
from dataclasses import replace

import numpy as np
from scipy.linalg import expm

from closed_loop import COMMAND_S, CYCLE_S, K_TODAY, READ_S
from model import Params, plant_module

T, TI, TAU = CYCLE_S, READ_S, COMMAND_S
SPEED_UNITS = 327.6 * 0.828        # firmware 'scaled' speed units per m/s (271)
A_V_TODAY = 0.85
A_Q = 0.6
DEG = 180 / np.pi
def continuous(p=None):
    """A, B for the plant's state s (model.py: (x, v, th, w); model_flex.py: (x, v, thl, wl, thb, wb)) and the
    model's u, friction and cogging off."""
    p = replace(p or Params(), fric=0.0, cog20_rate=0.0, cog24_rate=0.0)
    m = plant_module(p)
    n = len(m.initial(0.0))
    s0, eps = np.zeros(n), 1e-6
    A = np.zeros((n, n))
    for j in range(n):
        d = np.zeros(n)
        d[j] = eps
        A[:, j] = (m.deriv(p, s0 + d, 0.0) - m.deriv(p, s0 - d, 0.0)) / (2 * eps)
    B = ((m.deriv(p, s0, 1.0) - m.deriv(p, s0, -1.0)) / 2)[:, None]
    return A, B


def flow(A, B, t):
    """Phi(t), Gamma(t): s(t) = Phi s(0) + Gamma u for u held over [0, t]."""
    n = A.shape[0]
    M = np.zeros((n + 1, n + 1))
    M[:n, :n], M[:n, n:] = A, B
    E = expm(M * t)
    return E[:n, :n], E[:n, n:]


def weighted(p=None):
    """Indices in z of (x, v, th, w) for the cost: the axle, and the body the IMU is on."""
    B = plant_module(p or Params()).BODY
    return [0, 1, B, B + 1]


def plant(p=None, a_v=A_V_TODAY, corrected=True, motor=1.0, a_q=A_Q, v_corr_by_dqf=False, lqr2=False):
    """Az, Bz (z_{k+1} = Az z_k + Bz u_k) and C (y_k = C z_k); z = (s, u_prev, vf_prev, dqf_prev).
    a_q: the rate smoothing per cycle (A_Q: the firmware's shared 0.6).
    v_corr_by_dqf: correct the speed with the smoothed rate (the Hover's robot.cc) instead of the raw one.
    lqr2: LQR2's IMU path, the Hover's 0.6/0.4 rate smoothing on the IMU task's samples, ~2 a cycle (every ~2.4 ms):
    taken as two, at the cycle's start and at TI (0.36 / 0.24 / 0.4 per cycle)."""
    p = p or Params()
    m = plant_module(p)
    A, B = continuous(p)
    n = A.shape[0]
    N = n + 3
    B = B * motor
    Tc, TIc, TAUc, TWc = T, TI, TAU, 0.0
    PhT, _ = flow(A, B, Tc)
    Ph1, G1 = flow(A, B, TAUc)              # old command over [0, TAU]
    Ph2, G2 = flow(A, B, Tc - TAUc)         # new command over [TAU, T]
    PhI, GI = flow(A, B, TIc)               # the IMU's moment
    PhW, GW = flow(A, B, TWc)               # the wheels' moment
    # readings as rows on z
    s_rows = np.hstack([PhW, GW, np.zeros((n, 2))])                  # the state when the wheels are read
    imu = np.hstack([PhI, GI, np.zeros((n, 2))])                     # the state at TI
    J = np.zeros((2, n))                                             # the encoder (linear in s)
    for j in range(n):
        d = np.zeros(n)
        d[j] = 1.0
        J[:, j] = m.encoder(p, d)
    x_enc, v_enc = J[0] @ s_rows, J[1] @ s_rows
    th_i, w_i = imu[m.BODY], imu[m.BODY + 1]                          # the IMU's body
    e = np.eye(N)
    if lqr2:                                # two samples: the cycle's start (the state z) and TI
        dqf = 0.36 * e[n + 2] + 0.24 * e[m.BODY + 1] + 0.4 * w_i
    else:
        dqf = a_q * e[n + 2] + (1 - a_q) * w_i
    v_meas = v_enc + ((p.r * (dqf if v_corr_by_dqf else w_i)) if corrected else 0.0)
    vf = a_v * e[n + 1] + (1 - a_v) * v_meas
    xpos = x_enc + (p.r * th_i if corrected else 0.0)
    C = np.vstack([xpos, vf, th_i, dqf])
    Az = np.zeros((N, N))
    Bz = np.zeros((N, 1))
    Az[:n, :n] = PhT
    Az[:n, n] = (Ph2 @ G1)[:, 0]
    Bz[:n] = G2
    Bz[n, 0] = 1.0                          # u_k becomes the next cycle's u_prev
    Az[n + 1] = vf
    Az[n + 2] = dqf
    return Az, Bz, C


def to_fw(K):
    """Model gains on y (u = K y; y = x m, vf m/s, th rad, dqf rad/s) -> the firmware's
    u_fw = -(kx x + kv v_scaled + kq q + kdq dq), q and dq in its frame (forward lean negative, deg)."""
    return dict(x=-K[0], vx=-K[1] / SPEED_UNITS, q=K[2] / DEG, dq=K[3] / DEG)


def from_fw(k):
    return np.array([-k["x"], -k["vx"] * SPEED_UNITS, k["q"] * DEG, k["dq"] * DEG])


def modes(Acl):
    """Closed-loop modes: (frequency Hz, damping ratio, time constant s, |eigenvalue|)."""
    out = []
    for lam in np.linalg.eigvals(Acl):
        if lam.imag < -1e-9 or abs(lam) < 1e-9:
            continue
        s = np.log(lam) / T
        wn = abs(s)
        zeta = -s.real / wn if wn > 0 else 1.0
        out.append((s.imag / (2 * np.pi), zeta, -1 / s.real if s.real < 0 else np.inf, abs(lam)))
    return sorted(out, key=lambda m: (m[0], -m[2]))


def main():
    p = Params()
    A, B = continuous(p)
    print("open loop (motors at 0, damping only): s =",
          ", ".join(f"{e.real:+.2f}{e.imag:+.2f}j" for e in np.linalg.eigvals(A)), "1/s")
    K = from_fw(K_TODAY)
    for corrected in (False, True):
        Az, Bz, C = plant(p, corrected=corrected)
        Acl = Az + Bz @ K[None, :] @ C
        print(f"today's gains {K_TODAY}, {'corrected' if corrected else 'raw encoder (as today)'} readings:")
        for f, z, tau, mag in modes(Acl):
            kind = "oscillation" if f > 1e-6 else "decay"
            print(f"  {kind:11} {f:5.2f} Hz  damping {z:+.2f}  time constant {tau:7.3f} s  "
                  f"{'stable' if mag < 1 else 'UNSTABLE'}")


if __name__ == "__main__":
    main()
