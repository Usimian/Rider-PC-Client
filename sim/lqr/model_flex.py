"""The Rider model with its body sprung on its legs (2026-10-28): model.py's robot, split in two above the axle.

PARKED 2026-10-28, not used by the flashed LQR2: designed the Hover's way (0.1 s speed smoothing) the law has no fast
mode on the floor, so the resonance below did not need modelling. fit_flex.py's grid preferred a soft spring
(8 N m/rad: cost 6.5 vs rigid 10.1) and was stopped before its refinement. Pick it up if the Rider's ~20-25 Hz
response (or LQR's 21 Hz buzz) has to be designed around.

Why: a 1-30 Hz sweep on the floor (data/captures/2026-10-28_chirp_lqr_30hz.csv, freq_response.py) shows the robot
answering 1.3x the rigid model at 9 Hz rising to ~4x at 23 Hz with a steeply growing lag -- a lightly damped
resonance at ~20-25 Hz the rigid model lacks; LQR's 21 Hz buzz and LQR2's 14 Hz shake on the floor sit there. The
wheel servos are on the legs and the body (with the IMU) is held on the legs by the leg servos, a spring.

Bodies: the wheels (at the axle), the LEGS (the wheel servos and links: they carry the motors' stator and the
encoders), the BODY (everything else, with the IMU), the legs and the body both pivoting about the axle (the
real joint is the leg servos, above it: an effective spring), joined by a torsional spring K and damper C.
State s = (x, v, thl, wl, thb, wb): the axle, the legs' angle and rate, the body's angle and rate (+ forward lean).
  * the motor torque T acts between the wheels and the LEGS; the encoder reads the wheel against the legs
  * the IMU is on the body (closed_loop reads body angle, rate and the axle/body accelerations)
The legs' share (mass, centre, inertia) is ASSUMED; the body gets the rest, so that the whole robot above the
axle has model.Params' mass, centre of mass and inertia exactly: a stiff spring gives back model.py.
"""
from dataclasses import dataclass

import numpy as np

from model import G, Params, dead_zone

BODY = 4                                   # index of the IMU body's angle in s (its rate at BODY + 1)


@dataclass
class FlexParams(Params):
    flex_k: float = 30.0                   # N m/rad, the body on the legs (to be fitted: fit_flex.py)
    flex_c: float = 0.02                   # N m s/rad
    m_legs: float = 0.080                  # kg, ASSUMED (rider_params.leg_mass_kg: wheel servos ~2 x 20 g + links)
    l_legs: float = 0.020                  # m, the legs' centre above the axle, ASSUMED
    i_legs_com: float = 1.7e-5             # kg m^2, the legs about their own centre, ASSUMED (50 mm rods)

    @property
    def i_legs(self):                      # legs about the axle
        return self.i_legs_com + self.m_legs * self.l_legs ** 2

    @property
    def m_body(self):
        return self.mb - self.m_legs

    @property
    def l_body(self):                      # the body's centre, so the two together sit at model.Params.L
        return (self.mb * self.L - self.m_legs * self.l_legs) / self.m_body

    @property
    def i_body(self):                      # the body about the axle, so the two together are ib_axle
        return self.ib_axle - self.i_legs


def initial(th0):
    return np.array([0.0, 0.0, th0, 0.0, th0, 0.0])


def encoder(p, s):
    """The firmware's wheel_x and wheel speed: the wheel's turn against the LEGS (the servos are on them)."""
    return s[0] - p.r * s[2], s[1] - p.r * s[3]


def accel3(p, s, u):
    """(x'', thl'', thb'') for state s and command u."""
    x, v, thl, wl, thb, wb = s
    rel = v / p.r - wl
    phi = x / p.r - thl
    cog = 0.0
    for off in (0.0, p.cog_phase2):
        cog += p.b * (p.cog20_rate * np.sin(20 * phi + off) + p.cog24_rate * np.sin(24 * phi + 2 * off))
    T = 2.0 * (p.ku * dead_zone(u, p.u0) - p.b * rel - p.fric * np.tanh(rel / p.fric_w)) + cog
    tau = p.flex_k * (thb - thl) + p.flex_c * (wb - wl)          # the spring's pull on the legs (minus on the body)
    ml, ll, mbd, lb = p.m_legs, p.l_legs, p.m_body, p.l_body
    M = np.array([[p.mb + p.m_wheels + 2.0 * p.iw_one / p.r ** 2, ml * ll * np.cos(thl), mbd * lb * np.cos(thb)],
                  [ml * ll * np.cos(thl), p.i_legs, 0.0],
                  [mbd * lb * np.cos(thb), 0.0, p.i_body]])
    rhs = np.array([T / p.r + ml * ll * np.sin(thl) * wl ** 2 + mbd * lb * np.sin(thb) * wb ** 2,
                    ml * G * ll * np.sin(thl) - T + tau,
                    mbd * G * lb * np.sin(thb) - tau])
    return np.linalg.solve(M, rhs)


def accel_body(p, s, u):
    """(x'', the IMU body's angular acceleration), as closed_loop needs them."""
    a = accel3(p, s, u)
    return a[0], a[2]


def deriv(p, s, u):
    a = accel3(p, s, u)
    return np.array([s[1], a[0], s[3], a[1], s[5], a[2]])


def step(p, s, u, dt):
    k1 = deriv(p, s, u)
    k2 = deriv(p, s + 0.5 * dt * k1, u)
    k3 = deriv(p, s + 0.5 * dt * k2, u)
    k4 = deriv(p, s + dt * k3, u)
    return s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


if __name__ == "__main__":
    p = FlexParams()
    f_body = np.sqrt(p.flex_k / p.i_body) / (2 * np.pi)
    print(f"legs {p.m_legs * 1000:.0f} g at {p.l_legs * 1000:.0f} mm, {p.i_legs * 1e4:.2f} kg cm^2; body "
          f"{p.m_body * 1000:.0f} g at {p.l_body * 1000:.1f} mm, {p.i_body * 1e4:.1f} kg cm^2 about the axle; spring "
          f"{p.flex_k:.0f} N m/rad: the body alone on a fixed leg rings at {f_body:.1f} Hz")
