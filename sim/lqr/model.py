"""A model of the Rider as a wheeled inverted pendulum, for designing its LQR (2026-10-28).

Ported from the Hover's sim/hover_model.py (~/hover, commit e2fa373): the two robots have the same
hub-motor wheels and the same ESP32 drive, so the motor model and its fitted numbers start out
shared, and the Rider's own recordings check them (fit_sweep.py). Run with the system Python
(numpy + scipy), not sim/.venv (that is the MuJoCo sim the RL policy trains in).

Planar: the body pitches about the wheel axle while the wheels roll without slipping. States:
x (axle position, m, + forward), v (dx/dt), th (body pitch from vertical, rad, + forward lean),
w (dth/dt). The legs are held rigid at ride height (480/921); the body is everything above the axle.

    (M + Iw/r^2) x'' + mb L cos(th) th'' - mb L sin(th) th'^2 = T / r
    mb L cos(th) x''  + (Ib + mb L^2) th''  - mb g L sin(th)   = -T

M = mb + mw. T is the motors' total torque on the wheels (the body takes -T). Each motor, in the
open-loop drive mode both robots use (wheel register 0x11 = 0), behaves like a voltage-driven DC
motor with friction:

    T_one = ku * dz(u) - b * rel - fric * tanh(rel / fric_w) + b * cogging_rate(phi)
    rel = v/r - w (wheel rate against the body), dz(u) = sign(u) * max(|u| - u0, 0)

u is in the firmware's wheel-command units, + driving forward: the firmware's balance output u_fw
as sent to the left wheel (cmdL = u_fw + turn, cmdR = -u_fw + turn), so model u = (cmdL - cmdR) / 2
= u_fw (MEASURED: cmdL against the encoder's forward speed, +0.85 in the 10-19 sweeps).

The encoders are in the wheel servos on the legs: they read the wheel's turn against the body
(encoder()), MEASURED on the Rider 2026-10-28 (tools/balance/imu_position_fit.py, -0.92 of r x lean
rate with the axle held by hand).

Parameters carry their provenance. Shared with the Hover = the same part, its value from the
Hover's measurements and fits (docs/hover_params.md there), to be checked on the Rider.
"""
from dataclasses import dataclass, replace

import numpy as np

G = 9.81


@dataclass
class Params:
    m_total: float = 0.560     # kg, MEASURED 2026-06-13 (560 g total)
    com_all: float = 0.0405    # m, whole robot's centre of mass above the axle, MEASURED 2026-10-28, legs
                               # powered at ride height: on its back balanced across a round pencil parallel to
                               # the axle, the pencil 40 mm level from the wheel centre (Marc, +/-1-2 mm), the IMU
                               # at 84.3 deg there (data/captures/2026-10-28_pencil_balance_and_flat.csv) and
                               # 3.55 deg balancing upright: 40 / sin(84.3 - 3.55) = 40.5 mm. The 2026-06-13
                               # 45 mm (RL sim) is superseded
    r: float = 0.030           # m, wheel radius, KNOWN (firmware WHEEL_CIRC_M)
    m_wheels: float = 0.110    # kg, both wheel assemblies at the axle, shared with the Hover (ASSUMED 55 g
                               # each there; the RL sim assumed 40 g -- one of the two is wrong)
    iw_one: float = 3.0e-5     # kg m^2, one wheel's rotating part about the axle, shared (ASSUMED)
    # The body's inertia and the motor's loaded behaviour were fitted against each other until both agreed
    # (2026-10-28, three rounds, as on the Hover): fit_drop.py on four hand drop tests (legs at ride height, the
    # motor at u = 0), fit_sweep.py on the 10-19 LQR standing sweep (ku_over_b kept shared).
    ib_axle: float = 17.5e-4   # kg m^2, body about the axle, DERIVED: angle within 0.24-0.99 deg rms, encoder
                               # travel 1.2-2.5 mm rms, release to 30 deg. Gyro-only tipping rate 8.9-9.6 /s
                               # (drop_rate.py; Hover 8.0). Was the RL sim's box, 15.9
    ku_over_b: float = 0.100   # rad/s per unit, shared (Hover MEASURED in the air: 3.0 mm/s per unit);
                               # Rider bench 2026-06-19: 0.129 from a speed chain later found 26 % strong
                               # (rider_params.vel_max_rad_s) -> 0.102
    # The motor, DERIVED: the Hover's values (b 1.53e-3, fric 3.55e-3 = 23 units, fric_w 0.094) unchanged already
    # explained 80 % of the standing sweep's pitch and 85 % of its wheel motion over 60 ms, and 96-98 % of the
    # wheel motion in the driving sweeps, with no speed-dependent lag; these ~94 % / 94 % standing. The friction is
    # ~1.4x the Hover's, as the weight on the wheels is (560 vs 415 g): ASSUMED the cause.
    b: float = 1.63e-3         # N m s/rad, one motor's damping (back-EMF through the driver)
    u0: float = 10.0           # dead zone in the drive, units. ESTIMATED 2026-10-29 (amplitude_check.py): the standing
                               # robot answers the +/-60 sweep 1.6-2.1x the +/-30 per unit of command; 10 units gives
                               # the model 1.3-1.7x (0: 1.1-1.2x; 20-30: stick-slip). The RL sim measured ~19 on
                               # 10-19 under its velocity-servo model. Was 0 ("as on the Hover")
    fric: float = 5.29e-3      # N m, Coulomb friction per wheel (33 units)
    fric_w: float = 0.844      # rad/s, the friction rounded off (tanh) below this
    # Torque off (balancing off: main.cpp writes the wheels' register 0x18 = 0): the wheels turn by hand but feel
    # braked (Marc, 10-28) -- the driver's own load, not the drive at u = 0. off() gives the model in that state.
    # ASSUMED the drive's damping and friction (what fit_drop.py took for the drops) until fitted (fit_joint.py).
    b_off: float = 1.63e-3     # N m s/rad per wheel
    fric_off: float = 5.29e-3  # N m per wheel
    cog20_rate: float = 0.30   # cogging per wheel at 20 and 24 per turn of the wheel against the body,
    cog24_rate: float = 0.23   # rad/s of speed ripple (torque = that x b), shared (Hover MEASURED in the air)
    cog_phase2: float = 1.3    # rad, wheel 2's offset against wheel 1, shared (ASSUMED)

    @property
    def mb(self):  # body mass
        return self.m_total - self.m_wheels

    @property
    def L(self):  # body centre of mass above the axle (the wheels' mass sits at the axle)
        return self.com_all * self.m_total / self.mb

    @property
    def ib_com(self):  # body inertia about its own centre of mass
        return self.ib_axle - self.mb * self.L ** 2

    @property
    def ku(self):  # torque per unit of command above the dead zone, one motor, N m
        return self.ku_over_b * self.b

    def off(self):
        """The robot with its wheels' torque off (balancing off): the driver's braking in place of the drive's
        damping and friction; only meaningful at u = 0."""
        return replace(self, b=self.b_off, fric=self.fric_off)


def encoder(p: Params, s):
    """What the firmware's wheel_x and wheel speed read for state s: x - r th and v - r w (the
    wheel's turn against the body, times r), not the axle's travel. Works on (4, N) states too."""
    return s[0] - p.r * s[2], s[1] - p.r * s[3]


def friction(p, rel):
    """One wheel's friction torque (N m) at rate rel against its body: Coulomb fric rounded through zero by
    tanh(rel / fric_w); model_flex uses the same. (A stiction term was tried 2026-10-29 -- amplitude_check.py --
    and made the standing amplitude dependence worse: removed.)"""
    return p.fric * np.tanh(rel / p.fric_w)


def dead_zone(u, u0):
    return np.sign(u) * np.maximum(np.abs(u) - u0, 0.0)


def accel(p: Params, s, u):
    """Accelerations (x'', th'') for state s = (x, v, th, w) and command u (both wheels the same,
    + drives forward). s may be (4, N) and u (N,): N cases at once."""
    x, v, th, w = s
    rel = v / p.r - w                    # wheel rate against the body, rad/s
    phi = x / p.r - th                   # wheel angle against the body, rad
    drive = dead_zone(u, p.u0)
    cog = 0.0
    for off in (0.0, p.cog_phase2):
        cog += p.b * (p.cog20_rate * np.sin(20 * phi + off) + p.cog24_rate * np.sin(24 * phi + 2 * off))
    T = 2.0 * (p.ku * drive - p.b * rel - friction(p, rel)) + cog
    M = p.mb + p.m_wheels
    a11 = M + 2.0 * p.iw_one / p.r ** 2
    a12 = p.mb * p.L * np.cos(th)
    a22 = p.ib_com + p.mb * p.L ** 2
    rhs1 = T / p.r + p.mb * p.L * np.sin(th) * w ** 2
    rhs2 = p.mb * G * p.L * np.sin(th) - T
    det = a11 * a22 - a12 * a12
    xdd = (rhs1 * a22 - a12 * rhs2) / det
    thdd = (a11 * rhs2 - a12 * rhs1) / det
    return xdd, thdd


# The interface closed_loop.py and linear_model.py use, shared with model_flex.py (the body sprung on its legs):
BODY = 2                                   # index of the IMU body's angle in s (its rate at BODY + 1)


def initial(th0):
    return np.array([0.0, 0.0, th0, 0.0])


def state(p: Params, th, w, x=0.0, v=0.0):
    """A state with the body at angle th, rate w and the axle at x, v (+ forward); works on arrays. model_flex.state
    is the same for the sprung body, so the fits take either."""
    return np.array([x + 0 * th, v + 0 * th, th, w])


def accel_body(p: Params, s, u):
    """(x'', the IMU body's angular acceleration)."""
    return accel(p, s, u)


def deriv(p: Params, s, u):
    xdd, thdd = accel(p, s, u)
    return np.array([s[1], xdd, s[3], thdd])


def plant_module(p):
    """model or model_flex, for these parameters."""
    if hasattr(p, "flex_k"):
        import model_flex
        return model_flex
    import model
    return model


def step(p: Params, s, u, dt):
    """One RK4 step of length dt with the command held."""
    def f(s):
        xdd, thdd = accel(p, s, u)
        return np.array([s[1], xdd, s[3], thdd])
    k1 = f(s)
    k2 = f(s + 0.5 * dt * k1)
    k3 = f(s + 0.5 * dt * k2)
    k4 = f(s + dt * k3)
    return s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


if __name__ == "__main__":
    p = Params()
    lam = np.sqrt(p.mb * G * p.L / p.ib_axle)   # fixed-pivot tipping rate, motors off
    print(f"body {p.mb * 1000:.0f} g, its centre of mass {p.L * 1000:.1f} mm above the axle, inertia "
          f"{p.ib_axle * 1e4:.1f} kg cm^2 about the axle ({p.ib_com * 1e4:.2f} about its centre); "
          f"tipping rate {lam:.1f} /s (the lean doubles every {np.log(2) / lam * 1000:.0f} ms)")
    print(f"motor: b {p.b:.2e} N m s, ku {p.ku:.2e} N m per unit, friction {p.fric / p.ku:.0f} units, "
          f"in the air {p.ku_over_b * p.r * 1000:.1f} mm/s per unit")
