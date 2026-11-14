"""Getting up from a resting pose (2026-11-14): kick the wheels toward the lean, then hand over to the flashed LQR.

The Rider rests stably in two poses (Marc, photos and ruler 11-14):
  back:    on the legs' knee stub, IMU +33.9 deg, the stub touching 60 mm BEHIND the tyre's contact (legs limp, hanging);
  forward: on the chassis' nose prong, IMU -26.3 deg, the prong touching 55 mm AHEAD of the tyre's contact.
Each contact is a small sphere on the body, placed so that the body rests at exactly that angle with the tyre and
the contact both on the floor (worked back to upright: nose ~61 mm ahead of the axle, stub ~65 mm behind; Luwu's
CAD has the prongs at +62 mm and the leg arms' ends at -69 mm). The body is rigid: the legs are as they rest
(limp but holding, Marc), so this is a kick with the legs left limp until the robot is up.

The plant is lqr_in_rl_sim.py's MuJoCo Rider with the measured DC motor (measured_plant.py); the controller is
sim/lqr/closed_loop.flashed() unchanged, run by closed_loop.run (the firmware's timing, IMU loop and actuator
delay). The kick replaces the law's output: at rest (u 0) for T_REST, then +/-U toward the lean until the
firmware's own pitch estimate is within HAND_DEG of upright (or KICK_MAX_S passes), then the LQR from a fresh
plan start, as main.cpp resumes after a fall (planX/planV taken from the robot).

    ./.venv/bin/python getup_sim.py            # sweep: pose x kick command x handover angle x battery
    ./.venv/bin/python getup_sim.py --one back 400 10
"""
import argparse
import os
import sys

import mujoco
import numpy as np

import measured_plant as mp
from lqr_in_rl_sim import MujocoPlant
from rider_params import DEFAULT

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lqr"))
import closed_loop as CL                                             # noqa: E402

UP = CL.FLASHED_UP_DEG                     # the IMU reads this upright (gLqrUp)
# The kick may exceed the firmware's +/-400 (gUMax): the motor model's clamp is opened to the bus's +/-1023, while
# the LQR still clamps its own output at 400 (closed_loop.U_MAX) -- a higher limit for the kick alone. Above 400
# the motor model is extrapolated (fitted within +/-400), and the 8.4 V rail's sag is not modelled (1000 browned out
# the right wheel, ID 21, on 2026-06-18).
mp.U_MAX = 1023.0
POSES = {                                  # IMU at rest (firmware sign: + = leaning back), floor distance (m, + ahead)
    "back": (33.9, -0.060),
    "forward": (-26.3, +0.055),
}
SPHERE_R = 0.003
T_REST, KICK_MAX_S, T_AFTER = 1.5, 0.6, 3.0
FALL_CUT_DEG = 25.0                        # main.cpp gFallDeg: the tilt error that cuts the wheels


def body_angle(imu_deg):
    """Firmware IMU reading -> the sim's body angle (rad, + forward lean, 0 = balanced)."""
    return np.radians(-(imu_deg - UP))


def contact_in_body(imu_deg, floor_dx, r):
    """The contact sphere's centre in the body frame, from its rest angle and floor distance from the tyre."""
    th = body_angle(imu_deg)
    xw, zw = floor_dx, -r + SPHERE_R                                 # relative to the axle, resting
    return xw * np.cos(th) - zw * np.sin(th), xw * np.sin(th) + zw * np.cos(th)


NOMINAL = dict(gain=1.0, fric_scale=1.0, mu_tyre=1.0, mu_contact=1.0, com_dx=0.0, com_dz=0.0, inertia=1.0)
# --robust draws (uniform): the battery; the motor's friction; the tyre on the floor and the stub / nose on it
# (MuJoCo's default 1.0 was a guess); the centre of mass, whose place in a resting pose is NOT known -- the legs
# hang (back) or lie folded flat (forward) instead of at ride height, which moves the body against the axle -- so
# it is bracketed: +-15 mm fore/aft, 30-55 mm above the axle (ride height measured 40.5); the body's inertia x0.7-1.3.
RANGES = dict(gain=(0.85, 1.05), fric_scale=(0.7, 1.3), mu_tyre=(0.5, 1.2), mu_contact=(0.3, 1.0),
              com_dx=(-0.015, 0.015), com_dz=(-0.0105, 0.0145), inertia=(0.7, 1.3))


def draw(rng):
    return {k: float(rng.uniform(*v)) for k, v in RANGES.items()}


class GetupPlant(MujocoPlant):
    """MujocoPlant's Rider with the nose and the knee stub as contacts, settled at rest in `pose`. var: NOMINAL's
    keys (battery, frictions, centre of mass shift in the body frame, inertia scale)."""

    def __init__(self, pose, var=None):
        v = dict(NOMINAL, **(var or {}))
        self.p = mp.measured()
        r = self.p.r
        geoms = ""
        for name, (imu, dx) in POSES.items():
            xb, zb = contact_in_body(imu, dx, r)
            geoms += (f'<geom name="{name}_contact" type="sphere" pos="{xb:.5f} 0 {zb:.5f}" size="{SPHERE_R}" '
                      f'mass="0" friction="{v["mu_contact"]:.3f} 0.005 0.0001" rgba="1 1 1 1"/>\n      ')
        xml = mp.build_mjcf(self.p, CL_RP)
        old_inertial = f'<inertial pos="0 0 {self.p.L}" mass="{self.p.mb}" diaginertia="{self.p.ib_com} {self.p.ib_com} {self.p.ib_com}"/>'
        assert old_inertial in xml, "measured_plant.build_mjcf's body inertial changed: update getup_sim.py"
        ib = self.p.ib_com * v["inertia"]
        xml = xml.replace(old_inertial, f'<inertial pos="{v["com_dx"]:.5f} 0 {self.p.L + v["com_dz"]:.5f}" '
                                        f'mass="{self.p.mb}" diaginertia="{ib} {ib} {ib}"/>')
        # MuJoCo takes the larger of two geoms' frictions: the floor is made frictionless so each part's own counts
        xml = xml.replace('<geom name="floor" type="plane"', '<geom name="floor" type="plane" friction="0 0.005 0.0001"', 1)
        xml = xml.replace('<geom name="wheel" type="cylinder"',
                          f'<geom name="wheel" type="cylinder" friction="{v["mu_tyre"]:.3f} 0.005 0.0001"', 1)
        xml = xml.replace('<body name="wheels"', geoms + '<body name="wheels"', 1)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.model)
        self.motor = mp.DcMotor(self.p, gain=v["gain"], fric_scale=v["fric_scale"])
        q = lambda n: (self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)],
                       self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)])
        (self.qx, self.dx), (self.qp, self.dp), (self.qw, self.dw) = q("slide_x"), q("pitch"), q("wheel_spin")
        if pose is None:                                             # upright (sit_sim.py): settle onto the wheel
            for _ in range(1200):                                    # with the body held level, as MujocoPlant
                self.d.qpos[self.qp] = 0.0
                self.d.qvel[:] = 0.0
                mujoco.mj_step(self.model, self.d)
        else:
            self.d.qpos[self.qp] = body_angle(POSES[pose][0]) * 0.9  # just short of the rest: let it settle onto it
            for _ in range(8000):                                    # 2 s, unpowered
                self.d.ctrl[0] = 0.0
                mujoco.mj_step(self.model, self.d)
        self.rest_deg = np.degrees(self.d.qpos[self.qp])
        self.d.qpos[self.qx] = 0.0; self.d.qpos[self.qw] = 0.0; self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.d)


CL_RP = DEFAULT.__class__(**{**DEFAULT.__dict__, "physics_timestep_s": CL.SUB_S})


class GetupFirmware:
    """The flashed firmware with the kick in front of it. Every cycle the firmware runs as on the robot (its IMU
    estimate and speed smoothing keep going); its output is replaced at rest and during the kick, and its plan is
    restarted (plan_x None = main.cpp's plan start from the robot) at the handover."""

    def __init__(self, fw, u_kick, hand_deg, catch_amax=None):
        """catch_amax: the plan's acceleration limit (m/s^2) for CATCH_S after the handover (its braking from the
        kick's speed), then back to the flashed one; None = the flashed one throughout."""
        self.fw, self.u_kick, self.hand_deg, self.catch_amax = fw, u_kick, hand_deg, catch_amax
        self.amax0 = fw.plan["a_max"]
        self.t, self.phase, self.t_hand = 0.0, "rest", None

    def __getattr__(self, name):                                     # read_imu, read_wheels, hover_sample, ...
        return getattr(self.fw, name)

    def pitch_est(self):
        """The firmware's pitch error (deg, firmware sign: + = back), as the law and the fall cut read it."""
        fw = self.fw
        return (fw.theta2 - UP) if fw.hover_imu else (fw.theta - CL.SET_DEG)

    def control(self, dt, stick=0.0):
        """Decide the phase on this cycle's estimate (read_imu has run), then run the firmware's cycle once."""
        self.t += dt
        q = self.pitch_est()
        if self.phase == "rest" and self.t >= T_REST:
            self.phase, self.t_kick, self.sign = "kick", self.t, (-1.0 if q > 0 else 1.0)   # back -> wheels back
        if self.phase == "kick" and (abs(q) <= self.hand_deg or self.t - self.t_kick >= KICK_MAX_S):
            self.phase, self.t_hand = "lqr", self.t
            self.fw.plan_x = None                                    # resume: a fresh plan from where it is
        if self.catch_amax is not None and self.phase == "lqr":
            self.fw.plan["a_max"] = self.catch_amax if self.t - self.t_hand < CATCH_S else self.amax0
        u = self.fw.control(dt, stick)                               # always run: its estimates stay current
        if self.phase == "rest":
            return 0.0
        if self.phase == "kick":
            return self.sign * self.u_kick
        return u


CATCH_S = 1.5                              # how long a catch_amax holds after the handover


def trial(pose, u_kick, hand_deg, var=None, seed=0, catch_amax=None):
    """One get-up; var: the plant's variation (NOMINAL's keys); seed: the IMU noise and the firmware's cycle draws;
    catch_amax: see GetupFirmware."""
    plant = GetupPlant(pose, var)
    gf = GetupFirmware(CL.flashed(rng=np.random.default_rng(seed)), u_kick, hand_deg, catch_amax)
    L, fell = CL.run(fw=gf, plant=plant, T=T_REST + KICK_MAX_S + T_AFTER, log_all=True, seed=seed)
    L = np.array(L)
    t, u, v, lean = L[:, 0], L[:, 2], L[:, 3], L[:, 4]
    x = np.cumsum(v * np.gradient(t))
    res = dict(pose=pose, u=u_kick, hand=hand_deg, gain=dict(NOMINAL, **(var or {}))["gain"], rest=plant.rest_deg,
               fell=fell, t_hand=None if gf.t_hand is None else gf.t_hand - T_REST, var=var)
    k = (t >= T_REST) & (t < (gf.t_hand if gf.t_hand is not None else np.inf))
    res["kick_lo"], res["kick_hi"] = (lean[k].min(), lean[k].max()) if k.any() else (np.nan, np.nan)
    if gf.t_hand is not None:
        a = t >= gf.t_hand
        start_sign = np.sign(lean[t < T_REST][-1])
        res["over"] = max(0.0, (-start_sign * lean[a]).max())       # past vertical, the far side (deg)
        res["peak"] = np.abs(lean[a]).max()
        res["cut"] = res["peak"] > FALL_CUT_DEG
        res["roll_mm"] = 1000 * np.abs(x[t >= T_REST] - x[t < T_REST][-1]).max()
        res["end_mm"] = 1000 * (x[-1] - x[t < T_REST][-1])           # where it stands at the end, from the start
        res["end_sd"] = lean[t > t[-1] - 1.0].std() if not fell else None
        res["umax"] = np.abs(u[a]).max()
    return res


def show(r):
    s = (f"{r['pose']:8s} kick {r['u']:4.0f}  hand {r['hand']:4.1f}  motors x{r['gain']:.2f}  rest {r['rest']:+6.1f}: ")
    if r["t_hand"] is None:
        return s + f"never handed over (lean during the kick {r['kick_lo']:+.0f}..{r['kick_hi']:+.0f} deg)"
    s += f"up in {r['t_hand']*1000:4.0f} ms, "
    if r["fell"]:
        return s + f"FELL (past 45 deg), peak {r.get('peak', 0):.0f}"
    return s + (f"overshoot {r['over']:4.1f} deg, peak {r['peak']:4.1f}{' (CUT at 25)' if r['cut'] else ''}, "
                f"rolled {r['roll_mm']:4.0f} mm, standing sd {r['end_sd']:.2f} deg")


def ok(r):
    """Stood up and stayed up without tripping the firmware's 25 deg cut."""
    return r["t_hand"] is not None and not r["fell"] and not r.get("cut", True) and r["peak"] < FALL_CUT_DEG


def robust(n, kicks, hand):
    """n randomized robots (RANGES, each its own noise/timing seed) per pose and kick; the same robots for every kick."""
    for pose in POSES:
        for u_kick in kicks:
            rs = []
            for i in range(n):
                rs.append(trial(pose, u_kick, hand, draw(np.random.default_rng(1000 + i)), seed=1000 + i))
            good = [r for r in rs if ok(r)]
            bad = [r for r in rs if not ok(r)]
            print(f"\n{pose:8s} kick {u_kick:.0f}, hand {hand:.0f}: {len(good)}/{n} up and standing")
            if good:
                q = lambda k: np.percentile([r[k] for r in good], [50, 90, 100])
                ms = "/".join(f"{x*1000:.0f}" for x in q("t_hand"))
                ov = "/".join(f"{x:.1f}" for x in q("over"))
                mm = "/".join(f"{x:.0f}" for x in q("roll_mm"))
                print(f"   median/90th pct/max: up in {ms} ms; overshoot {ov} deg; rolled {mm} mm")
            for r in bad:
                vv = r["var"]
                print("   FAIL " + show(r) + "\n        " + " ".join(
                    f"{k}={vv[k]*1000:+.1f}mm" if k.startswith("com") else f"{k}={vv[k]:.2f}" for k in vv))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--one", nargs=3, metavar=("POSE", "U", "HAND"))
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--robust", type=int, metavar="N", help="N randomized robots per pose and kick (RANGES)")
    ap.add_argument("--kicks", type=float, nargs="+", default=[350.0, 400.0])
    ap.add_argument("--hand", type=float, default=10.0)
    ap.add_argument("--com-dx-mm", type=float, default=None,
                    help="the fore/aft CoM range +-this (mm) for --robust (RANGES: 15). Marc 11-14: stood up level on "
                         "the limp legs it balances at once, so its balance point is near the flashed 2.8 deg -- a "
                         "few mm at most (4 mm ~ 5.6 deg)")
    ap.add_argument("--catch-amax", type=float, nargs="+", metavar="M_S2",
                    help="compare the plan's braking limit for the first CATCH_S after the handover (0.6 = flashed): "
                         "how far it travels, how far it leans; N random robots with --robust N")
    a = ap.parse_args()
    if a.com_dx_mm is not None:
        RANGES["com_dx"] = (-a.com_dx_mm / 1000, a.com_dx_mm / 1000)
    if a.catch_amax:
        n = a.robust or 1
        for pose in POSES:
            for am in a.catch_amax:
                rs = [trial(pose, a.kicks[-1], a.hand, draw(np.random.default_rng(1000 + i)) if a.robust else None,
                            seed=1000 + i, catch_amax=am) for i in range(n)]
                good = [r for r in rs if ok(r)]
                if not good:
                    print(f"{pose:8s} catch a_max {am:3.1f}: 0/{n} up and standing")
                    continue
                f = lambda k: np.percentile([abs(r[k]) for r in good], [50, 100])
                print(f"{pose:8s} catch a_max {am:3.1f}: {len(good)}/{n} up; farthest {f('roll_mm')[0]:4.0f}/"
                      f"{f('roll_mm')[1]:4.0f} mm (median/max), ends {f('end_mm')[0]:4.0f} mm from the start; "
                      f"peak lean after the catch {f('peak')[0]:4.1f}/{f('peak')[1]:4.1f} deg", flush=True)
        return
    if a.one:
        print(show(trial(a.one[0], float(a.one[1]), float(a.one[2]), dict(gain=a.gain))))
        return
    if a.robust:
        robust(a.robust, a.kicks, a.hand)
        return
    for gain in (1.0, 0.85):
        for pose in POSES:
            for u_kick in (250, 300, 350, 400):
                for hand in (5.0, 10.0, 15.0):
                    print(show(trial(pose, u_kick, hand, dict(gain=gain))), flush=True)


if __name__ == "__main__":
    main()
