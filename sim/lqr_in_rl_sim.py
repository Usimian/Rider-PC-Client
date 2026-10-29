"""The flashed LQR in the RL sim's measured plant (2026-10-29): the known-good controller checks the plant.

The firmware copy is sim/lqr/closed_loop.py's, unchanged (the Hover IMU path, the law, the drive plan with its held
home) -- the one that matched the floor; MujocoPlant puts the RL sim's MuJoCo plant (measured_plant.py) behind the
interface closed_loop.run drives (closed_loop.ModelPlant). If the LQR does on this plant what it does on sim/lqr's
model and on the floor, the plant the policies train on is right; if not, every policy inherits the error.

    ./.venv/bin/python lqr_in_rl_sim.py            # both plants, the same scenarios, side by side
"""
import os
import sys
from dataclasses import replace

import mujoco
import numpy as np

import measured_plant as mp
from rider_params import DEFAULT

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lqr"))
import closed_loop as CL                                             # noqa: E402  (sim/lqr, numpy only)
from drive_design import LQR2, feedforward                           # noqa: E402

A_MAX = 0.6                                  # main.cpp gLqrAMax (2026-10-28)
# the loop's timing: closed_loop.run's default, fw_timing.FW_NOW_LQR (each cycle's period drawn from the robot's)


class MujocoPlant:
    """measured_plant.py's MuJoCo Rider, stepped at closed_loop.SUB_S, offering closed_loop.ModelPlant's interface.
    gain scales the motor (the battery); the wheel sits on the floor's soft contact (it can slip, model.py's cannot)."""
    B = 2

    def __init__(self, th0, gain=1.0):
        self.p = mp.measured()
        self.model = mujoco.MjModel.from_xml_string(
            mp.build_mjcf(self.p, replace(DEFAULT, physics_timestep_s=CL.SUB_S)))
        self.d = mujoco.MjData(self.model)
        self.motor = mp.DcMotor(self.p, gain=gain)
        q = lambda n: (self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)],
                       self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)])
        (self.qx, self.dx), (self.qp, self.dp), (self.qw, self.dw) = q("slide_x"), q("pitch"), q("wheel_spin")
        for _ in range(1200):                                        # settle onto the contact, the body held at th0
            self.d.qpos[self.qp] = th0
            self.d.qvel[:] = 0.0
            mujoco.mj_step(self.model, self.d)
        self.d.qpos[self.qx] = 0.0; self.d.qpos[self.qw] = 0.0; self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.d)

    @property
    def s(self):
        d = self.d
        return np.array([d.qpos[self.qx], d.qvel[self.dx], d.qpos[self.qp], d.qvel[self.dp]])

    def accel_body(self, u):
        return self.d.qacc[self.dx], self.d.qacc[self.dp]              # from the last step (under the held command)

    def step(self, u, dt):
        assert abs(dt - self.model.opt.timestep) < 1e-12
        self.motor.command_units(u)
        self.d.ctrl[0] = self.motor.torque(self.d.qvel[self.dw], self.d.qpos[self.qw])
        mujoco.mj_step(self.model, self.d)

    def kick(self, dw):
        self.d.qvel[self.dp] += dw                                   # the body's rate, the wheels' turn over the
        self.d.qvel[self.dw] -= dw                                   # ground unchanged (model.py's kick)


def firmware():
    p = CL.Params()
    plan = dict(feedforward(p), a_max=A_MAX, t_jerk=0.12, gain=8.0)
    return CL.Firmware(k=LQR2, up_deg=2.8, fric_comp=True, plan=plan, hover_imu=True, tau_v=0.100)


def scenario(plant_name, kind, gain=1.0):
    """One run: returns the per-cycle log (t, stick, u, axle speed, lean deg + forward) and whether it fell."""
    th0 = 0.0
    plant = MujocoPlant(th0, gain) if plant_name == "mujoco" else None
    if plant is None and gain != 1.0:
        plant = CL.ModelPlant(replace(CL.Params(), ku_over_b=CL.Params().ku_over_b * gain), th0)
    kw = dict(fw=firmware(), th0_deg=0.0, log_all=True, plant=plant)
    if kind == "stand":
        return CL.run(T=8.0, **kw)
    if kind == "push":
        return CL.run(T=6.0, kick=(2.0, 90.0), **kw)
    v = float(kind.split()[-1])
    if kind.startswith("start"):                                    # full stick from standing, held, then released
        return CL.run(T=8.0, drive=(np.array([0.0, 1.0, 4.0]), np.array([0.0, v, 0.0])), **kw)
    if kind.startswith("reversal"):                                 # full stick, 0.15 s through zero, full the other way
        return CL.run(T=8.0, drive=(np.array([0.0, 1.0, 4.0, 4.15, 6.5]), np.array([0.0, v, 0.0, -v, 0.0])), **kw)
    raise ValueError(kind)


def measures(kind, L):
    t, u, v, lean = L[:, 0], L[:, 2], L[:, 3], L[:, 4]
    rate = np.gradient(lean, t)
    if kind == "stand":
        m = t > 3.0
        return f"lean sd {lean[m].std():.3f} deg, command sd {u[m].std():4.1f}"
    if kind == "push":
        m = t > 2.0
        x = np.cumsum(v * np.gradient(t))
        return f"lean to {np.abs(lean[m]).max():4.1f} deg, rolled {1000 * np.abs(x[m] - x[t <= 2.0][-1]).max():4.0f} mm"
    m = (t > 1.0) & (t < 2.0)
    out = f"start: swing {np.abs(rate[m]).max():4.0f} deg/s, lean {np.abs(lean[m]).max():4.1f} deg"
    if kind.startswith("start"):
        r = t > 4.0
        x = np.cumsum(v * np.gradient(t))
        out += f"; top {np.abs(v[(t > 1) & (t < 4)]).max():.2f} m/s; release: rolled {1000 * (np.abs(x[r] - x[r][0])).max():4.0f} mm"
    else:
        r = (t > 4.0) & (t < 6.5)
        out += f"; reversal: peak back {-v[r].min():.2f} m/s, lean {np.abs(lean[r]).max():4.1f} deg, command to {np.abs(u[r]).max():.0f}"
    return out


def main():
    global A_MAX
    kinds = ["stand", "push", "start 0.30", "start 0.45", "reversal 0.30", "reversal 0.45"]
    for gain in (1.0, 0.85):
        print(f"\n== motor {gain:.2f}x, timing: {CL.FW_NOW_LQR.name}")
        for kind in kinds:
            for plant in ("model", "mujoco"):
                L, fell = scenario(plant, kind, gain)
                print(f"  {kind:14s} {plant:6s}: " + ("FELL" if fell else measures(kind, np.array(L))))
    # the floor's LQR fall (2026-10-28 evening): top speed 0.6, a_max 1.0, a low battery -- "at maximum speed, it
    # just falls". A faithful plant falls there too, out of motor (the command at +/-400)
    A_MAX, vmax = 1.0, CL.VMAX
    CL.VMAX = 0.6
    print("\n== the floor's fall case: top speed 0.6 m/s, a_max 1.0, full stick from standing (MuJoCo plant)")
    for gain in (1.0, 0.85, 0.75):
        L, fell = scenario("mujoco", "start 0.60", gain)
        L = np.array(L)
        print(f"  motor {gain:.2f}x: " + (f"FELL at {L[-1, 0] - 1.0:.2f} s after the stick" if fell
                                         else measures("start 0.60", L)) + f", command up to {np.abs(L[:, 2]).max():.0f}")
    A_MAX, CL.VMAX = 0.6, vmax


if __name__ == "__main__":
    main()
