"""The RL sim's measured plant (measured_plant.py, MuJoCo) against the model it is built from (sim/lqr/model.py,
RK4): the same command into both, the body's lean and the encoder's travel compared (2026-10-28).

    ./.venv/bin/python plant_check.py

Cases: a fall from 5 deg with the motors at 0 (the masses and inertia), a constant 100-unit command from upright
(the motor's push), a 3 Hz +/-150-unit command from 2 deg (damping, friction and timing together). The MuJoCo
wheel rolls on a soft contact where model.py's cannot slip -- the one difference the build allows.
"""
import mujoco
import numpy as np

import measured_plant as mp
from rider_params import DEFAULT

T = 0.30
CTRL_DT = 1 / 240.0


def mujoco_run(th0, u_of_t):
    m = mp.measured()
    mod = mujoco.MjModel.from_xml_string(mp.build_mjcf(m, DEFAULT))
    d = mujoco.MjData(mod)
    q = {n: (mod.jnt_qposadr[mujoco.mj_name2id(mod, mujoco.mjtObj.mjOBJ_JOINT, n)],
             mod.jnt_dofadr[mujoco.mj_name2id(mod, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in ("slide_x", "pitch", "wheel_spin")}
    for _ in range(300):                       # settle onto the contact, the body held at th0
        d.qpos[q["pitch"][0]] = th0; d.qvel[q["pitch"][1]] = 0.0; d.qvel[q["slide_x"][1]] = 0.0
        d.qvel[q["wheel_spin"][1]] = 0.0
        mujoco.mj_step(mod, d)
    d.qpos[q["slide_x"][0]] = 0.0; d.qpos[q["wheel_spin"][0]] = 0.0; d.time = 0.0
    mujoco.mj_forward(mod, d)
    motor = mp.DcMotor(m)
    decim = max(1, round(CTRL_DT / mod.opt.timestep))
    out = []
    t = 0.0
    while t < T:
        motor.command(u_of_t(t) / mp.POL_RAW_PER_ACTION)
        for _ in range(decim):
            d.ctrl[0] = motor.torque(d.qvel[q["wheel_spin"][1]], d.qpos[q["wheel_spin"][0]])
            mujoco.mj_step(mod, d)
        t += decim * mod.opt.timestep
        enc = d.qpos[q["wheel_spin"][0]] * m.r
        out.append((t, np.degrees(d.qpos[q["pitch"][0]]), enc * 1000))
    return np.array(out)


def model_run(th0, u_of_t, times):
    m = mp.measured()
    L = mp.lqr_model
    s = L.initial(th0)
    out, t, sub = [], 0.0, 0.00025
    for tc in times:
        u = u_of_t(t)
        while t < tc - 1e-9:
            s = L.step(m, s, u, sub)
            t += sub
        out.append((tc, np.degrees(s[2]), L.encoder(m, s)[0] * 1000))
    return np.array(out)


def main():
    cases = [("fall from 5 deg, motors at 0", np.radians(5.0), lambda t: 0.0),
             ("100 units from upright", 0.0, lambda t: 100.0),
             ("3 Hz +/-150 units from 2 deg", np.radians(2.0), lambda t: 150.0 * np.sin(2 * np.pi * 3 * t))]
    for label, th0, u in cases:
        a = mujoco_run(th0, u)
        # model.py holds the command over each control step too: sample u at the same step starts
        steps = np.concatenate([[0.0], a[:-1, 0]])
        held = lambda t, st=steps, uf=u: uf(st[np.searchsorted(st, t + 1e-12, side="right") - 1])
        b = model_run(th0, held, a[:, 0])
        k = a[:, 0] <= T
        print(f"{label:30s}: lean at {T:.2f} s  MuJoCo {a[k][-1, 1]:+7.2f}  model {b[k][-1, 1]:+7.2f} deg "
              f"(largest gap {np.abs(a[k, 1] - b[k, 1]).max():.2f}); encoder MuJoCo {a[k][-1, 2]:+6.1f}  model "
              f"{b[k][-1, 2]:+6.1f} mm (largest gap {np.abs(a[k, 2] - b[k, 2]).max():.1f})")


if __name__ == "__main__":
    main()
