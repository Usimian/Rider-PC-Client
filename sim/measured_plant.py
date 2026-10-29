"""The measured Rider as the RL sim's plant (2026-10-28): the body and the motor the LQR was designed on.

sim/lqr/model.py is the single source of the numbers (the measured masses, centre of mass and inertia, and the
measured DC motor, each with its provenance there); this module only puts them into MuJoCo:

  * the body: an explicit inertial -- mb at L above the axle, ib_axle about it (model.py's mb, L, ib_com) -- not
    a box guessed from the outside; the wheels: m_wheels with 2 x iw_one, no extra rotor inertia.
  * the motor (model.py accel()): per wheel T_one = ku * dz(u) - b * rel - fric * tanh(rel / fric_w) + cogging,
    rel = the wheel's turn against the body. The damping b * rel is the wheel joint's own damping (that joint turns
    the wheel against the chassis, as the encoder reads it); the rest is the torque DcMotor sets each physics step.
    u is the firmware's wheel command: policy action x POL_RAW_PER_ACTION (main.cpp: 30 rad/s x 7.764).

The old plant (rider_model.build_mjcf + rider_env.ActuatorModel: a stiff velocity servo with fitted lags) stays
selectable as RiderParams.plant = "servo" until this one is checked against the robot.
"""
import importlib.util
import os
import sys
from dataclasses import dataclass, replace

import numpy as np

_spec = importlib.util.spec_from_file_location(
    "lqr_model", os.path.join(os.path.dirname(os.path.abspath(__file__)), "lqr", "model.py"))
lqr_model = importlib.util.module_from_spec(_spec)                   # sim/lqr/model.py, loaded by path (no sys.path
sys.modules["lqr_model"] = lqr_model                                 # entry, so it shadows nothing); registered
_spec.loader.exec_module(lqr_model)                                  # first, for its dataclass

POL_RAW_PER_ACTION = 30.0 * 7.764           # main.cpp POL_VELMAX_RADS x POL_RAW_PER_RADS: action 1 -> 232.9 units
U_MAX = 400.0                               # main.cpp gUMax: the firmware clamps every command to +/-400


def measured(**over) -> lqr_model.Params:
    """sim/lqr/model.py's measured Rider, with overrides (domain randomization)."""
    return replace(lqr_model.Params(), **over)


def build_mjcf(m: lqr_model.Params, rp) -> str:
    """MuJoCo model of the measured Rider. rp (RiderParams) gives only what model.py does not: the track width,
    the physics step, and the floor's friction."""
    half_track = rp.track_width_m / 2.0
    iw = 2.0 * m.iw_one
    return f"""<mujoco model="rider_measured">
  <option timestep="{rp.physics_timestep_s}" gravity="0 0 -{lqr_model.G}" integrator="implicitfast"/>
  <visual><headlight diffuse="0.6 0.6 0.6"/><global offwidth="640" offheight="480"/></visual>
  <default>
    <geom contype="1" conaffinity="1" friction="1.0 0.005 0.0001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.05" rgba="0.3 0.3 0.35 1"/>
    <body name="chassis" pos="0 0 {m.r}">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <joint name="slide_z" type="slide" axis="0 0 1"/>
      <joint name="pitch"   type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 {m.L}" mass="{m.mb}" diaginertia="{m.ib_com} {m.ib_com} {m.ib_com}"/>
      <geom name="torso" type="box" pos="0 0 {m.L}" size="0.025 0.04 0.05" contype="0" conaffinity="0"
            mass="0" rgba="0.85 0.5 0.2 1"/>
      <body name="wheels" pos="0 0 0">
        <joint name="wheel_spin" type="hinge" axis="0 1 0" damping="{2.0 * m.b}"/>
        <inertial pos="0 0 0" mass="{m.m_wheels}" diaginertia="{iw} {iw} {iw}"/>
        <geom name="wheel" type="cylinder" fromto="0 {-half_track} 0 0 {half_track} 0" size="{m.r}" mass="0"
              rgba="0.1 0.1 0.1 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <!-- the measured motor's torque less its damping (that is the joint's), set by DcMotor each physics step -->
    <motor name="wheel" joint="wheel_spin" gear="1" ctrllimited="false"/>
  </actuator>
</mujoco>
"""


@dataclass
class DcMotor:
    """Both wheel motors, as model.py: the command held between control steps, the torque from it each physics
    step. gain scales ku (the battery: a full pack drives harder) for domain randomization; fric_scale the
    friction. delay_steps: whole control steps between the policy's output and the motor (0 = model.py's timing:
    the command goes out right after the readings it was computed from)."""
    m: lqr_model.Params
    gain: float = 1.0
    fric_scale: float = 1.0
    delay_steps: int = 0

    def __post_init__(self):
        self.reset()

    def reset(self):
        self._queue = [0.0] * (self.delay_steps + 1)
        self.u = 0.0

    def command(self, action: float):
        """The policy's action (policy units, + forward) -> the firmware's wheel command, delayed and clamped."""
        return self.command_units(action * POL_RAW_PER_ACTION)

    def command_units(self, u: float):
        """A wheel command already in the firmware's units (the LQR's output), delayed and clamped."""
        self._queue.append(float(np.clip(u, -U_MAX, U_MAX)))
        self.u = self._queue.pop(0)
        return self.u

    def torque(self, rel: float, phi: float) -> float:
        """Total torque on the wheels (N m, + forward) less the damping: rel = the wheel's rate against the body
        (rad/s), phi = its angle against the body (rad)."""
        m = self.m
        drive = lqr_model.dead_zone(self.u, m.u0)
        cog = 0.0
        for off in (0.0, m.cog_phase2):
            cog += m.b * (m.cog20_rate * np.sin(20 * phi + off) + m.cog24_rate * np.sin(24 * phi + 2 * off))
        return 2.0 * (self.gain * m.ku * drive - self.fric_scale * m.fric * np.tanh(rel / m.fric_w)) + cog
