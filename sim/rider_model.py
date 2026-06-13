"""MuJoCo MJCF generator for the Rider balancer.

Planar wheeled inverted pendulum (sagittal plane): the chassis has fore/aft
(slide_x), vertical (slide_z, so it settles onto the wheel under gravity), and
pitch DOFs; the wheel pair is one cylinder spinning about the lateral axis and
contacting the floor with friction (that friction is what turns wheel torque
into forward motion -- the same traction the real tight wheel loop gives us).

The model is generated from RiderParams so params live in exactly one place.
"""
from rider_params import RiderParams


def build_mjcf(p: RiderParams) -> str:
    hx, hy, hz = p.body_half_extent_m
    half_track = p.track_width_m / 2.0
    wheel_mass_pair = p.wheel_mass_kg * 2.0
    r = p.wheel_radius_m
    vmax = p.vel_max_rad_s
    frng = p.vel_forcerange_Nm
    return f"""<mujoco model="rider_balancer">
  <option timestep="{p.physics_timestep_s}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual><headlight diffuse="0.6 0.6 0.6"/><global offwidth="640" offheight="480"/></visual>
  <default>
    <geom contype="1" conaffinity="1" friction="1.0 0.005 0.0001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.05" rgba="0.3 0.3 0.35 1"/>
    <body name="chassis" pos="0 0 {r}">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <joint name="slide_z" type="slide" axis="0 0 1"/>
      <joint name="pitch"   type="hinge" axis="0 1 0"/>
      <geom name="torso" type="box" pos="0 0 {p.com_height_m}" size="{hx} {hy} {hz}"
            mass="{p.body_mass_kg}" rgba="0.85 0.5 0.2 1"/>
      <body name="wheels" pos="0 0 0">
        <joint name="wheel_spin" type="hinge" axis="0 1 0"/>
        <geom name="wheel" type="cylinder" fromto="0 {-half_track} 0 0 {half_track} 0"
              size="{r}" mass="{wheel_mass_pair}" rgba="0.1 0.1 0.1 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <!-- velocity servo: the real wheel is velocity-controlled (measured). The
         tight internal loop is modeled stiffly here (kv); the real 13ms lag +
         3ms latency live in ActuatorModel, which shapes the velocity setpoint. -->
    <velocity name="wheel" joint="wheel_spin" kv="{p.vel_kv}"
              ctrlrange="{-vmax} {vmax}" forcerange="{-frng} {frng}"/>
  </actuator>
</mujoco>
"""
