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
    vmax = 1.25 * p.vel_max_rad_s * max(1.0, p.cmd_limit)   # headroom: DR vel_gain > 1 (and a command range
                                                             # beyond the policy's full scale) must not be clipped
    frng = p.vel_forcerange_Nm
    flex = p.leg_flex_stiffness_Nm_rad is not None
    torso_mass = p.body_mass_kg - (p.leg_mass_kg if flex else 0.0)
    torso = f"""<geom name="torso" type="box" pos="0 0 {p.com_height_m}" size="{hx} {hy} {hz}"
            mass="{torso_mass}" rgba="0.85 0.5 0.2 1"/>"""
    if flex:
        # leg give (2026-10-19 experiment): the torso (IMU) hangs on the legs through a torsional spring about
        # the axle; the wheel servo (and its torque reaction) sits on the legs. 'pitch' is then the LEGS'
        # angle and 'leg_flex' the torso relative to them -- the env adds them for the body pitch.
        # the legs carry the wheel servos: give them that share of the measured body mass (not ~0, which makes
        # a stiff spring ring faster than the time step can follow). Torso + legs geoms don't collide: the torso
        # body is the wheels' SIBLING here (not their parent), so MuJoCo would let the torso box rub the wheel.
        torso = torso.replace("<geom ", '<geom contype="0" conaffinity="0" ')
        torso = f"""<geom name="legs" type="box" pos="0 0 0.02" size="0.01 {hy} 0.02" mass="{p.leg_mass_kg}"
            contype="0" conaffinity="0" rgba="0.6 0.6 0.6 1"/>
      <body name="torso_body" pos="0 0 0">
        <joint name="leg_flex" type="hinge" axis="0 1 0" stiffness="{p.leg_flex_stiffness_Nm_rad}"
               damping="{p.leg_flex_damping}"/>
        {torso}
      </body>"""
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
      {torso}
      <body name="wheels" pos="0 0 0">
        <joint name="wheel_spin" type="hinge" axis="0 1 0" armature="{p.wheel_armature}"/>
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
