"""3-D diff-drive MJCF for the joint balance+turn policy (Phase 1 of JOINT_POLICY_PLAN.md).

Extends the 2-D sagittal model (rider_model.py) to a coaxial two-wheel balancer that can
also YAW. Chassis DOF: planar position (slide_x, slide_y), vertical settle (slide_z),
heading (yaw), and balance (pitch); roll is omitted -- the coaxial wheels constrain it on a
flat floor, and leaving it out keeps the obs clean. Two INDEPENDENT wheels at +/-track/2,
each with its own velocity actuator -> differential wheel speed = turning.

Joint chain order matters: yaw is applied BEFORE pitch so the pitch/wheel axes ride the
heading. The 2-D model (rider_model.py) is kept intact as the working balance baseline.
"""
from rider_params import RiderParams


def build_mjcf_3d(p: RiderParams) -> str:
    hx, hy, hz = p.body_half_extent_m
    half_track = p.track_width_m / 2.0
    r = p.wheel_radius_m
    ww = 0.008                      # wheel half-width (m)
    vmax = p.vel_max_rad_s
    frng = p.vel_forcerange_Nm
    return f"""<mujoco model="rider_diffdrive">
  <option timestep="{p.physics_timestep_s}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual><headlight diffuse="0.6 0.6 0.6"/><global offwidth="640" offheight="480"/></visual>
  <default>
    <geom contype="1" conaffinity="1" friction="1.0 0.005 0.0001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.05" rgba="0.3 0.3 0.35 1"/>
    <body name="chassis" pos="0 0 {r}">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <joint name="slide_y" type="slide" axis="0 1 0"/>
      <joint name="slide_z" type="slide" axis="0 0 1"/>
      <joint name="yaw"     type="hinge" axis="0 0 1"/>
      <joint name="pitch"   type="hinge" axis="0 1 0"/>
      <geom name="torso" type="box" pos="0 0 {p.com_height_m}" size="{hx} {hy} {hz}"
            mass="{p.body_mass_kg}" rgba="0.85 0.5 0.2 1"/>
      <body name="wheel_L" pos="0 {half_track} 0">
        <joint name="spin_L" type="hinge" axis="0 1 0" armature="{p.wheel_armature}"/>
        <geom name="wheelL" type="cylinder" fromto="0 {-ww} 0 0 {ww} 0" size="{r}"
              mass="{p.wheel_mass_kg}" rgba="0.1 0.1 0.1 1"/>
      </body>
      <body name="wheel_R" pos="0 {-half_track} 0">
        <joint name="spin_R" type="hinge" axis="0 1 0" armature="{p.wheel_armature}"/>
        <geom name="wheelR" type="cylinder" fromto="0 {-ww} 0 0 {ww} 0" size="{r}"
              mass="{p.wheel_mass_kg}" rgba="0.1 0.1 0.1 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="L" joint="spin_L" kv="{p.vel_kv}" ctrlrange="{-vmax} {vmax}" forcerange="{-frng} {frng}"/>
    <velocity name="R" joint="spin_R" kv="{p.vel_kv}" ctrlrange="{-vmax} {vmax}" forcerange="{-frng} {frng}"/>
  </actuator>
</mujoco>
"""
