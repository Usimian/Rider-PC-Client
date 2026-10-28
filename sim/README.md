# Rider balancer — RL / sim-to-real scaffold

A MuJoCo + Gymnasium sim of the XGO Rider 2-wheel balancer, for training a
policy in sim and deploying it on the ESP32. The Rider is a good candidate
because its **wheel loop is tight** (low backlash, 1024 cnt/rev, snappy internal
servo loop) — backlash is the #1 killer of both balancers and sim-to-real, and
this robot doesn't have it.

## Status

Scaffold **validated headless** (`sanity_check.py`): zero action falls in ~0.24 s,
a hand-tuned PD balances the full 10 s. Plant is controllable, traction works.
**Not yet trained, not yet on hardware.**

## Files

| file | what |
|---|---|
| `rider_params.py` | all physical + actuator params, tagged by provenance (single source of truth) |
| `rider_model.py`  | generates the MJCF model from params (planar wheeled inverted pendulum) |
| `rider_env.py`    | Gymnasium env + isolated `ActuatorModel` (deadband/sat/lag/latency live here) |
| `measured_plant.py` | the other plant (`RiderParams.plant = "measured"`, 2026-10-28): the body and DC motor of `sim/lqr/model.py` (the LQR's checked model, its numbers used directly); straight-line only, no spin model yet |
| `plant_check.py`  | the measured plant in MuJoCo against `sim/lqr/model.py` on the same commands |
| `sanity_check.py` | headless: builds, falls on zero action, PD balances |
| `train.py`        | SB3 PPO training + eval (net_arch [64,64] → fits on the ESP32) |

Run: `sim/.venv/bin/python sanity_check.py` (or `train.py`).

## Parameter provenance — what we trust vs. what we must measure

**KNOWN** (firmware): wheel r=0.03 m, 1024 cnt/rev, gyro 16.4 LSB/°/s, loop 234 Hz idle / ~199 Hz
driving (MEASURED 2026-09-16 after the wheel-read fix; training randomizes 180-260 Hz).

**Sensing path (2026-09-16):** the policy is trained on what the firmware feeds it, not clean state --
`rider_env.FirmwareSensing` reproduces the encoder-differenced + low-passed velocity (`gPolVelLP`
0.7) and the low-passed pitch rate (`dqf` 0.6). `--legacy_obs` (train.py / drive_sim.py) gives the
old clean-state sim. The actuator pure delay is calibrated so the sim's command→velocity
cross-correlation lag matches the robot's measured 30-35 ms (`drive_sim.py`, see
`docs/loop_delay_budget.md`); the sim reproduces the flashed policy's forward 8.8 Hz ring, its
quiet reverse, and the delay-retrain's full-stick fall. Not yet reproduced: `ppo_drive34_l4`'s
on-robot ~2 Hz build-recover hunt (perfectly smooth in sim).

**ESTIMATED** (refine with a scale + ruler — *no robot power needed*):
- `body_mass_kg`, `wheel_mass_kg` — **weigh the robot** (and a wheel if removable).
- `com_height_m` — CoM height above the axle; eyeball / balance-on-edge.
- `track_width_m`, `body_half_extent_m` — ruler.

**BENCH** (the make-or-break — needs the robot powered, passthrough fw + telemetry):
- `cmd_mode` — does the servo cleanly track **torque** or **velocity**? Pick the tighter one.
- `torque_max_Nm` / `vel_max_rad_s`, `actuator_tau_s`, `deadband_frac`, `latency_s`
- Method: step inputs (`wt` direct drive), log encoder velocity response, fit
  lag + deadband + saturation + delay. The tighter the loop, the narrower the
  domain-randomization band → the higher the transfer confidence.

Until the BENCH params are measured, a trained policy will balance *in sim* but
should **not** be trusted on hardware.

## Deploy path (later)

Train MLP → export weights → hand-rolled inference on the ESP32 at 250 Hz
(policy is tiny, runs in microseconds). Pi sends high-level targets. Strongly
consider **residual RL** (policy corrects our existing balance controller) to
shrink the sim-to-real gap before going end-to-end.
