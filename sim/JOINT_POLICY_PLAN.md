# Joint balance + turn policy — plan

**Why:** turning is currently hand-tuned firmware (a yaw differential `tn` summed onto the
symmetric balance torque `u`: `L=u+tn, R=-u+tn`) bolted onto a *balance-only* learned policy.
That additive scheme keeps throwing edge cases on the real (asymmetric) robot:
- stale-frame position homing after a turn (the "spin 2.5× → home the wrong way" bug),
- free fore/aft drift if the position loop is disabled during a turn,
- **one-sided pivot**: when the balance/position common-mode `u` ≈ the turn `tn`, one wheel's
  net command (`−u+tn`) nulls out and that wheel stalls — measured 37–49% of a turn, *not* a
  bus fault (`rfail=0`), and it's the stickier right (dead-leg) side that stalls,
- fast-spin pitch corruption (centripetal accel on the IMU → `th` hit 63° → fall).

A single policy that outputs **per-wheel** commands and is trained with the real per-wheel
asymmetry should coordinate balance+turn natively and dissolve this whole class.

## Current sim (baseline)
- `rider_model.py`: **2-D sagittal** inverted pendulum — `slide_x / slide_z / pitch` + one wheel
  cylinder, **1 velocity actuator**. No yaw, no differential.
- `rider_env.py`: **1 action** (wheel vel); 7-dim obs `[pitch, prate, x_err, x_vel, wheel_vel,
  pitch_int, x_int]` × frame_stack (deployed FS=2); `pure_balance` (position handled in firmware
  code); fore/aft `mirror_aug`; `ActuatorModel` = latency + first-order lag + deadband; DR over
  mass/friction/actuator.
- `rider_params.py`: track 0.097 m, wheel r 0.03, vel_max 30 rad/s, τ 13 ms, latency 3 ms — well
  characterized (bench, 2026-06-13).
- `export_policy.py`: SB3 PPO → `.npz` + `policy.h`; firmware `policyInfer` runs the MLP. Deployed
  net = `ppo_v_pure`.

## Core design decision
**Action = 2 per-wheel velocities `[left_vel, right_vel]`**, NOT common+differential. The policy
commands each wheel directly → there is no "sum that nulls a wheel," so the one-sided-pivot mode
can't form, and the policy can learn to drive the sticky wheel harder.

## Changes by file
**`rider_model.py` — 2-D → 3-D diff-drive**
- Chassis DOF: `x, y, yaw, pitch` (+ `z` settle). Roll is ~constrained by the coaxial wheels;
  allow minimal or lock it for v1.
- Two wheels at ±track/2, each its own spin hinge + velocity actuator (**2 actuators**). Yaw comes
  from differential wheel speed × ground friction.

**`rider_env.py` — the bulk of the work**
- **Two `ActuatorModel`s** (per wheel) with **per-wheel DR** (different gain/stiction/lag/deadband).
  *Make-or-break sim-to-real piece — models the real sticky right wheel.*
- Obs adds `yaw_rate`, per-wheel velocities, and **command inputs** `cmd_fwd`, `cmd_yaw` →
  command-conditioned (one policy: balance + drive + turn + hold).
- Reward: upright + track `cmd_fwd`/`cmd_yaw` + position/heading hold + per-wheel command
  smoothness + **explicit one-wheel-stall penalty** + fall penalty.
- Episode command profiles: spin-in-place, straight drive, combinations, hold.
- `mirror_aug` extended to **left/right** (swap wheels, negate yaw) for turn symmetry.

**`rider_params.py`** — add per-wheel asymmetry params (L/R gain, stiction, deadband) + yaw limits.

**`export_policy.py` + firmware `policyInfer`** — 2-output net + new obs vector. Firmware assembles
the new obs (gyro-Z yaw rate, per-wheel encoder vels, the commands) and applies the **2 outputs
straight to L/R** — deletes the `u+tn` summing (and with it `gYawFF/gYawKp/gTurn/gHdg*` hand-tuning).

## Risks / new work
- Per-wheel asymmetry fidelity — ideally bench-characterize each wheel's stiction/gain (passthrough
  fw + per-wheel telemetry); otherwise wide DR.
- Yaw/friction realism in MuJoCo (lateral traction → realistic turn rate).
- Stable 3-D balancer in sim (roll DOF / coaxial constraint).
- Longer retrain (2 actions + ~10-dim command-conditioned obs); maybe a curriculum (balance first,
  add commands second).
- Mitigant: balance sim-to-real already works, so this is an *extension*, not a leap.

## Phasing
1. **Rebuild the 3-D diff-drive model**; confirm it balances in sim with the 2-action policy (no
   turn commands yet) — validates model + 2-action plumbing.
2. **Add commands + per-wheel DR + retrain** → learns turning/driving/holding.
3. **Export + firmware policy-path rewrite** (2-output, direct per-wheel) + transfer/tune.

Keep the current 1-action balance policy + hand-tuned turn as the working fallback until Phase 3
transfers and is validated on hardware. See [[project_xgo_rider_rl_sim]],
[[project_xgo_rider_balance_solved]].
