#!/usr/bin/env python3
"""ToF safety governor (runs on the Pi) -- floor-referenced obstacle + cliff.

Subscribes to the VL53L5CX frames (rider/tof) and the body state
(rider/debug/telem: roll, th=pitch, legR/legL), projects every zone into a
FLOOR-REFERENCED frame using the calibrated kinematic model, and publishes a
FORWARD-speed factor on rider/safety/fwd_limit:

    {"factor": 0.0..1.0, "reason": "clear|obstacle|cliff", "min_mm": <fwd dist>}

The DS4 controller (rider_controller.py) multiplies its FORWARD drive by factor
-- reverse is never limited, so you can always back out of a wall OR off a cliff.

Two stops, both forward-only:
  * OBSTACLE -- a return standing ABOVE the floor (z in [OBS_Z_MIN, OBS_Z_MAX])
    in the drive corridor: cap forward, tapering to 0 as it gets close.
  * CLIFF    -- near floor present (row 7 reads z~0: we're on a surface) AND the
    forward floor has vanished (rows 5-6 return nothing / far below floor): the
    surface ends ahead -> force factor 0.

Floor leveling/height is corrected per-frame from the IMU + leg height, so the
balance dance and ride height don't move the floor. Model constants come from
tools/tof/tof_calib_solve.py (3-height flat-floor fit, floor flatness ~11 mm).
Fails open: no frames/telem -> controller keeps factor 1.0.
"""
import json
import math
import signal

import paho.mqtt.client as mqtt

BROKER, PORT = "localhost", 1883
TOF_TOPIC = "rider/tof"
TELEM_TOPIC = "rider/debug/telem"
LIMIT_TOPIC = "rider/safety/fwd_limit"

# --- calibrated kinematic model (tof_calib_solve.py) ---
ROLL_SIGN, PITCH_SIGN = -1.0, -1.0      # IMU -> world sign convention
BETA_ROLL, BETA_PITCH = -1.55, 1.22     # fixed mount (shim+bias) offset, deg
H_A, H_C = 0.000172, 0.1381             # sensor height H = H_A*(legR-legL) + H_C  [m]
LEG_IDX_DEFAULT = -441                  # mid ride height, until leg telem arrives

# --- drive corridor + obstacle band ---
COLS = (2, 3, 4, 5)        # center columns = path straight ahead
OBS_Z_MIN = 0.07           # m above floor: ignore floor itself (and near-floor pitch noise) below this
OBS_Z_MAX = 0.50           # m above floor: ignore stuff overhead
STOP_FWD = 0.20            # m forward: full stop at/under this
SLOW_FWD = 0.80            # m forward: full speed beyond; linear taper between

# --- cliff detection ---
NEAR_ROWS = (7,)           # reliable near-floor band (~0.25 m ahead)
FWD_ROWS = (5, 6)          # forward floor band (~0.5 m ahead) -- vanishes first at an edge
FLOOR_TOL = 0.05           # m: |z| under this counts as "floor"
CLIFF_DROP = 0.08          # m: z below -this counts as "floor gone" (dropped away)
NEAR_MIN = 2               # >= this many near-floor zones valid -> we're on a surface
FWD_MISSING_MIN = 4        # >= this many forward-floor zones gone -> edge ahead
CLIFF_DEBOUNCE = 2         # consecutive cliff frames before asserting (anti-glitch)

VALID = (5, 6)             # VL53L5CX target_status: 5 = valid, 6 = 50% valid

# per-zone center angles (8 zones over 45 deg FoV, centered)
_half, _step = 22.5, 45.0 / 8.0
_ang = [-_half + (k + 0.5) * _step for k in range(8)]


def _unit_ray(i, j):
    el = math.radians(-_ang[i]); az = math.radians(-_ang[j])
    ce, se, ca, sa = math.cos(el), math.sin(el), math.cos(az), math.sin(az)
    return (ce * ca, ce * sa, se)

RAY = [[_unit_ray(i, j) for j in range(8)] for i in range(8)]


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))

def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0, s), (0, 1, 0), (-s, 0, c))

def _mm(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))

def _mv(M, v):
    return tuple(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2] for i in range(3))

R_MOUNT = _mm(_Rx(math.radians(BETA_ROLL)), _Ry(math.radians(BETA_PITCH)))

_state = {"roll": 0.0, "pitch": 0.0, "legidx": LEG_IDX_DEFAULT, "cliff_run": 0}
_running = True


def _stop(*_a):
    global _running
    _running = False


def factor_from_fwd(x):
    if x <= STOP_FWD:
        return 0.0
    if x >= SLOW_FWD:
        return 1.0
    return (x - STOP_FWD) / (SLOW_FWD - STOP_FWD)


def evaluate(d, s, roll, pitch, legidx):
    """Floor-reference one frame; return per-frame obstacle/cliff signals.
    Pure (no state) so it can be replayed against captured frames in tests."""
    Ri = _mm(_Rx(math.radians(ROLL_SIGN * roll)), _Ry(math.radians(PITCH_SIGN * pitch)))
    R = _mm(Ri, R_MOUNT)
    H = H_A * legidx + H_C

    nearest_obs = None          # nearest obstacle forward distance (m)
    near_floor = 0              # near-band zones reading floor
    fwd_missing = 0             # forward-band zones with floor gone
    floor_edge = None           # farthest forward dist where floor is still seen (= the edge)

    for j in COLS:
        for i in range(8):
            k = i * 8 + j
            if s[k] in VALID and d[k] > 0:
                rng = d[k] / 1000.0
                p = _mv(R, (rng * RAY[i][j][0], rng * RAY[i][j][1], rng * RAY[i][j][2]))
                fwd, z = p[0], p[2] + H        # forward dist, height above floor
                if OBS_Z_MIN < z < OBS_Z_MAX and fwd > 0:
                    if nearest_obs is None or fwd < nearest_obs:
                        nearest_obs = fwd
                if abs(z) < FLOOR_TOL and fwd > 0:          # this zone sees floor
                    if floor_edge is None or fwd > floor_edge:
                        floor_edge = fwd                    # push the visible-floor edge outward
                    if i in NEAR_ROWS:
                        near_floor += 1
                elif i in FWD_ROWS and z < -CLIFF_DROP:
                    fwd_missing += 1           # returned, but far below floor = dropped away
            elif i in FWD_ROWS:
                fwd_missing += 1               # no return where forward floor should be

    return {"nearest_obs": nearest_obs, "near_floor": near_floor,
            "fwd_missing": fwd_missing, "floor_edge": floor_edge}


def main():
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_tof_safety")

    def on_connect(c, u, flags, rc, properties=None):
        c.subscribe(TOF_TOPIC)
        c.subscribe(TELEM_TOPIC)

    def on_telem(payload):
        d = {}
        for kv in payload.split():
            if "=" in kv:
                k, v = kv.split("=", 1); d[k] = v
        try:
            _state["roll"] = float(d["roll"]); _state["pitch"] = float(d["th"])
            _state["legidx"] = int(d["legR"]) - int(d["legL"])
        except (KeyError, ValueError):
            pass

    def on_tof(c, f):
        r = evaluate(f["d"], f["s"], _state["roll"], _state["pitch"], _state["legidx"])
        near, miss, edge, obs = r["near_floor"], r["fwd_missing"], r["floor_edge"], r["nearest_obs"]

        # a drop-off is in view when the forward floor is gone (open air ahead, debounced)
        gap = miss >= FWD_MISSING_MIN
        _state["cliff_run"] = _state["cliff_run"] + 1 if gap else 0
        armed = _state["cliff_run"] >= CLIFF_DEBOUNCE

        # cliff: taper to stop ~STOP_FWD short of where the visible floor ends (the edge)
        if armed and near < NEAR_MIN:
            cliff_factor = 0.0                       # no ground under us + gap ahead -> at/over edge
        elif armed and edge is not None:
            cliff_factor = factor_from_fwd(edge)     # stop STOP_FWD before the floor edge
        else:
            cliff_factor = 1.0

        obs_factor = factor_from_fwd(obs) if obs is not None else 1.0

        # --- combine: forward-only, most restrictive wins ---
        factor = min(cliff_factor, obs_factor)
        # report the nearest thing ahead (the active limiter, else nearest obstacle/edge),
        # at ANY distance -- the screen shows the live distance, not just when limiting.
        if armed and cliff_factor <= obs_factor:
            reason, mm = "cliff", (int(edge * 1000) if edge is not None else -1)
        elif obs is not None:
            reason, mm = "obstacle", int(obs * 1000)
        else:
            reason, mm = "none", -1

        c.publish(LIMIT_TOPIC,
                  json.dumps({"factor": round(factor, 3), "reason": reason, "min_mm": mm}),
                  qos=0, retain=True)

    def on_message(c, u, msg):
        if msg.topic == TELEM_TOPIC:
            on_telem(msg.payload.decode())
            return
        try:
            on_tof(c, json.loads(msg.payload))
        except Exception:
            return

    mqc.on_connect = on_connect
    mqc.on_message = on_message
    mqc.reconnect_delay_set(min_delay=1, max_delay=10)
    mqc.connect(BROKER, PORT, keepalive=30)
    mqc.loop_start()
    while _running:
        signal.pause()
    # on exit, clear the limit so a stale retained stop can't strand the drive
    mqc.publish(LIMIT_TOPIC, json.dumps({"factor": 1.0, "reason": "none", "min_mm": -1}),
                qos=0, retain=True)
    mqc.loop_stop()
    mqc.disconnect()


if __name__ == "__main__":
    main()
