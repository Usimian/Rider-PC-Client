#!/usr/bin/env python3
"""ToF safety governor (runs on the Pi) -- floor-referenced obstacle stop.

Subscribes to the VL53L5CX frames (rider/tof) and the body state
(rider/debug/telem: roll, th=pitch, legR/legL), projects every zone into a
FLOOR-REFERENCED frame using the calibrated kinematic model, and publishes a
FORWARD-speed factor on rider/safety/fwd_limit:

    {"factor": 0.0..1.0, "reason": "none|obstacle", "min_mm": <fwd dist>,
     "on": bool, "would": 0.0..1.0, "cls": "<64 zone classes>",
     "pts": [64 x [fwd, left, up] mm, floor-referenced, or null], "h_mm": <sensor height>,
     "slow_mm": SLOW_FWD, "stop_mm": STOP_FWD}

pts/h_mm/slow_mm/stop_mm feed the web page's 3D depth view (2026-10-27): it draws what the stop decides from.

The DS4 controller (rider_controller.py) multiplies its FORWARD drive by factor
-- reverse is never limited, so you can always back out from a wall.

ON/OFF (2026-10-19): the service always runs and always evaluates, but it only LIMITS when switched
on by the RETAINED message rider/safety/enable {"on": true|false} (the web page's button). Off --
the default until a retained 'on' arrives -- it publishes factor 1.0 while still reporting what it
sees ("would" = the factor it would apply, "reason"/"min_mm", and the per-zone "cls"), so the page
and LCD can show it. cls: one char per zone, row-major (row 0 = top/far, column 0 = robot's left):
'o' obstacle, 'f' floor, 'd' floor dropped away, 'h' overhead, 'b' just below floor, '.' no return.

One stop, forward-only:
  * OBSTACLE -- a return standing ABOVE the floor (z in [OBS_Z_MIN, OBS_Z_MAX])
    in the drive corridor: cap forward, tapering to 0 as it gets close.

No table-edge stop (removed 2026-10-27). At ride height the sensor is ~6 cm above the surface, so the rows that
would see an edge in time (~30 cm out) meet the surface at ~11 deg; as the body rocks back a few degrees they stop
returning, on solid table too. Every edge rule tried either let the robot drive off (it did, three table runs) or
stopped it while standing on solid table (replays: tools/tof/tof_edge_replay.py, captures
data/captures/2026-10-26_tof_*.csv). A wall faces the sensor and returns at any tilt. Edges need a sensor that
looks straight down at the front, as robot vacuums do.

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
ENABLE_TOPIC = "rider/safety/enable"    # retained {"on": bool} from the web page; absent = off

# --- calibrated kinematic model (tof_calib_solve.py) ---
ROLL_SIGN, PITCH_SIGN = -1.0, -1.0      # IMU -> world sign convention
BETA_ROLL, BETA_PITCH = -1.55, 1.22     # fixed mount (shim+bias) offset, deg
H_A, H_C = 0.000172, 0.1381             # sensor height H = H_A*(legR-legL) + H_C  [m]
LEG_IDX_DEFAULT = -441                  # mid ride height, until leg telem arrives

# --- drive corridor + obstacle band ---
COLS = (2, 3, 4, 5)        # center columns = path straight ahead
OBS_Z_MIN = 0.05           # m above floor: obstacle floor. MUST equal FLOOR_TOL (below) -- anything the
                           # floor-detector rejects as "too high to be floor" in the corridor is an obstacle.
                           # A gap here (OBS_Z_MIN > FLOOR_TOL) makes short objects (that band tall) invisible.
OBS_Z_MAX = 0.50           # m above floor: ignore stuff overhead
STOP_FWD = 0.10            # m forward: full stop at/under this (was 0.20; Marc 10-27: half as close)
SLOW_FWD = 0.40            # m forward: full speed beyond; linear taper between (was 0.80)

# --- zone classes (cls, for the page's 8x8 view) ---
FLOOR_TOL = 0.05           # m: |z| under this counts as "floor"
DROP_Z = 0.08              # m: z below -this reads 'd' (floor dropped away), between -FLOOR_TOL and it 'b'

VALID = (5, 6, 9)          # VL53L5CX target_status: 5 = valid, 6 = 50% valid (no wrap-around check),
                           # 9 = valid range with a large pulse (merged target). 9 was rejected before 10-27:
                           # 1-6 zones/frame facing a wall at ~50 cm (tools/tof/tof_raw_vs_verdict.py)

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

_state = {"roll": 0.0, "pitch": 0.0, "legidx": LEG_IDX_DEFAULT, "on": False}
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
    """Floor-reference one frame; return the nearest obstacle ahead, the zone classes, each zone's point and the
    sensor height. Pure (no state) so it can be replayed against captured frames in tests.
    pts[k] = [forward, left, up] in mm, floor-referenced (origin on the floor under the sensor), or None."""
    Ri = _mm(_Rx(math.radians(ROLL_SIGN * roll)), _Ry(math.radians(PITCH_SIGN * pitch)))
    R = _mm(Ri, R_MOUNT)
    H = H_A * legidx + H_C

    nearest_obs = None          # nearest obstacle forward distance (m), drive corridor (COLS) only
    cls = ["."] * 64            # per-zone class, all 64 zones (for display); the stop uses COLS only
    pts = [None] * 64

    for i in range(8):
        for j in range(8):
            k = i * 8 + j
            if s[k] in VALID and d[k] > 0:
                rng = d[k] / 1000.0
                p = _mv(R, (rng * RAY[i][j][0], rng * RAY[i][j][1], rng * RAY[i][j][2]))
                fwd, z = p[0], p[2] + H        # forward dist, height above floor
                pts[k] = [round(fwd * 1000), round(p[1] * 1000), round(z * 1000)]
                if z <= -FLOOR_TOL:
                    cls[k] = "d" if z < -DROP_Z else "b"
                elif z < FLOOR_TOL:
                    cls[k] = "f"
                else:
                    cls[k] = "o" if z < OBS_Z_MAX else "h"
                if j in COLS and OBS_Z_MIN < z < OBS_Z_MAX and fwd > 0:
                    if nearest_obs is None or fwd < nearest_obs:
                        nearest_obs = fwd

    return {"nearest_obs": nearest_obs, "cls": "".join(cls), "pts": pts, "h_mm": round(H * 1000)}


def main():
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_tof_safety")

    def on_connect(c, u, flags, rc, properties=None):
        c.subscribe(TOF_TOPIC)
        c.subscribe(TELEM_TOPIC)
        c.subscribe(ENABLE_TOPIC)

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
        obs = r["nearest_obs"]
        factor = factor_from_fwd(obs) if obs is not None else 1.0     # forward-only
        # report the nearest obstacle ahead at ANY distance -- the screen shows the live distance,
        # not just when limiting.
        if obs is not None:
            reason, mm = "obstacle", int(obs * 1000)
        else:
            reason, mm = "none", -1

        c.publish(LIMIT_TOPIC,
                  json.dumps({"factor": round(factor, 3) if _state["on"] else 1.0, "reason": reason,
                              "min_mm": mm, "on": _state["on"], "would": round(factor, 3), "cls": r["cls"],
                              "pts": r["pts"], "h_mm": r["h_mm"],
                              "slow_mm": round(SLOW_FWD * 1000), "stop_mm": round(STOP_FWD * 1000)}),
                  qos=0, retain=True)

    def on_message(c, u, msg):
        if msg.topic == ENABLE_TOPIC:
            try:
                _state["on"] = bool(json.loads(msg.payload).get("on", False))
            except Exception:
                _state["on"] = False
            print("limiting", "ON" if _state["on"] else "off", flush=True)
            return
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
    mqc.publish(LIMIT_TOPIC, json.dumps({"factor": 1.0, "reason": "none", "min_mm": -1, "on": False,
                                          "would": 1.0, "cls": "." * 64}),
                qos=0, retain=True)
    mqc.loop_stop()
    mqc.disconnect()


if __name__ == "__main__":
    main()
