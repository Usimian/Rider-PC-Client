#!/usr/bin/env python3
"""ToF safety governor (runs on the Pi).

Subscribes to the VL53L5CX frames (rider/tof, from rider_tof.py) and publishes a
FORWARD-speed factor on rider/safety/fwd_limit:

    {"factor": 0.0..1.0, "min_mm": <nearest center-zone obstacle, mm>}

factor = 1.0 when the path ahead is clear (>= SLOW_MM), tapering linearly to 0.0
at STOP_MM (full stop). The DS4 controller (rider_controller.py) multiplies its
FORWARD drive by this factor -- reverse is never limited, so you can always back
out. Fails open: if this node is down, no messages -> controller keeps factor 1.0.

Only the center zones are watched (the drive path straight ahead); edges/ceiling
are ignored. Tune ROWS/COLS/STOP_MM/SLOW_MM to taste.
"""
import json
import signal

import paho.mqtt.client as mqtt

BROKER, PORT = "localhost", 1883
TOF_TOPIC = "rider/tof"
LIMIT_TOPIC = "rider/safety/fwd_limit"

STOP_MM = 200     # full stop at/under 20 cm
SLOW_MM = 800     # full speed beyond 80 cm; linear taper between
ROWS = range(2, 6)   # 8x8: center band of rows (drive path straight ahead)
COLS = range(2, 6)   #      center columns
VALID = (5, 6)       # VL53L5CX target_status: 5 = valid, 6 = 50% valid range

_running = True


def _stop(*_a):
    global _running
    _running = False


def factor_from_mm(mm):
    if mm <= STOP_MM:
        return 0.0
    if mm >= SLOW_MM:
        return 1.0
    return (mm - STOP_MM) / (SLOW_MM - STOP_MM)


def main():
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_tof_safety")

    def on_connect(c, u, flags, rc, properties=None):
        c.subscribe(TOF_TOPIC)

    def on_tof(c, u, msg):
        try:
            f = json.loads(msg.payload)
        except Exception:
            return
        d, s = f["d"], f["s"]
        nearest = min((d[r * 8 + col] for r in ROWS for col in COLS
                       if s[r * 8 + col] in VALID and d[r * 8 + col] > 0), default=None)
        factor = 1.0 if nearest is None else factor_from_mm(nearest)
        c.publish(LIMIT_TOPIC,
                  json.dumps({"factor": round(factor, 3), "min_mm": nearest if nearest else -1}),
                  qos=0, retain=True)

    mqc.on_connect = on_connect
    mqc.on_message = on_tof
    mqc.reconnect_delay_set(min_delay=1, max_delay=10)
    mqc.connect(BROKER, PORT, keepalive=30)
    mqc.loop_start()
    while _running:
        signal.pause()
    # on exit, clear the limit so a stale retained 'stop' can't strand the drive
    mqc.publish(LIMIT_TOPIC, json.dumps({"factor": 1.0, "min_mm": -1}), qos=0, retain=True)
    mqc.loop_stop()
    mqc.disconnect()


if __name__ == "__main__":
    main()
