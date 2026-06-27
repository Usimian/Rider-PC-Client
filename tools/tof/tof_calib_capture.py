#!/usr/bin/env python3
"""Calibration capture for the ToF floor-reference model.

Pairs each VL53L5CX frame (rider/tof) with the body state at that instant
(rider/debug/telem: roll, th=pitch, legR/legL) and writes them to a JSON file.
Run it once per leg height (low / mid / high) on a FLAT floor with the floor
band visible; feed the resulting files to tof_calib_solve.py.

    python3 tools/tof/tof_calib_capture.py --broker 10.0.0.95 --secs 5 calib_mid.json
"""
import argparse
import json
import time

import paho.mqtt.client as mqtt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="output JSON path")
    ap.add_argument("--broker", default="10.0.0.95", help="Pi MQTT broker IP")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--secs", type=float, default=5.0)
    args = ap.parse_args()

    att = {"roll": None, "pitch": None, "legR": None, "legL": None}
    samples = []

    def parse(p):
        d = {}
        for kv in p.split():
            if "=" in kv:
                k, v = kv.split("=", 1); d[k] = v
        return d

    def on_msg(c, u, msg):
        if msg.topic == "rider/debug/telem":
            d = parse(msg.payload.decode())
            try:
                att["roll"] = float(d["roll"]); att["pitch"] = float(d["th"])
                att["legR"] = int(d["legR"]); att["legL"] = int(d["legL"])
            except (KeyError, ValueError):
                pass
            return
        if att["roll"] is None:
            return
        try:
            f = json.loads(msg.payload)
        except Exception:
            return
        samples.append({"d": f["d"], "s": f["s"], "roll": att["roll"],
                        "pitch": att["pitch"], "legR": att["legR"], "legL": att["legL"]})

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="tof_calib")
    c.on_connect = lambda cl, u, fl, rc, p=None: (cl.subscribe("rider/tof"),
                                                  cl.subscribe("rider/debug/telem"))
    c.on_message = on_msg
    c.connect(args.broker, args.port, keepalive=15)
    c.loop_start()
    t0 = time.time()
    while time.time() - t0 < args.secs:
        time.sleep(0.1)
    c.loop_stop()
    with open(args.out, "w") as fh:
        json.dump(samples, fh)
    print("captured %d paired ToF+attitude frames -> %s" % (len(samples), args.out))
    if samples:
        a = samples[len(samples) // 2]
        print("  mid-sample: roll=%.2f pitch=%.2f legidx=%d"
              % (a["roll"], a["pitch"], a["legR"] - a["legL"]))


if __name__ == "__main__":
    main()
