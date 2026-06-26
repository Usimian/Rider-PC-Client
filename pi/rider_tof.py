#!/usr/bin/env python3
"""Publish VL53L5CX 8x8 ToF frames to MQTT for the workstation pointcloud node.

Runs on the Pi against the ~/tofvenv venv (has vl53l5cx-ctypes + paho + smbus2):
    ~/tofvenv/bin/python rider_tof.py

Publishes JSON {"res":8,"d":[64 distance_mm],"s":[64 target_status]} to rider/tof.
The workstation `tools/tof/tof_pointcloud_node.py` turns it into a ROS2 PointCloud2.
"""
import json
import time

from smbus2 import SMBus
import paho.mqtt.client as mqtt
import vl53l5cx_ctypes as vl53l5cx

BROKER, PORT, TOPIC = "localhost", 1883, "rider/tof"
RES = 8  # 8x8 = 64 zones


def main():
    mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_tof")
    mqc.connect(BROKER, PORT, keepalive=30)
    mqc.loop_start()

    print("init VL53L5CX (uploads ~84KB firmware)...", flush=True)
    v = vl53l5cx.VL53L5CX(i2c_dev=SMBus(1))
    v.set_resolution(RES * RES)
    try:
        v.set_ranging_frequency_hz(10)
    except Exception:
        pass
    v.start_ranging()
    print("publishing -> rider/tof", flush=True)

    n = 0
    while True:
        if v.data_ready():
            d = v.get_data()
            n_zones = RES * RES
            # ULD results are [NB_TARGET_PER_ZONE=1][n_zones]; take target 0
            frame = {"res": RES,
                     "d": [int(d.distance_mm[0][i]) for i in range(n_zones)],
                     "s": [int(d.target_status[0][i]) for i in range(n_zones)]}
            mqc.publish(TOPIC, json.dumps(frame), qos=0)
            n += 1
            if n % 50 == 0:
                print(f"  {n} frames", flush=True)
        time.sleep(0.02)


if __name__ == "__main__":
    main()
