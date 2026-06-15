#!/usr/bin/env python3
"""Rider Pi camera service -> MQTT image responder.

Bridges the CSI camera (OV5647 via picamera2) to the PC-client GUI using the
request/response image protocol the GUI already speaks:

  GUI  -> rider/control/image_capture  {"request_id", "resolution"}   (high|low|tiny)
  here -> rider/response/image_capture {"request_id","success","image_data"(b64 JPEG),...}

Kept as a SEPARATE process from rider_status_screen.py (the serial / balance
owner) so camera capture + JPEG encode can never stall the balance telemetry /
command-relay loop. Talks only to the local mosquitto broker.

Run:    /home/pi/xgovenv/bin/python rider_camera.py
"""
import base64
import json
import threading
import time

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from picamera2 import Picamera2

BROKER, PORT = "localhost", 1883
TOPIC_REQ = "rider/control/image_capture"
TOPIC_RESP = "rider/response/image_capture"

# GUI resolution keys -> output JPEG size
RES = {"high": (640, 480), "low": (320, 240), "tiny": (160, 120)}
JPEG_QUALITY = 70

# --- camera: configure once at 640x480 and keep it open (no per-request init) ---
# picamera2 "RGB888" delivers the array in BGR byte order, which is exactly what
# cv2.imencode expects -> correct colors with no channel swap.
picam = Picamera2()
_cfg = picam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam.configure(_cfg)
picam.start()
time.sleep(0.5)            # let AE/AWB settle before the first frame
_cam_lock = threading.Lock()


def capture_jpeg(resolution):
    """Grab a frame, scale to the requested size, return (base64 jpeg, byte len)."""
    w, h = RES.get(resolution, RES["high"])
    with _cam_lock:
        frame = picam.capture_array()           # HxWx3, BGR order (see note above)
    if (frame.shape[1], frame.shape[0]) != (w, h):
        frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    data = jpg.tobytes()
    return base64.b64encode(data).decode("ascii"), len(data)


def on_connect(client, userdata, flags, reason_code, properties=None):
    client.subscribe(TOPIC_REQ)
    print("camera service connected, subscribed to %s" % TOPIC_REQ, flush=True)


def on_message(client, userdata, msg):
    try:
        p = json.loads(msg.payload.decode())
    except Exception:
        p = {}
    req_id = p.get("request_id", "")
    resolution = p.get("resolution", "high")
    try:
        b64, nbytes = capture_jpeg(resolution)
        client.publish(TOPIC_RESP, json.dumps({
            "request_id": req_id,
            "success": True,
            "image_data": b64,
            "resolution": resolution,
            "image_size": nbytes,
            "timestamp": time.time(),
        }))
    except Exception as e:
        print("capture failed: %s" % e, flush=True)
        client.publish(TOPIC_RESP, json.dumps({
            "request_id": req_id,
            "success": False,
            "error": str(e),
            "timestamp": time.time(),
        }))


def main():
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_camera")
    c.on_connect = on_connect
    c.on_message = on_message
    c.reconnect_delay_set(min_delay=1, max_delay=10)
    c.connect(BROKER, PORT, keepalive=30)
    c.loop_forever()


if __name__ == "__main__":
    main()
