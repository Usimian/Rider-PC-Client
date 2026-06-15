#!/usr/bin/env python3
"""Rider Pi camera service -> MQTT image responder (on-demand).

Bridges the CSI camera (OV5647 via picamera2) to the PC-client GUI using the
request/response image protocol the GUI already speaks:

  GUI  -> rider/control/image_capture  {"request_id", "resolution"}   (high|low|tiny)
  here -> rider/response/image_capture {"request_id","success","image_data"(b64 JPEG),...}

ON-DEMAND: the camera (sensor + ISP) is started on the first request and stopped
after a few idle seconds, so it draws no power when nobody is viewing the feed.
First frame after idle pays a short AE/AWB settle (~0.35 s); during continuous
viewing the GUI's request stream keeps it open.

Kept as a SEPARATE process from rider_status_screen.py (the serial / balance
owner) so camera work can never stall the balance loop. Local broker only.

Run:    /home/pi/xgovenv/bin/python rider_camera.py
"""
import base64
import json
import threading
import time

import cv2
import paho.mqtt.client as mqtt
from picamera2 import Picamera2

BROKER, PORT = "localhost", 1883
TOPIC_REQ = "rider/control/image_capture"
TOPIC_RESP = "rider/response/image_capture"

RES = {"high": (640, 480), "low": (320, 240), "tiny": (160, 120)}
JPEG_QUALITY = 70
IDLE_STOP_S = 6.0          # stop the camera this long after the last request

_lock = threading.Lock()
_picam = None              # Picamera2 | None  (None = stopped, no power)
_last_req = 0.0            # time.monotonic() of the last capture request


def _ensure_started():
    """Lazily start the camera. picamera2 'RGB888' delivers BGR, which is what
    cv2.imencode wants -> correct colors with no channel swap."""
    global _picam
    if _picam is None:
        p = Picamera2()
        p.configure(p.create_video_configuration(main={"size": (640, 480), "format": "RGB888"}))
        p.start()
        time.sleep(0.35)   # let AE/AWB settle before the first frame
        _picam = p


def _stop():
    global _picam
    if _picam is not None:
        try:
            _picam.stop()
            _picam.close()
        except Exception:
            pass
        _picam = None


def capture_jpeg(resolution):
    """Grab a frame (starting the camera if needed), scale, return (b64 jpeg, len)."""
    global _last_req
    w, h = RES.get(resolution, RES["high"])
    with _lock:
        _ensure_started()
        _last_req = time.monotonic()
        frame = _picam.capture_array()          # HxWx3, BGR order
    if (frame.shape[1], frame.shape[0]) != (w, h):
        frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    data = jpg.tobytes()
    return base64.b64encode(data).decode("ascii"), len(data)


def _idle_watchdog():
    """Stop the camera once it's been idle past IDLE_STOP_S -> zero power when unused."""
    while True:
        time.sleep(1.0)
        with _lock:
            if _picam is not None and time.monotonic() - _last_req > IDLE_STOP_S:
                _stop()


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
    threading.Thread(target=_idle_watchdog, daemon=True).start()
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rider_camera")
    c.on_connect = on_connect
    c.on_message = on_message
    c.reconnect_delay_set(min_delay=1, max_delay=10)
    c.connect(BROKER, PORT, keepalive=30)
    c.loop_forever()


if __name__ == "__main__":
    main()
