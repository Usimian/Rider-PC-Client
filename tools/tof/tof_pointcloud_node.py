#!/usr/bin/env python3
"""Rider VL53L5CX ToF -> ROS2 PointCloud2 (for rviz2), bridged over MQTT,
attitude- AND height-corrected into a floor-referenced `map` frame.

The sensor is on the Rider's (constantly pitching, leg-height-variable) body.
This node subscribes to the 8x8 frames (rider/tof) and the body state
(rider/debug/telem: roll, th=pitch, legR/legL) and publishes a dynamic
tof_link->map transform from a calibrated forward-kinematic model:

    world point = R_imu(s_r*roll, s_p*pitch) . R_mount(beta_r, beta_p) . (d * ray)
    sensor height above floor  H = A*(legR-legL) + C

so points render flat on the floor (z=0) regardless of body tilt OR leg height.
The model constants are from tools/tof/tof_calib_solve.py (3-height flat-floor
fit, floor flatness ~11 mm). Each zone's point is a proper spherical ray d*u(el,az).

Run (workstation, ROS2 sourced):
    python3 tools/tof/tof_pointcloud_node.py --broker 10.0.0.95
Then rviz2: Fixed Frame = map; floor sits at z=0, obstacles show true height.
"""
import argparse
import json
import math

import numpy as np
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

VALID_STATUS = (5, 6)

# --- calibrated kinematic model (tof_calib_solve.py, 3-height flat-floor fit) ---
ROLL_SIGN, PITCH_SIGN = -1.0, -1.0      # IMU -> world sign convention
BETA_ROLL, BETA_PITCH = -1.55, 1.22     # fixed mount (shim+bias) offset, deg
H_A, H_C = 0.000172, 0.1381             # sensor height: H = H_A*(legR-legL) + H_C  [m]
LEG_IDX_DEFAULT = -441                  # mid ride height, used until leg telem arrives

# per-zone center angles (8 zones over 45 deg FoV, centered)
_half, _step = 22.5, 45.0 / 8.0
_ang = [-_half + (k + 0.5) * _step for k in range(8)]


def _unit_ray(i, j):
    el = math.radians(-_ang[i]); az = math.radians(-_ang[j])   # row->elev(down=+i), col->azim
    ce, se, ca, sa = math.cos(el), math.sin(el), math.cos(az), math.sin(az)
    return np.array([ce * ca, ce * sa, se])

RAY = np.array([[_unit_ray(i, j) for j in range(8)] for i in range(8)])  # (8,8,3)


def _Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def quat_from_R(R):
    t = R[0,0] + R[1,1] + R[2,2]
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        return ((R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s)
    i = int(np.argmax([R[0,0], R[1,1], R[2,2]]))
    if i == 0:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return (0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s)
    if i == 1:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return ((R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s)
    s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
    return ((R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s)


def parse_telem(payload):
    d = {}
    for kv in payload.split():
        if "=" in kv:
            k, v = kv.split("=", 1); d[k] = v
    try:
        return float(d["roll"]), float(d["th"]), int(d["legR"]), int(d["legL"])
    except Exception:
        return None


class TofCloud(Node):
    def __init__(self, broker, port, topic, telem_topic, frame):
        super().__init__("tof_pointcloud")
        self.frame = frame
        self.pub = self.create_publisher(PointCloud2, "/tof/points", 10)
        self._tfb = TransformBroadcaster(self)
        self._Rmount = _Rx(math.radians(BETA_ROLL)) @ _Ry(math.radians(BETA_PITCH))
        self._roll = 0.0; self._pitch = 0.0; self._legidx = LEG_IDX_DEFAULT
        self._latest = None
        self.create_timer(0.05, self._tick)   # 20 Hz

        self._topic, self._telem_topic = topic, telem_topic
        self._mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="tof_pc")
        self._mqc.on_message = self._on_msg
        self._mqc.on_connect = self._on_connect
        self._mqc.connect(broker, port, keepalive=30)
        self._mqc.loop_start()
        self.get_logger().info(
            f"MQTT {broker}:{port} {topic}+{telem_topic} -> /tof/points "
            f"(frame={frame}, floor-referenced: tilt+height corrected)")

    def _on_connect(self, c, u, flags, rc, p=None):
        c.subscribe(self._topic); c.subscribe(self._telem_topic)

    def _on_msg(self, c, u, msg):
        if msg.topic == self._telem_topic:
            t = parse_telem(msg.payload.decode())
            if t:
                self._roll, self._pitch, legR, legL = t
                self._legidx = legR - legL
            return
        try:
            self._latest = json.loads(msg.payload)
        except Exception:
            pass

    def _Rworld(self):
        Ri = _Rx(math.radians(ROLL_SIGN * self._roll)) @ _Ry(math.radians(PITCH_SIGN * self._pitch))
        return Ri @ self._Rmount

    def _tick(self):
        now = self.get_clock().now().to_msg()
        R = self._Rworld()
        H = H_A * self._legidx + H_C            # sensor height above floor (m)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = self.frame
        qx, qy, qz, qw = quat_from_R(R)
        t.transform.rotation.x = qx; t.transform.rotation.y = qy
        t.transform.rotation.z = qz; t.transform.rotation.w = qw
        t.transform.translation.z = H          # sensor sits H above the floor (map origin)
        self._tfb.sendTransform(t)

        f = self._latest
        if not f:
            return
        d, s = f["d"], f["s"]
        pts = []
        for i in range(8):
            for j in range(8):
                k = i * 8 + j
                if s[k] not in VALID_STATUS:
                    continue
                rng = d[k] / 1000.0
                if rng <= 0.0:
                    continue
                p = rng * RAY[i, j]            # spherical ray in sensor frame (tof_link)
                pts.append((float(p[0]), float(p[1]), float(p[2]), float(d[k])))
        header = Header(); header.stamp = now; header.frame_id = self.frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.pub.publish(point_cloud2.create_cloud(header, fields, pts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="10.0.0.95", help="Pi MQTT broker IP (matches ssh rider HostName)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--topic", default="rider/tof")
    ap.add_argument("--telem-topic", default="rider/debug/telem")
    ap.add_argument("--frame", default="tof_link")
    args = ap.parse_args()

    rclpy.init()
    node = TofCloud(args.broker, args.port, args.topic, args.telem_topic, args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
