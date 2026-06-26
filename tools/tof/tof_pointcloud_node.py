#!/usr/bin/env python3
"""Rider VL53L5CX ToF -> ROS2 PointCloud2 (for rviz2), bridged over MQTT.

The sensor is on the Rider's Pi (no ROS2); the Pi's `rider_tof.py` publishes 8x8
frames to MQTT (rider/tof). This node runs on the WORKSTATION (ROS2 Jazzy),
subscribes to that MQTT topic, projects each zone's range through the sensor's
~45deg x 45deg FoV into a point, and publishes sensor_msgs/PointCloud2 on
/tof/points in frame `tof_link` (a static TF child of `map` is broadcast so rviz
has a fixed frame).

Run (workstation, ROS2 sourced):
    python3 tools/tof/tof_pointcloud_node.py --broker 10.0.0.95
Then in rviz2: Fixed Frame = map (or tof_link), add PointCloud2 on /tof/points.
"""
import argparse
import json
import math
import threading

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# valid VL53L5CX target_status codes to keep (5 = 100% valid, 6 = 50% valid range)
VALID_STATUS = (5, 6)
# sign flips if the cloud comes out mirrored/upside-down vs the real scene
FLIP_AZ = 1.0   # column -> horizontal; flip to -1 if left/right is mirrored
FLIP_EL = 1.0   # row -> vertical; flip to -1 if up/down is mirrored


class TofCloud(Node):
    def __init__(self, broker, port, topic, frame, fov_deg):
        super().__init__("tof_pointcloud")
        self.frame = frame
        self.pub = self.create_publisher(PointCloud2, "/tof/points", 10)

        # static TF: tof_link in map (origin) so rviz has a fixed frame to anchor on
        self._stf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = frame
        t.transform.rotation.w = 1.0
        self._stf.sendTransform(t)

        # per-zone center angles (8 zones spanning fov_deg, centered)
        half = math.radians(fov_deg) / 2.0
        step = math.radians(fov_deg) / 8.0
        self._ang = [-half + (k + 0.5) * step for k in range(8)]

        self._latest = None
        self._lock = threading.Lock()
        self.create_timer(0.05, self._tick)   # 20 Hz republish

        self._mqc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="tof_pc")
        self._mqc.on_message = self._on_msg
        self._mqc.on_connect = lambda c, u, f, rc, p=None: c.subscribe(topic)
        self._mqc.connect(broker, port, keepalive=30)
        self._mqc.loop_start()
        self.get_logger().info(
            f"MQTT {broker}:{port}/{topic} -> /tof/points (frame={frame}, fov={fov_deg}deg)")

    def _on_msg(self, c, u, msg):
        try:
            self._latest = json.loads(msg.payload)
        except Exception:
            pass

    def _tick(self):
        f = self._latest
        if not f:
            return
        d, s = f["d"], f["s"]
        ang = self._ang
        pts = []
        for i in range(8):            # row -> elevation
            ay = ang[i] * FLIP_EL
            for j in range(8):        # col -> azimuth
                k = i * 8 + j
                if s[k] not in VALID_STATUS:
                    continue
                rng = d[k] / 1000.0   # mm -> m
                if rng <= 0.0:
                    continue
                ax = ang[j] * FLIP_AZ
                x = rng
                y = -rng * math.tan(ax)   # +col(right) -> -y (ROS y=left)
                z = -rng * math.tan(ay)   # +row(down)  -> -z (ROS z=up)
                pts.append((x, y, z, float(d[k])))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame
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
    ap.add_argument("--frame", default="tof_link")
    ap.add_argument("--fov", type=float, default=45.0, help="full FoV per side, deg")
    args = ap.parse_args()

    rclpy.init()
    node = TofCloud(args.broker, args.port, args.topic, args.frame, args.fov)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
