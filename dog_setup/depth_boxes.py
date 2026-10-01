#!/usr/bin/env python3
"""Depth-only object boxes for the Orbbec Dabai DC1 (ROS 2 Foxy).

Subscribes to depth in millimetres, either the compressedDepth topic (16-bit
PNG, lossless, ~20x smaller; the default) or a raw 16UC1 image. Splits the scene
into separate objects at depth discontinuities, and for each object publishes:
  - an annotated depth image with a bounding box and distance label
  - a JSON list of objects (pixel box, depth Z, straight-line range, 3D point)

Outputs
  /depth_boxes/image/compressed  sensor_msgs/CompressedImage  (JPEG, for rqt_image_view)
  /depth_boxes/image             sensor_msgs/Image  (bgr8, only built while subscribed)
  /depth_boxes/objects           std_msgs/String    (JSON)
"""
import array
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String


class DepthBoxes(Node):
    def __init__(self):
        super().__init__('depth_boxes')

        self.declare_parameter('depth_topic', '/topcam/depth/image_raw/compressedDepth')
        self.declare_parameter('info_topic', '/topcam/depth/camera_info')
        self.declare_parameter('min_mm', 300.0)       # ignore anything closer
        self.declare_parameter('max_mm', 5000.0)      # ignore anything farther
        self.declare_parameter('edge_mm', 40.0)       # depth jump that splits objects
        self.declare_parameter('min_area_px', 800)    # drop small blobs / noise
        self.declare_parameter('max_area_frac', 0.5)  # drop blobs bigger than this
                                                      # fraction of the image (floor, wall)
        self.declare_parameter('period_s', 0.0)       # min seconds between processed
                                                      # frames; 0 = every frame

        p = self.get_parameter
        self.min_mm = p('min_mm').value
        self.max_mm = p('max_mm').value
        self.edge_mm = p('edge_mm').value
        self.min_area = p('min_area_px').value
        self.max_area_frac = p('max_area_frac').value
        self.period = p('period_s').value
        self.last_t = None

        self.fx = self.fy = self.cx = self.cy = None
        self.warned_encoding = False

        self.create_subscription(
            CameraInfo, p('info_topic').value, self.on_info, qos_profile_sensor_data)
        depth_topic = p('depth_topic').value
        if depth_topic.endswith('compressedDepth'):
            self.create_subscription(
                CompressedImage, depth_topic, self.on_compressed, qos_profile_sensor_data)
        else:
            self.create_subscription(
                Image, depth_topic, self.on_raw, qos_profile_sensor_data)

        self.pub_img = self.create_publisher(Image, '/depth_boxes/image', 1)
        self.pub_jpg = self.create_publisher(
            CompressedImage, '/depth_boxes/image/compressed', 1)
        self.pub_obj = self.create_publisher(String, '/depth_boxes/objects', 10)
        self.get_logger().info('depth_boxes running, waiting for depth images...')

    def on_info(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def throttled(self):
        now = time.monotonic()
        if self.last_t is not None and now - self.last_t < self.period:
            return True
        self.last_t = now
        return False

    def warn_once(self, text):
        if not self.warned_encoding:
            self.get_logger().error(text)
            self.warned_encoding = True

    def on_compressed(self, msg):
        if self.throttled():
            return
        # compressedDepth = small config header, then a 16-bit PNG in millimetres.
        data = bytes(msg.data)
        start = data.find(b'\x89PNG')
        depth = None
        if start >= 0:
            depth = cv2.imdecode(np.frombuffer(data[start:], np.uint8), cv2.IMREAD_UNCHANGED)
        if depth is None or depth.dtype != np.uint16:
            self.warn_once(f'Could not decode 16-bit depth from format "{msg.format}".')
            return
        self.process(depth, msg.header)

    def on_raw(self, msg):
        if self.throttled():
            return
        if msg.encoding not in ('16UC1', 'mono16'):
            self.warn_once(f'Expected 16UC1 depth, got {msg.encoding}.')
            return
        # Decode without cv_bridge: 16-bit values, row stride may include padding.
        depth = np.frombuffer(msg.data, dtype=np.uint16)
        depth = depth.reshape(msg.height, msg.step // 2)[:, :msg.width]
        self.process(depth, msg.header)

    def process(self, depth, header):
        height, width = depth.shape

        valid = (depth > self.min_mm) & (depth < self.max_mm)

        # Cut the scene apart wherever depth jumps sharply (object edges).
        filled = np.where(valid, depth, self.max_mm).astype(np.float32)
        kernel = np.ones((5, 5), np.uint8)
        grad = cv2.dilate(filled, kernel) - cv2.erode(filled, kernel)
        mask = (valid & (grad < self.edge_mm)).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

        # Annotated depth image (invalid pixels black).
        scaled = cv2.convertScaleAbs(
            np.clip(depth, 0, self.max_mm), alpha=255.0 / self.max_mm)
        vis = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
        vis[~valid] = 0

        max_area = self.max_area_frac * width * height
        objects = []
        for i in range(1, n):
            x, y, w, h, area = (int(v) for v in stats[i])
            if area < self.min_area or area > max_area:
                continue

            region = labels[y:y + h, x:x + w] == i
            z = float(np.median(depth[y:y + h, x:x + w][region])) / 1000.0  # metres

            u, v = x + w / 2.0, y + h / 2.0
            if self.fx:
                X = (u - self.cx) * z / self.fx
                Y = (v - self.cy) * z / self.fy
                rng = math.sqrt(X * X + Y * Y + z * z)
                point = [round(X, 3), round(Y, 3), round(z, 3)]
            else:
                rng, point = z, None

            objects.append({
                'box_xywh': [x, y, w, h],
                'depth_z_m': round(z, 3),
                'range_m': round(rng, 3),
                'point_xyz_m': point,
            })

            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 255, 255), 2)
            label = f'{rng:.2f} m'
            cv2.putText(vis, label, (x, max(y - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        objects.sort(key=lambda o: o['range_m'])

        ok, jpg = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            out = CompressedImage()
            out.header = header
            out.format = 'jpeg'
            out.data = array.array('B', jpg.tobytes())
            self.pub_jpg.publish(out)

        # The raw image is ~0.9 MB; only build it if someone (e.g. RViz) wants it.
        if self.pub_img.get_subscription_count() > 0:
            out = Image()
            out.header = header
            out.height, out.width = height, width
            out.encoding = 'bgr8'
            out.step = width * 3
            # array.array skips Foxy's per-byte Python check (~0.2 s per frame).
            out.data = array.array('B', vis.tobytes())
            self.pub_img.publish(out)

        self.pub_obj.publish(String(data=json.dumps(objects)))


def main():
    rclpy.init()
    node = DepthBoxes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
