#!/usr/bin/env python3
import json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformException

CUBE_TOP_Z = 0.04

COLOR_RANGES = {
    "red_cube":    [((0, 120, 80), (8, 255, 255)), ((170, 120, 80), (179, 255, 255))],
    "yellow_cube": [((22, 120, 80), (35, 255, 255))],
    "blue_cube":   [((100, 120, 60), (130, 255, 255))],
}

MIN_CONTOUR_AREA = 60

class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")
        self.bridge = CvBridge()
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, "/overhead_cam/camera_info", self._on_camera_info, 1)
        self.create_subscription(Image, "/overhead_cam/image_raw", self._on_image, 5)
        self.pub = self.create_publisher(String, "/detected_objects", 5)

        self.get_logger().info("object_detector: cho camera_info va anh...")

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _pixel_to_world(self, u, v, cam_to_base):
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])

        q = cam_to_base.transform.rotation
        t = cam_to_base.transform.translation
        rot = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        ray_base = rot @ ray_cam
        origin = np.array([t.x, t.y, t.z])

        if abs(ray_base[2]) < 1e-6:
            return None
        s = (CUBE_TOP_Z - origin[2]) / ray_base[2]
        point = origin + s * ray_base
        return float(point[0]), float(point[1])

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        return np.array([
            [1 - 2*(y*y+z*z),   2*(x*y-z*w),     2*(x*z+y*w)],
            [2*(x*y+z*w),       1 - 2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),       2*(y*z+x*w),     1 - 2*(x*x+y*y)],
        ])

    def _on_image(self, msg: Image):
        if self.camera_info is None:
            return
        try:
            cam_to_base = self.tf_buffer.lookup_transform(
                "base_link", "overhead_cam_optical_frame", rclpy.time.Time()
            )
        except TransformException:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detections = []
        for name, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
                continue

            M = cv2.moments(largest)
            u = M["m10"] / M["m00"]
            v = M["m01"] / M["m00"]

            world_xy = self._pixel_to_world(u, v, cam_to_base)
            if world_xy is not None:
                detections.append({"object": name, "x": world_xy[0], "y": world_xy[1]})

        out = String()
        out.data = json.dumps({"stamp": self.get_clock().now().nanoseconds, "objects": detections})
        self.pub.publish(out)

def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
