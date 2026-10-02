#!/usr/bin/env python3
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ZONE_RADIUS = 0.08

ZONE_POSITIONS = {
    "zone_a":    (0.40,  0.15),
    "zone_b":    (0.40,  0.00),
    "zone_c":    (0.40, -0.15),
    "temp_zone": (0.34,  0.30),
}

class PerceptionInterface:
    def __init__(self, node: Node, max_age_sec: float = 1.0):
        self.node = node
        self.max_age_sec = max_age_sec
        self._latest = None
        self._received_at = 0.0
        node.create_subscription(String, "/detected_objects", self._on_msg, 5)

    def _on_msg(self, msg: String):
        self._latest = json.loads(msg.data)
        self._received_at = time.time()

    def detect_objects(self, timeout_sec: float = 3.0) -> dict:
        start = time.time()
        while self._latest is None or (time.time() - self._received_at) > self.max_age_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                raise RuntimeError("Khong nhan duoc du lieu camera moi (object_detector co dang chay khong?)")
        return {o["object"]: (o["x"], o["y"]) for o in self._latest["objects"]}

    def check_zone(self, zone_name: str) -> str | None:
        zx, zy = ZONE_POSITIONS[zone_name]
        objects = self.detect_objects()
        for name, (x, y) in objects.items():
            if ((x - zx) ** 2 + (y - zy) ** 2) ** 0.5 <= ZONE_RADIUS:
                return name
        return None

    def find_free_position(self, exclude_zones: set | None = None) -> str | None:
        exclude_zones = exclude_zones or set()
        candidates = ["temp_zone", "zone_a", "zone_b", "zone_c"]
        for zone in candidates:
            if zone in exclude_zones:
                continue
            if self.check_zone(zone) is None:
                return zone
        return None
