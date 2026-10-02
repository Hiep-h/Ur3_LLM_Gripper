#!/usr/bin/env python3
import json
import time
from rclpy.node import Node
from std_msgs.msg import String


class PerceptionInterface:
    """Nguon su that duy nhat ve vi tri cube: camera tren cao (/detected_objects).

    Node phai dang duoc spin boi executor o thread khac.
    """

    def __init__(self, node: Node, max_age_sec: float = 1.0):
        self.node = node
        self.max_age_sec = max_age_sec
        self._latest = None
        self._received_at = 0.0
        node.create_subscription(String, "/detected_objects", self._on_msg, 5)

    def _on_msg(self, msg: String):
        self._latest = json.loads(msg.data)
        self._received_at = time.time()

    def detect_objects(self, timeout_sec: float = 5.0, fresh: bool = True) -> dict:
        """Tra ve {object_name: (x, y)} trong base_link.

        fresh=True: doi mot khung hinh MOI (sau thoi diem goi), de tranh doc
        du lieu cu luc tay robot vua di chuyen.
        """
        start = time.time()
        while True:
            have = self._latest is not None
            ok_age = (time.time() - self._received_at) <= self.max_age_sec
            ok_fresh = (not fresh) or self._received_at >= start
            if have and ok_age and ok_fresh:
                break
            if time.time() - start > timeout_sec:
                raise RuntimeError(
                    "Khong nhan duoc du lieu camera moi (object_detector co dang chay khong?)"
                )
            time.sleep(0.05)
        return {o["object"]: (o["x"], o["y"]) for o in self._latest["objects"]}
