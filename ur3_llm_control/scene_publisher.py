#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


class ScenePublisher(Node):
    def __init__(self):
        super().__init__("scene_publisher")

        share_dir = get_package_share_directory("ur3_llm_control")
        scene_path = os.path.join(share_dir, "config", "scene.yaml")
        with open(scene_path, "r") as f:
            self.scene_cfg = yaml.safe_load(f)

        self.base_frame = self.scene_cfg.get("frame_id", "base_link")
        self.broadcaster = StaticTransformBroadcaster(self)

        self._publish_all_tf()
        self._add_collision_boxes()

        self.get_logger().info("Scene publisher: da publish TF cho ban/zone/diem do (khong co cube)")

    def _make_transform(self, frame_name: str, xyz) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = frame_name
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.w = 1.0
        return t

    def _publish_all_tf(self):
        # CHI phat TF cho moc co dinh (ban, zone, diem do).
        # Tuyet doi KHONG phat TF tinh cho cube: vi tri cube lay tu camera.
        transforms = []
        for name, xyz in self.scene_cfg.get("zones", {}).items():
            transforms.append(self._make_transform(name, xyz))
        for name, xyz in self.scene_cfg.get("park_positions", {}).items():
            transforms.append(self._make_transform(name, xyz))
        transforms.append(self._make_transform("work_table", self.scene_cfg["table"]["center"]))
        self.broadcaster.sendTransform(transforms)

    def _add_collision_boxes(self):
        client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("apply_planning_scene khong san sang, bo qua add_box")
            return

        def make_box(name, x, y, z, sx, sy, sz):
            co = CollisionObject()
            co.id = name
            co.header.frame_id = self.base_frame
            co.operation = CollisionObject.ADD
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [sx, sy, sz]
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.w = 1.0
            co.primitives = [prim]
            co.primitive_poses = [pose]
            return co

        table = self.scene_cfg["table"]
        cx, cy, cz = table["center"]
        sx, sy, sz = table["size"]
        objects = [make_box("work_table", cx, cy, cz, sx, sy, sz)]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() and future.result().success:
            self.get_logger().info("Da them collision box mat ban vao planning scene")
        else:
            self.get_logger().warn("Them collision boxes that bai")


def main():
    rclpy.init()
    node = ScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
