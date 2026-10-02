#!/usr/bin/env python3
import time
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_ros
from tf2_ros import TransformException

class SkillStatus(Enum):
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    INVALID_OBJECT = "INVALID_OBJECT"
    INVALID_ZONE = "INVALID_ZONE"
    PLANNING_FAILED = "PLANNING_FAILED"

HOME_JOINTS = {
    "shoulder_pan_joint":  0.0,
    "shoulder_lift_joint": -1.57,
    "elbow_joint":         -1.57,
    "wrist_1_joint":       -1.57,
    "wrist_2_joint":        1.57,
    "wrist_3_joint":        0.0,
}

GRASP_ORIENTATION = (0.7071, -0.7071, 0.0, 0.0)

class RobotSkills:
    def __init__(self, node: Node, group_name="ur_manipulator", ee_link="tool0",
                 base_frame="base_link", approach_offset_z=0.15):
        self.node = node
        self.group_name = group_name
        self.ee_link = ee_link
        self.base_frame = base_frame
        self.approach_offset_z = approach_offset_z

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self._move_client = ActionClient(node, MoveGroup, "/move_action")
        self._gripper_client = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")

    def _spin_wait(self, future, timeout_sec=20.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                return None
            time.sleep(0.05)
        return future.result()

    def _send_gripper(self, opening: float, duration_sec: float = 1.0) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["left_finger_joint", "right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [opening, opening]
        point.time_from_start.sec = int(duration_sec)
        goal.trajectory.points = [point]

        future = self._gripper_client.send_goal_async(goal)
        handle = self._spin_wait(future)
        if not handle or not handle.accepted:
            return False
        res = self._spin_wait(handle.get_result_async(), timeout_sec=10.0)
        return res is not None and res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def open_gripper(self) -> bool:
        return self._send_gripper(0.04)

    def close_gripper(self) -> bool:
        return self._send_gripper(0.0)

    def _lookup_pose(self, frame_name: str):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, frame_name,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            p = t.transform.translation
            return (p.x, p.y, p.z)
        except TransformException:
            return None

    def _move_to_joint(self, joint_dict: dict) -> SkillStatus:
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            return SkillStatus.FAILED

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 15
        goal.request.allowed_planning_time = 7.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.planner_id = "RRTConnectkConfigDefault"

        c = Constraints()
        for jname, jval in joint_dict.items():
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jval)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)

        handle = self._spin_wait(self._move_client.send_goal_async(goal))
        if not handle or not handle.accepted:
            return SkillStatus.PLANNING_FAILED

        res = self._spin_wait(handle.get_result_async(), timeout_sec=30.0)
        if not res or res.result.error_code.val != 1:
            return SkillStatus.PLANNING_FAILED
        return SkillStatus.SUCCESS

    def _move_to_pose(self, x, y, z, qx, qy, qz, qw) -> SkillStatus:
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            return SkillStatus.FAILED

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 15
        goal.request.allowed_planning_time = 7.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.planner_id = "RRTConnectkConfigDefault"

        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]
        pc.constraint_region.primitives.append(sp)

        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_link
        oc.orientation.x = float(qx)
        oc.orientation.y = float(qy)
        oc.orientation.z = float(qz)
        oc.orientation.w = float(qw)
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(c)

        handle = self._spin_wait(self._move_client.send_goal_async(goal))
        if not handle or not handle.accepted:
            return SkillStatus.PLANNING_FAILED

        res = self._spin_wait(handle.get_result_async(), timeout_sec=30.0)
        if not res or res.result.error_code.val != 1:
            return SkillStatus.PLANNING_FAILED
        return SkillStatus.SUCCESS

    def home(self) -> SkillStatus:
        return self._move_to_joint(HOME_JOINTS)

    def pick(self, object_name: str) -> SkillStatus:
        xyz = self._lookup_pose(object_name)
        if xyz is None:
            return SkillStatus.INVALID_OBJECT

        x, y, z = xyz
        qx, qy, qz, qw = GRASP_ORIENTATION

        self.open_gripper()

        # Approach
        st = self._move_to_pose(x, y, z + self.approach_offset_z, qx, qy, qz, qw)
        if st != SkillStatus.SUCCESS:
            return st

        # Ha tay xuong gap
        st = self._move_to_pose(x, y, z + 0.05, qx, qy, qz, qw)
        if st != SkillStatus.SUCCESS:
            return st

        self.close_gripper()
        time.sleep(0.5)

        # Retract
        return self._move_to_pose(x, y, z + self.approach_offset_z, qx, qy, qz, qw)

    def place(self, object_name: str, zone_name: str) -> SkillStatus:
        xyz = self._lookup_pose(zone_name)
        if xyz is None:
            return SkillStatus.INVALID_ZONE

        x, y, z = xyz
        qx, qy, qz, qw = GRASP_ORIENTATION

        # Approach zone
        st = self._move_to_pose(x, y, z + self.approach_offset_z, qx, qy, qz, qw)
        if st != SkillStatus.SUCCESS:
            return st

        # Ha xuong dat
        st = self._move_to_pose(x, y, z + 0.05, qx, qy, qz, qw)
        if st != SkillStatus.SUCCESS:
            return st

        self.open_gripper()
        time.sleep(0.5)

        # Retract
        return self._move_to_pose(x, y, z + self.approach_offset_z, qx, qy, qz, qw)
