#!/usr/bin/env python3
import time
import rclpy
import rclpy.time
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import tf2_ros
try:
    from gazebo_msgs.msg import ModelStates
except ImportError:  # chi dung de debug, khong bat buoc
    ModelStates = None

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

MOVEIT_ERRORS = {
    -1: "PLANNING_FAILED", -2: "INVALID_MOTION_PLAN", -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED", -5: "UNABLE_TO_AQUIRE_SENSOR_DATA", -6: "TIMED_OUT", -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION", -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION", -13: "GOAL_VIOLATES_PATH_CONSTRAINTS", -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME", -16: "INVALID_GOAL_CONSTRAINTS", -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME", -19: "INVALID_OBJECT_NAME", -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE", -23: "ROBOT_STATE_STALE", -24: "SENSOR_INFO_STALE",
    -31: "NO_IK_SOLUTION",
}


# Cao do tool0 (base_link) khi gripper om vua cube 4cm / tha cube xuong ban.
# Ngon gripper dai 6cm, tam ngon nam 0.134 duoi tool0 -> day ngon = tool0_z - 0.164.
# IK/dieu khien co the lech ~1cm, nen de day ngon cach ban >= ~2cm (da quan sat PICK_Z=0.17 lam day ngon
# cham ban: ngon dinh xuong ban vi ma sat lon, khong khep duoc).
# Cube nam tren mieng zone day 7.5mm: cube cao tu 0.0075 den 0.0475.
PICK_Z = 0.19
PLACE_Z = 0.195
PRE_GRASP_DZ = 0.05
# Ngon gripper: mat trong o +-(0.015 + q). Cube 4cm (nua be rong 0.02) cham ngon o q = 0.005.
FINGER_OPEN = 0.04
# Ngon dieu khien bang LUC (effort PID, xem config/gripper_controllers.yaml). Lenh dong ve 0:
# khi gap cube 4cm, ngon bi chan o q ~ 0.005 va luc ep = p * 0.005 (~2 N moi ben, p = 400); p thap + controller 1000 Hz de khong dao dong.
FINGER_CLOSE = 0.0
# Co cube giua 2 ngon: tong q hai ngon ~ 0.010. Khong co cube: ngon dong het, tong ~ 0.
GRASP_MIN_SUM = 0.006
CUBE_LIFTED_Z = 0.06  # cube cao hon muc nay sau khi nhac thi coi la dang duoc giu (tam cube luc nam ban ~0.02)
LIFT_DZ = 0.08

class RobotSkills:
    def __init__(self, node: Node, perception, landmarks: dict,
                 group_name="ur_manipulator", ee_link="tool0", base_frame="base_link"):
        self.node = node
        self.group_name = group_name
        self.ee_link = ee_link
        self.base_frame = base_frame
        self.perception = perception
        self.landmarks = landmarks  # zone / diem do co dinh: {name: [x, y, z]}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self._finger_pos = {}
        self._joint_pos = {}
        self._ik_client = node.create_client(GetPositionIK, "/compute_ik")
        node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        # CHI DE DEBUG: do cao that cua cube trong Gazebo (khong dung de lap ke hoach/quyet dinh).
        self._model_z = {}
        if ModelStates is not None:
            node.create_subscription(ModelStates, "/gazebo/model_states", self._on_model_states, 5)

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
        if not self._gripper_client.wait_for_server(timeout_sec=20.0):
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
        return self._send_gripper(FINGER_OPEN, duration_sec=2.0)

    def close_gripper(self) -> bool:
        # Dong cham ve FINGER_CLOSE (=0) bang dieu khien LUC: gap cube thi ngon bi chan va chi ep
        # voi luc gioi han (p * sai so), khong ban cube ra nhu dieu khien vi tri truc tiep.
        return self._send_gripper(FINGER_CLOSE, duration_sec=4.0)

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_pos[name] = pos
            if name in ("left_finger_joint", "right_finger_joint"):
                self._finger_pos[name] = pos

    def _on_model_states(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            self._model_z[name] = (pose.position.x, pose.position.y, pose.position.z)

    def _log_cube(self, name: str, label: str):
        """Log vi tri that cua cube trong Gazebo de debug (cube co len cung gripper khong)."""
        p = self._model_z.get(name)
        if p is not None:
            self.node.get_logger().info(f"DEBUG {label}: {name} that o x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}")

    def _log_links(self, label: str):
        """CHI DE DEBUG: do cao (z) cac khop tay may va ngon gripper trong base_link."""
        parts = []
        for link in ("forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link",
                     "tool0", "left_finger_link", "right_finger_link"):
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, link, rclpy.time.Time())
                p = t.transform.translation
                parts.append(f"{link}=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
            except Exception:
                parts.append(f"{link}=?")
        self.node.get_logger().info(f"DEBUG {label}: " + " ".join(parts))

    def _cube_snapshot(self) -> dict:
        return {n: p for n, p in self._model_z.items() if n.endswith("_cube")}

    def _report_moved(self, before: dict, label: str):
        """CHI DE DEBUG: bao cac cube (thuc te trong Gazebo) da bi xe dich > 1cm so voi truoc."""
        moved = []
        for n, p in self._cube_snapshot().items():
            b = before.get(n)
            if b is not None:
                d = ((p[0] - b[0]) ** 2 + (p[1] - b[1]) ** 2 + (p[2] - b[2]) ** 2) ** 0.5
                if d > 0.01:
                    moved.append(f"{n} lech {d * 100:.1f}cm (z={p[2]:.3f})")
        self.node.get_logger().info(f"DEBUG {label}: " + ("cube bi dich: " + "; ".join(moved) if moved else "khong cube nao bi dich"))

    def _verify_grasp(self) -> bool:
        """Co cube giua 2 ngon neu tong do mo q_trai + q_phai >= GRASP_MIN_SUM.

        Khoang hep giua 2 mat ngon = 0.03 + q_trai + q_phai. Cube rong 4cm -> tong q ~ 0.010.
        Khong co cube: ngon dong het (q ~ 0) -> tong q ~ 0.
        Dung TONG (khong xet tung ngon) vi cube co the lech ve mot phia.
        """
        time.sleep(0.8)
        if len(self._finger_pos) < 2:
            return True  # khong co du lieu -> khong ket luan la hut
        ql = self._finger_pos.get("left_finger_joint", 0.0)
        qr = self._finger_pos.get("right_finger_joint", 0.0)
        total = ql + qr
        self.node.get_logger().info(
            f"Kiem tra gap: left={ql:.4f} right={qr:.4f} tong={total:.4f} (co cube neu >= {GRASP_MIN_SUM})")
        return GRASP_MIN_SUM <= total < 0.07

    def _move_to_joint(self, joint_dict: dict, label: str = "joint goal (home)") -> SkillStatus:
        if not self._move_client.wait_for_server(timeout_sec=20.0):
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
            self.node.get_logger().error(f"MoveIt tu choi goal: {label}")
            return SkillStatus.PLANNING_FAILED

        res = self._spin_wait(handle.get_result_async(), timeout_sec=30.0)
        if not res:
            self.node.get_logger().error(f"MoveIt khong tra ket qua (timeout): {label}")
            return SkillStatus.PLANNING_FAILED
        code = res.result.error_code.val
        if code != 1:
            self.node.get_logger().error(
                f"MoveIt that bai: {label} -> {MOVEIT_ERRORS.get(code, code)} ({code})")
            return SkillStatus.PLANNING_FAILED
        return SkillStatus.SUCCESS

    def _move_to_pose(self, x, y, z, qx, qy, qz, qw) -> SkillStatus:
        if not self._move_client.wait_for_server(timeout_sec=20.0):
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
            self.node.get_logger().error(f"MoveIt tu choi goal: pose goal x={x:.3f} y={y:.3f} z={z:.3f}")
            return SkillStatus.PLANNING_FAILED

        res = self._spin_wait(handle.get_result_async(), timeout_sec=30.0)
        if not res:
            self.node.get_logger().error(f"MoveIt khong tra ket qua (timeout): pose goal x={x:.3f} y={y:.3f} z={z:.3f}")
            return SkillStatus.PLANNING_FAILED
        code = res.result.error_code.val
        if code != 1:
            self.node.get_logger().error(
                f"MoveIt that bai: pose goal x={x:.3f} y={y:.3f} z={z:.3f} -> {MOVEIT_ERRORS.get(code, code)} ({code})")
            return SkillStatus.PLANNING_FAILED
        return SkillStatus.SUCCESS

    def home(self) -> SkillStatus:
        return self._move_to_joint(HOME_JOINTS)

    def _ik_joints(self, x, y, z):
        """Giai IK (MoveIt /compute_ik) khoi dau tu tu the khop HIEN TAI, de luon chon cach gap tay
        gan tu the hien tai (tranh MoveIt chon ngau nhien cach gap khac, vi du cang tay ha thap quet cube)."""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        for jn in HOME_JOINTS:
            if jn in self._joint_pos:
                req.ik_request.robot_state.joint_state.name.append(jn)
                req.ik_request.robot_state.joint_state.position.append(float(self._joint_pos[jn]))
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = GRASP_ORIENTATION
        req.ik_request.pose_stamped = pose
        res = self._spin_wait(self._ik_client.call_async(req), timeout_sec=5.0)
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            self.node.get_logger().warn(f"IK that bai cho ({x:.3f},{y:.3f},{z:.3f}): {MOVEIT_ERRORS.get(code, code)}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return {jn: sol[jn] for jn in HOME_JOINTS if jn in sol}

    def _move_xyz(self, x, y, z) -> SkillStatus:
        joints = self._ik_joints(x, y, z)
        if joints:
            st = self._move_to_joint(joints, label=f"IK goal x={x:.3f} y={y:.3f} z={z:.3f}")
            if st == SkillStatus.SUCCESS:
                return st
        # du phong: dat muc tieu theo pose
        return self._move_to_pose(x, y, z, *GRASP_ORIENTATION)

    def pick(self, object_name: str) -> SkillStatus:
        # Ve home truoc khi quan sat de tay robot khong che camera
        st = self.home()
        if st != SkillStatus.SUCCESS:
            return st

        # Toa do cube lay truc tiep tu camera (khong dung TF tinh)
        try:
            detected = self.perception.detect_objects()
        except RuntimeError as e:
            self.node.get_logger().error(str(e))
            return SkillStatus.FAILED
        if object_name not in detected:
            self.node.get_logger().error(f"Camera khong nhin thay {object_name}")
            return SkillStatus.INVALID_OBJECT
        x, y = detected[object_name]

        self.open_gripper()
        time.sleep(0.5)
        self.node.get_logger().info(
            f"DEBUG ngon sau khi mo: left={self._finger_pos.get('left_finger_joint', -1):.4f} "
            f"right={self._finger_pos.get('right_finger_joint', -1):.4f}")
        before = self._cube_snapshot()

        st = self._move_xyz(x, y, PICK_Z + PRE_GRASP_DZ)
        self._log_links("tren cube (pre-grasp)")
        self._report_moved(before, "sau pre-grasp")
        if st != SkillStatus.SUCCESS:
            return st
        st = self._move_xyz(x, y, PICK_Z)
        self._log_links("o do cao kep (grasp)")
        self._report_moved(before, "sau khi ha xuong grasp")
        if st != SkillStatus.SUCCESS:
            return st

        self._log_cube(object_name, "truoc khi kep")
        self.close_gripper()
        if not self._verify_grasp():
            self.node.get_logger().warn(f"Gap hut {object_name}: khong co cube giua 2 ngon")
            self.open_gripper()
            self._move_xyz(x, y, PICK_Z + LIFT_DZ)
            return SkillStatus.FAILED

        st = self._move_xyz(x, y, PICK_Z + LIFT_DZ)
        time.sleep(0.5)
        self._log_cube(object_name, "sau khi nhac (z ~0.10 la cube dang len cung gripper, z ~0.02 la cube con tren ban)")
        if st == SkillStatus.SUCCESS:
            p = self._model_z.get(object_name)
            # Kiem tra bang trang thai mo phong (chi co trong Gazebo): cube phai len khoi mat ban.
            if p is not None and p[2] < CUBE_LIFTED_Z:
                self.node.get_logger().error(
                    f"Gap THAT BAI: {object_name} van nam tren ban (z={p[2]:.3f}), khong len cung gripper")
                return SkillStatus.FAILED
        return st

    def place(self, object_name: str, zone_name: str) -> SkillStatus:
        pos = self.landmarks.get(zone_name)
        if pos is None:
            return SkillStatus.INVALID_ZONE
        x, y = pos[0], pos[1]

        st = self._move_xyz(x, y, PLACE_Z + PRE_GRASP_DZ)
        if st != SkillStatus.SUCCESS:
            return st
        st = self._move_xyz(x, y, PLACE_Z)
        if st != SkillStatus.SUCCESS:
            return st

        self.open_gripper()
        time.sleep(0.5)

        return self._move_xyz(x, y, PLACE_Z + LIFT_DZ)
