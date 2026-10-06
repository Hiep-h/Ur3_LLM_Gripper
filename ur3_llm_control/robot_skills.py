#!/usr/bin/env python3
import math
import os
import time
import rclpy
import rclpy.time
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint, JointConstraint,
                             CollisionObject, PlanningScene)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import tf2_ros
from ur3_llm_control import ur3_kin
try:
    from linkattacher_msgs.srv import AttachLink, DetachLink  # IFRA_LinkAttacher (tuy chon)
except ImportError:
    AttachLink = DetachLink = None
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
FINGER_CLOSE = 0.003  # dich dong CHUA dong han (cube chan o ~0.005/ngon): luc ep = p*(q-0.003) ~ vai N, khong ep qua tay
# Co cube giua 2 ngon: tong q hai ngon ~ 0.010. Khong co cube: ngon dong het, tong ~ 0.
GRASP_MIN_SUM = 0.0085  # ngon rong khep toi dich 0.003*2=0.006 nen phai cao hon; co cube ~0.0099-0.0104
CUBE_LIFTED_Z = 0.06  # cube cao hon muc nay sau khi nhac thi coi la dang duoc giu (tam cube luc nam ban ~0.02)
LIFT_DZ = 0.08
CUBE_OBSTACLE_SIZE = 0.08  # hop vat can quanh moi cube khac (cube 4cm + do hu)

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
        self._scene_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._obstacle_ids = []
        node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        # Vi tri that cua cube trong Gazebo: CHI de kiem tra ket qua pick/place, khong dung de lap ke hoach.
        self._model_z = {}
        if ModelStates is not None:
            node.create_subscription(ModelStates, "/gazebo/model_states", self._on_model_states, 5)

        self._move_client = ActionClient(node, MoveGroup, "/move_action")
        self._gripper_client = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self._use_attach = os.environ.get("UR3_USE_ATTACH", "1") != "0" and AttachLink is not None
        if os.environ.get("UR3_USE_ATTACH", "1") != "0" and AttachLink is None:
            node.get_logger().warn("Khong import duoc linkattacher_msgs: chay KHONG co attach (chi ma sat)")
        if self._use_attach:
            self._attach_client = node.create_client(AttachLink, "/ATTACHLINK")
            self._detach_client = node.create_client(DetachLink, "/DETACHLINK")
            node.get_logger().info("Attach: BAT (cube gan vao gripper bang fixed joint sau khi kep xac nhan)")
        self._attached = None
        self._arm_client = ActionClient(node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")

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

        log = self.node.get_logger()
        future = self._gripper_client.send_goal_async(goal)
        handle = self._spin_wait(future)
        if not handle or not handle.accepted:
            log.warn(f"gripper goal {opening:.3f}: KHONG duoc nhan (handle={handle})")
            return False
        res = self._spin_wait(handle.get_result_async(), timeout_sec=10.0)
        if res is None:
            log.warn(f"gripper goal {opening:.3f}: het thoi gian cho ket qua")
            return False
        ok = res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return ok

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
        return GRASP_MIN_SUM <= total < 0.07

    def _move_to_joint(self, joint_dict: dict, label: str = "joint goal (home)", tol: float = 0.01) -> SkillStatus:
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
            jc.tolerance_above = tol
            jc.tolerance_below = tol
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
        """IK cho pose (x,y,z) huong GRASP. Uu tien IK rieng (ur3_kin, Newton tu seed khuyu-len, ket qua xac
        dinh va gan seed); neu khong hoi tu thi dung MoveIt /compute_ik (KDL, de nhay nhanh khuyu/co tay)."""
        names = list(HOME_JOINTS)
        cur = [self._joint_pos.get(jn) for jn in names]
        have_cur = all(v is not None for v in cur)
        # Dang o tu the khuyu-len (elbow>0.3, lift<-0.5) thi dung chinh no lam seed; con lai (home...) dung PICK_SEED
        if have_cur and cur[2] > 0.3 and cur[1] < -0.5:
            seed = list(cur)
        else:
            seed = list(ur3_kin.PICK_SEED)
        q, ok = ur3_kin.ik([float(x), float(y), float(z)], ur3_kin.quat_to_R(*GRASP_ORIENTATION), seed)
        if ok and q[2] > 0.3 and all(abs(v) < 6.0 for v in q):
            out = {}
            for i, jn in enumerate(names):
                v = q[i]
                if have_cur:
                    d = v - cur[i]
                    v = cur[i] + math.atan2(math.sin(d), math.cos(d))
                out[jn] = v
            return out
        self.node.get_logger().warn(f"IK rieng khong hoi tu cho ({x:.3f},{y:.3f},{z:.3f}), dung MoveIt IK")
        return self._ik_joints_moveit(x, y, z)

    def _ik_joints_moveit(self, x, y, z):
        """Giai IK (MoveIt /compute_ik) khoi dau tu tu the khop HIEN TAI."""
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
        out = {}
        for jn in HOME_JOINTS:
            if jn not in sol:
                continue
            v = sol[jn]
            cur = self._joint_pos.get(jn)
            if cur is not None:
                # KDL co the tra goc tuong duong lech k*2pi (khop UR3 co gioi han +-2pi): chon goc GAN
                # nhat voi tu the hien tai, neu khong OMPL/noi suy se quay khop day ca vong (~270-360 do).
                d = v - cur
                v = cur + math.atan2(math.sin(d), math.cos(d))
            out[jn] = v
        return out

    def _set_obstacles(self, detected: dict, exclude: str | None):
        """Dat cac cube KHAC (vi tri tu camera) lam vat can trong MoveIt planning scene de duong di
        tranh chung. Cube dang gap (exclude) khong dua vao de gripper tiep can duoc."""
        if not self._scene_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn("apply_planning_scene khong san sang, bo qua vat can")
            return
        scene = PlanningScene()
        scene.is_diff = True
        if os.environ.get("UR3_NO_OBSTACLES"):
            detected = {}  # A/B test: bo vat can, chi xoa obstacle cu
        for oid in self._obstacle_ids:
            co = CollisionObject()
            co.id = oid
            co.header.frame_id = self.base_frame
            co.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(co)
        self._obstacle_ids = []
        for name, (cx, cy) in detected.items():
            if name == exclude:
                continue
            co = CollisionObject()
            co.id = f"obs_{name}"
            co.header.frame_id = self.base_frame
            co.operation = CollisionObject.ADD
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [CUBE_OBSTACLE_SIZE, CUBE_OBSTACLE_SIZE, 0.07]
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = float(cx), float(cy), 0.035
            pose.orientation.w = 1.0
            co.primitives = [prim]
            co.primitive_poses = [pose]
            scene.world.collision_objects.append(co)
            self._obstacle_ids.append(co.id)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        res = self._spin_wait(self._scene_client.call_async(req), timeout_sec=5.0)
        ok = bool(res and res.success)

    def _tool_xyz(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, rclpy.time.Time())
            p = t.transform.translation
            return p.x, p.y, p.z
        except Exception:
            return None

    def _move_xyz(self, x, y, z, precise: bool = False, recover: bool = True, straight: bool = False) -> SkillStatus:
        """Di chuyen tool0 toi (x, y, z). precise=True: do lai vi tri that va bu them neu lech > 4mm
        (dung khi kep/tha vi lech 1cm la du de chi 1 ngon cham cube)."""
        tx, ty, tz = x, y, z
        st = SkillStatus.FAILED
        recovered = False
        for attempt in range(3 if precise else 1):
            if straight:
                # Ha/nhac thang (khong dung OMPL: duong ngau nhien tung quet lech cube 3-4cm)
                st = self._move_vertical(tx, ty, tz, duration=4.0 if attempt == 0 else 1.5)
                if st != SkillStatus.SUCCESS or not precise:
                    return st
                time.sleep(0.4)
                cur = self._tool_xyz()
                if cur is None:
                    return st
                ex, ey, ez = x - cur[0], y - cur[1], z - cur[2]
                err = (ex * ex + ey * ey + ez * ez) ** 0.5
                if err < 0.004:
                    return st
                tx, ty, tz = tx + ex, ty + ey, tz + ez
                continue
            joints = self._ik_joints(tx, ty, tz)
            st = SkillStatus.FAILED
            if joints:
                st = self._move_to_joint(joints, label=f"IK goal x={tx:.3f} y={ty:.3f} z={tz:.3f}",
                                         tol=0.002 if precise else 0.01)
            if st != SkillStatus.SUCCESS:
                # du phong: dat muc tieu theo pose
                st = self._move_to_pose(tx, ty, tz, *GRASP_ORIENTATION)
            if st != SkillStatus.SUCCESS and recover and not recovered:
                # Chuyen dong hong giua chung (vi du CONTROL_FAILED): ve home roi thu lai mot lan
                self.node.get_logger().warn("Chuyen dong that bai, ve home roi thu lai mot lan")
                recovered = True
                if self.home() == SkillStatus.SUCCESS:
                    st = self._move_to_pose(tx, ty, tz, *GRASP_ORIENTATION)
                    joints = self._ik_joints(tx, ty, tz)
                    if st != SkillStatus.SUCCESS and joints:
                        st = self._move_to_joint(joints, label=f"IK goal (thu lai) x={tx:.3f} y={ty:.3f} z={tz:.3f}")
            if st != SkillStatus.SUCCESS or not precise:
                return st
            time.sleep(0.4)
            cur = self._tool_xyz()
            if cur is None:
                return st
            ex, ey, ez = x - cur[0], y - cur[1], z - cur[2]
            err = (ex * ex + ey * ey + ez * ez) ** 0.5
            if err < 0.004:
                return st
            if err > 0.02:
                self.node.get_logger().error("Sai so vi tri qua lon (>2cm): khong bu, coi la that bai")
                return SkillStatus.FAILED
            tx, ty, tz = tx + ex, ty + ey, tz + ez  # bu sai so
        return st

    def _move_vertical(self, x, y, z, duration=4.0) -> SkillStatus:
        """Di chuyen THANG (gan nhu theo duong thang) toi (x,y,z): giai IK tu tu the hien tai roi gui thang
        cho joint_trajectory_controller (noi suy tuyen tinh trong khong gian khop, chuyen doi ~vai cm nen
        gan nhu thang dung). Khac voi MoveIt/OMPL: OMPL co the quet tay vong lon (da thay: ~270 do quanh
        goc, keo ngon truot khoi cube). Neu that bai thi dung _move_xyz binh thuong."""
        log = self.node.get_logger()
        joints = self._ik_joints(x, y, z)
        if joints:
            delta = max(abs(joints[jn] - self._joint_pos.get(jn, joints[jn])) for jn in joints)
            if delta <= 0.6 and self._arm_client.wait_for_server(timeout_sec=5.0):
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = list(joints.keys())
                point = JointTrajectoryPoint()
                point.positions = [float(joints[jn]) for jn in joints]
                point.time_from_start.sec = int(duration)
                point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
                goal.trajectory.points = [point]
                handle = self._spin_wait(self._arm_client.send_goal_async(goal))
                if handle and handle.accepted:
                    res = self._spin_wait(handle.get_result_async(), timeout_sec=duration + 10.0)
                    if res is not None and res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                        time.sleep(0.3)
                        return SkillStatus.SUCCESS
                    code = None if res is None else res.result.error_code
                    log.warn(f"di chuyen thang that bai (error_code={code}), dung MoveIt")
                else:
                    log.warn("di chuyen thang: goal khong duoc nhan, dung MoveIt")
            else:
                log.warn("di chuyen thang: IK lech qua lon hoac controller khong san sang, dung MoveIt; "
                         "hien tai -> IK: " + " ".join(f"{jn.split('_joint')[0]}={self._joint_pos.get(jn, float('nan')):+.2f}->{joints[jn]:+.2f}" for jn in joints))
        return self._move_xyz(x, y, z, recover=False)

    def _link_call(self, client, srv_cls, model1, link1, model2, link2, what) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn(f"{what}: dich vu khong san sang (plugin chua nap?)")
            return False
        req = srv_cls.Request()
        req.model1_name, req.link1_name = model1, link1
        req.model2_name, req.link2_name = model2, link2
        res = self._spin_wait(client.call_async(req), timeout_sec=5.0)
        ok = bool(res is not None and res.success)
        msg = "" if res is None else res.message
        return ok

    def _attach(self, object_name: str) -> bool:
        """Gan cube vao gripper (CHI khi UR3_USE_ATTACH). Thu cac ten link co the sau khi gazebo gop fixed joint."""
        if not self._use_attach:
            return False
        for link in ("wrist_3_link", "gripper_base_link", "tool0"):
            if self._link_call(self._attach_client, AttachLink, "ur3", link, object_name, "link", "ATTACH"):
                self._attached = (link, object_name)
                return True
        return False

    def _detach(self) -> None:
        if not self._attached:
            return
        link, obj = self._attached
        self._link_call(self._detach_client, DetachLink, "ur3", link, obj, "link", "DETACH")
        self._attached = None

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
        self._set_obstacles(detected, exclude=object_name)

        self.open_gripper()
        time.sleep(0.5)

        st = self._move_xyz(x, y, PICK_Z + PRE_GRASP_DZ)
        if st != SkillStatus.SUCCESS:
            return st
        st = self._move_xyz(x, y, PICK_Z, precise=True, recover=False, straight=True)
        if st != SkillStatus.SUCCESS:
            return st

        self.close_gripper()
        if sum(self._finger_pos.values()) > 0.06:
            # Ngon khong khep (lenh dong bi bo qua): thu gui lai mot lan
            self.node.get_logger().warn("ngon khong khep sau lenh dong, gui lai lenh dong")
            self.close_gripper()
        if not self._verify_grasp():
            self.node.get_logger().warn(f"Gap hut {object_name}: khong co cube giua 2 ngon")
            self.open_gripper()
            self._move_xyz(x, y, PICK_Z + LIFT_DZ)
            return SkillStatus.FAILED

        self._attach(object_name)  # chi co tac dung khi UR3_USE_ATTACH=1
        # Dang cam cube giua 2 ngon: bo vat can de duong nhac thang len khong bi chan nham
        self._set_obstacles({}, exclude=None)
        st = self._move_vertical(x, y, PICK_Z + LIFT_DZ, duration=4.0 if self._attached else 8.0)
        time.sleep(0.5)
        if st == SkillStatus.SUCCESS:
            p = self._model_z.get(object_name)
            # Kiem tra bang trang thai mo phong (chi co trong Gazebo): cube phai len khoi mat ban.
            if p is not None and p[2] < CUBE_LIFTED_Z:
                self.node.get_logger().error(
                    f"Gap THAT BAI: {object_name} van nam tren ban (z={p[2]:.3f}), khong len cung gripper")
                self._detach()
                return SkillStatus.FAILED
        if st != SkillStatus.SUCCESS:
            self._detach()
        return st

    def place(self, object_name: str, zone_name: str) -> SkillStatus:
        pos = self.landmarks.get(zone_name)
        if pos is None:
            return SkillStatus.INVALID_ZONE
        x, y = pos[0], pos[1]

        st = self._move_xyz(x, y, PLACE_Z + PRE_GRASP_DZ, recover=False)
        if st != SkillStatus.SUCCESS:
            return st
        st = self._move_xyz(x, y, PLACE_Z, precise=True, recover=False, straight=True)
        if st != SkillStatus.SUCCESS:
            return st

        self._detach()  # chi co tac dung khi UR3_USE_ATTACH=1
        self.open_gripper()
        time.sleep(0.5)

        st = self._move_vertical(x, y, PLACE_Z + LIFT_DZ)
        time.sleep(1.0)
        # Kiem tra bang trang thai mo phong (chi co trong Gazebo): cube phai nam tren vung dich
        p = self._model_z.get(object_name)
        if p is not None:
            dist = ((p[0] - x) ** 2 + (p[1] - y) ** 2) ** 0.5
            self.node.get_logger().info(
                f"Tha {object_name} -> {zone_name}: that o ({p[0]:.3f}, {p[1]:.3f}), lech {dist * 100:.1f} cm")
            if dist > 0.04 or p[2] > 0.06:
                self.node.get_logger().error(f"Tha THAT BAI: {object_name} khong nam tren {zone_name}")
                return SkillStatus.FAILED
        return st
