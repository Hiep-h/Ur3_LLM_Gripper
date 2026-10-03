#!/usr/bin/env python3
import os
import yaml
import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from ur3_llm_control.perception import PerceptionInterface
from ur3_llm_control.llm_planner import LLMPlanner
from ur3_llm_control.task_validator import validate_plan
from ur3_llm_control.robot_skills import RobotSkills, SkillStatus
from ur3_llm_control.zone_manager import ZoneManager, expand_plan_with_conflict_resolution

def load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)

class SkillExecutorNode(Node):
    def __init__(self):
        super().__init__("skill_executor")

        share_dir = get_package_share_directory("ur3_llm_control")
        student_cfg = load_yaml(os.path.join(share_dir, "config", "student_config.yaml"))
        scene_cfg = load_yaml(os.path.join(share_dir, "config", "scene.yaml"))

        self.declare_parameter("llm_base_url", os.environ.get("LLM_BASE_URL", "http://localhost:20128/v1"))
        self.declare_parameter("llm_api_key", os.environ.get("LLM_API_KEY", ""))
        self.declare_parameter("llm_model", os.environ.get("LLM_MODEL", "hiep-combo"))

        base_url = self.get_parameter("llm_base_url").value
        api_key = self.get_parameter("llm_api_key").value
        model = self.get_parameter("llm_model").value

        self.planner = LLMPlanner(
            base_url=base_url,
            api_key=api_key,
            model=model,
            zone_mapping=student_cfg.get("zone_mapping", {}),
        )

        self.perception = PerceptionInterface(self)

        zones = scene_cfg.get("zones", {})
        parks = scene_cfg.get("park_positions", {})
        self.zone_manager = ZoneManager(zones, parks)

        self.skills = RobotSkills(
            self,
            perception=self.perception,
            landmarks={**zones, **parks},
            group_name="ur_manipulator",
            ee_link="tool0",
            base_frame=scene_cfg.get("frame_id", "base_link"),
        )

    def resolve_with_camera(self, steps: list[dict]) -> list[dict]:
        """Doc trang thai thuc tu camera, roi mo phong tung buoc qua ZoneManager
        de tu chen buoc don vat can sang diem do (park_*)."""
        self.skills.home()  # tay khong che camera
        detected = self.perception.detect_objects()
        print(f"CAMERA: {detected}")
        self.zone_manager.sync_from_camera(detected)
        return expand_plan_with_conflict_resolution(steps, self.zone_manager)

    def run_command(self, user_command: str):
        print(f"\nUSER COMMAND: {user_command}")

        try:
            plan = self.planner.get_plan(user_command)
        except Exception as e:
            print(f"LLM PLAN: ERROR ({e})")
            print("TASK FAILED")
            return

        print(f"LLM PLAN: {plan}")

        ok, reason = validate_plan(plan)
        if not ok:
            print(f"VALIDATION FAILED: {reason}")
            print("TASK FAILED")
            return

        try:
            resolved_steps = self.resolve_with_camera(plan["plan"])
        except RuntimeError as e:
            print(f"PERCEPTION CONFLICT FAILED: {e}")
            print("TASK FAILED")
            return

        if resolved_steps != plan["plan"]:
            print(f"RESOLVED PLAN (ZoneManager bo sung buoc don): {resolved_steps}")

        print("EXECUTION:")
        task_ok = True
        for step in resolved_steps:
            skill = step["skill"]

            if skill == "pick":
                status = self.skills.pick(step["object"])
                label = f"pick({step['object']})"
            elif skill == "place":
                status = self.skills.place(step["object"], step["zone"])
                label = f"place({step['object']}, {step['zone']})"
            else:
                status = self.skills.home()
                label = "home()"

            dots = "." * max(1, 28 - len(label))
            print(f"{label} {dots} {status.value}")

            if status != SkillStatus.SUCCESS:
                task_ok = False
                break

        if not task_ok:
            print("TASK FAILED")
            self.skills.home()
        else:
            print("TASK SUCCESS")

def main():
    rclpy.init()
    node = SkillExecutorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Luc khoi dong mot ngon co the bi truot lech (khop truot, gia toc luc dat tu the ban dau).
    # Mo gripper ngay de hai ngon tro lai doi xung truoc khi nhan lenh.
    if not node.skills.open_gripper():
        print("WARN: khong mo duoc gripper luc khoi dong (gripper_controller chua san sang?)")

    try:
        while rclpy.ok():
            cmd = input("\nNhap lenh (hoac 'exit'): ").strip()
            if cmd.lower() == "exit":
                break
            if cmd:
                node.run_command(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
