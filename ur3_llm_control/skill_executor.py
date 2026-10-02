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

        self.skills = RobotSkills(
            self,
            group_name="ur_manipulator",
            ee_link="tool0",
            base_frame=scene_cfg.get("frame_id", "base_link"),
            approach_offset_z=scene_cfg.get("approach_offset_z", 0.15),
        )

        self.perception = PerceptionInterface(self)

    def resolve_with_camera(self, steps: list[dict]) -> list[dict]:
        resolved = []
        i = 0
        while i < len(steps):
            step = steps[i]
            if step["skill"] == "pick" and i + 1 < len(steps) and steps[i+1]["skill"] == "place":
                obj = step["object"]
                place_step = steps[i + 1]
                target_zone = place_step["zone"]

                occupant = self.perception.check_zone(target_zone)
                if occupant is not None and occupant != obj:
                    free_zone = self.perception.find_free_position(exclude_zones={target_zone})
                    if free_zone is None:
                        raise RuntimeError(f"Camera: khong tim thay vi tri trong de don '{occupant}'")
                    print(f"CAMERA: check_zone({target_zone}) -> phat hien '{occupant}' dang chiem cho")
                    resolved.append({"skill": "pick", "object": occupant})
                    resolved.append({"skill": "place", "object": occupant, "zone": free_zone})

                resolved.append(step)
                resolved.append(place_step)
                i += 2
            else:
                resolved.append(step)
                i += 1
        return resolved

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
            print(f"RESOLVED PLAN (camera bo sung buoc don): {resolved_steps}")

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
