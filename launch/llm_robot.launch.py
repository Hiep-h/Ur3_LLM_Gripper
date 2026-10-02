import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

PKG = "ur3_llm_control"

def load_file(file_path):
    try:
        with open(file_path, "r") as f:
            return f.read()
    except Exception:
        return None

def load_yaml(file_path):
    try:
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    except Exception:
        return {}

def _box_sdf(name, size_xyz, rgba, static=True, physical=True):
    sx, sy, sz = size_xyz
    r, g, b, a = rgba
    collision_xml = f"""
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface>
          <friction>
            <ode>
              <mu>100.0</mu>
              <mu2>100.0</mu2>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>100.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>""" if physical else ""
    gravity = "true" if physical else "false"
    inertial_xml = f"""
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.000013</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.000013</iyy><iyz>0</iyz><izz>0.000013</izz>
        </inertia>
      </inertial>""" if physical else ""

    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <static>{'true' if static else 'false'}</static>
    <link name="link">
      <gravity>{gravity}</gravity>
      {inertial_xml}
      <velocity_decay>
        <linear>0.1</linear>
        <angular>0.8</angular>
      </velocity_decay>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
      </visual>{collision_xml}
    </link>
  </model>
</sdf>"""

def _spawn_box(name, x, y, z, size_xyz, rgba, static=True, physical=True):
    path = f"/tmp/{name}.sdf"
    with open(path, "w") as f:
        f.write(_box_sdf(name, size_xyz, rgba, static, physical))
    return Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", name, "-file", path, "-x", str(x), "-y", str(y), "-z", str(z)],
        output="screen",
    )

def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    ur_share = get_package_share_directory("ur_moveit_config")

    xacro_path = os.path.join(pkg_share, "urdf", "ur3_with_gripper.urdf.xacro")
    world_path = os.path.join(pkg_share, "worlds", "ur_scene.world")

    robot_description = {"robot_description": Command(["xacro ", xacro_path])}

    # Generate SRDF tu template xacro
    srdf_xacro = os.path.join(ur_share, "srdf", "ur.srdf.xacro")
    robot_description_semantic = {
        "robot_description_semantic": Command(["xacro ", srdf_xacro, " name:=ur"])
    }

    kinematics_yaml = load_yaml(os.path.join(ur_share, "config", "kinematics.yaml"))
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    ompl_planning_yaml = load_yaml(os.path.join(ur_share, "config", "ompl_planning.yaml"))
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_controllers = load_yaml(os.path.join(pkg_share, "config", "moveit_controllers.yaml"))
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_path,
             "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", "ur3", "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            {"use_sim_time": True},
        ],
    )

    load_arm_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_trajectory_controller"],
        output="screen",
    )
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )
    load_gripper_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "gripper_controller"],
        output="screen",
    )

    spawn_scene = [
        _spawn_box("work_table", 0.35, 0.0, -0.025, (1.0, 0.8, 0.05), (0.55, 0.35, 0.2, 1.0)),
        _spawn_box("red_cube",    0.28,  0.15, 0.025, (0.04, 0.04, 0.04), (1.0, 0.0, 0.0, 1.0), static=False, physical=True),
        _spawn_box("yellow_cube", 0.28,  0.00, 0.025, (0.04, 0.04, 0.04), (1.0, 1.0, 0.0, 1.0), static=False, physical=True),
        _spawn_box("blue_cube",   0.28, -0.15, 0.025, (0.04, 0.04, 0.04), (0.0, 0.0, 1.0, 1.0), static=False, physical=True),
        _spawn_box("zone_a", 0.40,  0.15, 0.005, (0.08, 0.08, 0.005), (1.0, 0.6, 0.6, 0.7)),
        _spawn_box("zone_b", 0.40,  0.00, 0.005, (0.08, 0.08, 0.005), (1.0, 1.0, 0.6, 0.7)),
        _spawn_box("zone_c", 0.40, -0.15, 0.005, (0.08, 0.08, 0.005), (0.6, 0.6, 1.0, 0.7)),
        _spawn_box("temp_zone", 0.34, 0.30, 0.005, (0.08, 0.08, 0.005), (0.7, 0.7, 0.7, 0.6)),
    ]

    camera_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["0.34", "0", "1.0", "0", "1.5708", "0", "base_link", "overhead_cam_optical_frame"],
        output="screen",
    )

    scene_node = Node(
        package=PKG, executable="scene_publisher", name="scene_publisher",
        output="screen", parameters=[{"use_sim_time": True}],
    )

    detector_node = Node(
        package=PKG, executable="object_detector", name="object_detector",
        output="screen", parameters=[{"use_sim_time": True}],
    )

    executor_node = Node(
        package=PKG, executable="skill_executor", name="skill_executor",
        output="screen", parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        gazebo,
        rsp_node,
        camera_tf,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=[move_group]),
        TimerAction(period=10.0, actions=[load_joint_state_broadcaster, load_arm_controller]),
        TimerAction(period=11.0, actions=[load_gripper_controller]),
        TimerAction(period=12.0, actions=spawn_scene),
        TimerAction(period=15.0, actions=[scene_node, detector_node]),
        TimerAction(period=18.0, actions=[executor_node]),
    ])
