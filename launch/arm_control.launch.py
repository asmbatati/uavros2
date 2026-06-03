#!/usr/bin/env python3
"""Arm controller stack for floating-base manipulators.

Brings up:
- robot_state_publisher with the arm URDF (xacro-expanded). The
  gz_ros2_control Gazebo plugin reads this `robot_description` to
  discover joints, hardware interfaces, and controller config.
- joint_state_broadcaster and joint_trajectory_controller (spawned
  via the controller_manager that lives inside gz_ros2_control).
- Optionally MoveIt's move_group when use_moveit:=true and the arm
  ships a config/moveit/ directory.

Notes
-----
* No separate ros2_control_node is started — gz_ros2_control hosts the
  controller_manager itself. Spawners connect to it by name.
* The xacro file's sim_mode arg defaults to "gz" so the URDF declares
  the GazeboSimSystem hardware plugin.
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def _setup(context, *_args, **_kwargs):
    arm = LaunchConfiguration("arm").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)
    sim = LaunchConfiguration("simulator").perform(context)
    use_moveit = LaunchConfiguration("use_moveit").perform(context).lower() == "true"

    if arm == "none":
        return [LogInfo(msg="[arm_control] arm:=none — nothing to launch.")]

    pkg_share = get_package_share_directory("uav_gz_sim")
    arm_share = os.path.join(pkg_share, "arms", arm)
    controllers_yaml = os.path.join(arm_share, "config", "controllers.yaml")
    xacro_path = os.path.join(arm_share, "urdf", f"{arm}.urdf.xacro")

    if not os.path.isfile(controllers_yaml):
        return [LogInfo(msg=(
            f"[arm_control] controllers.yaml not found at {controllers_yaml}. "
            f"Arm {arm!r} may not yet ship a controller config."
        ))]

    if not os.path.isfile(xacro_path):
        return [LogInfo(msg=(
            f"[arm_control] urdf/{arm}.urdf.xacro not found at {xacro_path}. "
            f"Arm {arm!r} doesn't have an in-tree URDF — likely references "
            f"an upstream description package; compose the URDF separately."
        ))]

    # Expand xacro -> URDF, with sim_mode chosen per simulator.
    sim_mode = "gz" if sim in ("gazebo",) else "mock"
    robot_description = xacro.process_file(
        xacro_path, mappings={"sim_mode": sim_mode}
    ).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description,
        }],
        output="screen",
    )

    # Spawners must wait until the controller_manager (hosted by the
    # gz_ros2_control plugin) is up. A short TimerAction is the
    # simplest cross-version-compatible way to delay.
    spawn_jsb = TimerAction(period=4.0, actions=[
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
    ])
    spawn_jtc = TimerAction(period=5.5, actions=[
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            arguments=["joint_trajectory_controller", "--param-file", controllers_yaml],
            output="screen",
        ),
    ])

    actions = [
        LogInfo(msg=f"[arm_control] sim={sim} arm={arm} ns={ns} moveit={use_moveit}"),
        rsp,
        spawn_jsb,
        spawn_jtc,
    ]

    if use_moveit:
        moveit_dir = os.path.join(arm_share, "config", "moveit")
        srdf_path = os.path.join(moveit_dir, f"{arm}.srdf")
        kin_path = os.path.join(moveit_dir, "kinematics.yaml")
        ompl_path = os.path.join(moveit_dir, "ompl_planning.yaml")
        ctrl_path = os.path.join(moveit_dir, "moveit_controllers.yaml")
        lim_path = os.path.join(moveit_dir, "joint_limits.yaml")

        missing = [p for p in (srdf_path, kin_path, ompl_path, ctrl_path, lim_path)
                   if not os.path.isfile(p)]
        if missing:
            actions.append(LogInfo(msg=(
                f"[arm_control] use_moveit:=true but MoveIt config is incomplete; "
                f"missing: {missing}. Skipping move_group."
            )))
        else:
            srdf_xml = open(srdf_path).read()
            import yaml
            with open(kin_path) as f:
                kinematics = yaml.safe_load(f)
            with open(ompl_path) as f:
                ompl = yaml.safe_load(f)
            with open(ctrl_path) as f:
                moveit_controllers = yaml.safe_load(f)
            with open(lim_path) as f:
                joint_limits = yaml.safe_load(f)

            move_group_params = [
                {"use_sim_time": True},
                {"robot_description": robot_description},
                {"robot_description_semantic": srdf_xml},
                {"robot_description_kinematics": kinematics},
                {"robot_description_planning": joint_limits},
                {"planning_pipelines": ["ompl"]},
                {"default_planning_pipeline": "ompl"},
                {"ompl": ompl},
                moveit_controllers,
            ]

            actions.append(TimerAction(period=6.0, actions=[
                Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    namespace=ns,
                    parameters=move_group_params,
                    output="screen",
                ),
            ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="none"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("simulator", default_value="gazebo"),
        DeclareLaunchArgument("use_moveit", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
