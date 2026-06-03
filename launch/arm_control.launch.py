#!/usr/bin/env python3
"""Arm controller stack for floating-base manipulators.

Reads ``arm:=<name>`` and spawns ros2_control_node with the per-arm
``controllers.yaml``, plus joint_state_broadcaster and
joint_trajectory_controller. Optionally launches MoveIt move_group when
``use_moveit:=true`` and a MoveIt config exists for the arm.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
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

    if not os.path.isfile(controllers_yaml):
        return [LogInfo(msg=(
            f"[arm_control] controllers.yaml not found at {controllers_yaml}. "
            f"Arm {arm!r} may not yet ship a controller config."
        ))]

    actions = [
        LogInfo(msg=f"[arm_control] sim={sim} arm={arm} ns={ns} moveit={use_moveit}"),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=ns,
            parameters=[{"use_sim_time": True}, controllers_yaml],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            arguments=["joint_trajectory_controller"],
            output="screen",
        ),
    ]

    if use_moveit:
        moveit_share = os.path.join(arm_share, "config", "moveit")
        if not os.path.isdir(moveit_share):
            actions.append(LogInfo(msg=(
                f"[arm_control] use_moveit:=true but no MoveIt config at {moveit_share}. "
                "Skipping move_group."
            )))
        else:
            actions.append(Node(
                package="moveit_ros_move_group",
                executable="move_group",
                namespace=ns,
                parameters=[{"use_sim_time": True}],
                output="screen",
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="none"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("simulator", default_value="gazebo"),
        DeclareLaunchArgument("use_moveit", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
