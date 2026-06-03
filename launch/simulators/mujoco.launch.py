#!/usr/bin/env python3
"""MuJoCo backend for uav_gz_sim.

Working scope: x500 + Panda. No PX4 — sim_control_bridge owns rotor mixing
and synthesizes MAVROS-shaped state topics so downstream code is identical.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _check_deps():
    try:
        import mujoco  # noqa: F401
    except ImportError as exc:
        raise RuntimeError(
            "mujoco is not installed. Install with:\n"
            "  pip install mujoco\n"
            "and see docs/SIMULATORS.md for mujoco_ros2_control setup."
        ) from exc


def _setup(context, *_args, **_kwargs):
    _check_deps()

    uav = LaunchConfiguration("uav").perform(context)
    arm = LaunchConfiguration("arm").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)

    bridge = Node(
        package="uav_gz_sim", executable="sim_control_bridge",
        name="sim_control_bridge",
        namespace=ns,
        parameters=[
            {"use_sim_time": True},
            {"simulator": "mujoco"},
            {"uav": uav},
            {"arm": arm},
        ],
        output="screen",
    )

    return [
        LogInfo(msg=f"[mujoco] Starting sim_control_bridge for uav={uav} arm={arm}"),
        bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("uav", default_value="x500"),
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("arm", default_value="none"),
        OpaqueFunction(function=_setup),
    ])
