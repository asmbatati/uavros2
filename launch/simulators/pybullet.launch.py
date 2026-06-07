#!/usr/bin/env python3
"""PyBullet backend for uavros2 — SCAFFOLDED.

Loads the UAV URDF and runs sim_control_bridge in placeholder-PID mode.
Sufficient for headless CI smoke tests; not for serious flight dynamics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _check_deps():
    try:
        import pybullet  # noqa: F401
    except ImportError as exc:
        raise RuntimeError(
            "pybullet is not installed. Install with:\n"
            "  pip install pybullet"
        ) from exc


def _setup(context, *_args, **_kwargs):
    _check_deps()

    uav = LaunchConfiguration("uav").perform(context)
    arm = LaunchConfiguration("arm").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)

    return [
        LogInfo(msg=f"[pybullet] Scaffolded backend — placeholder PID controller. uav={uav} arm={arm}"),
        Node(
            package="uavros2", executable="sim_control_bridge",
            name="sim_control_bridge",
            namespace=ns,
            parameters=[
                {"use_sim_time": False},
                {"simulator": "pybullet"},
                {"uav": uav},
                {"arm": arm},
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("uav", default_value="x500"),
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("arm", default_value="none"),
        OpaqueFunction(function=_setup),
    ])
