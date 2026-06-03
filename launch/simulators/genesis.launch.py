#!/usr/bin/env python3
"""Genesis backend for uav_gz_sim — STUB.

The Genesis API is still moving; revisit when stable.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown


def _setup(_context, *_args, **_kwargs):
    return [
        LogInfo(msg=(
            "[genesis] Genesis backend is not yet implemented.\n"
            "          See docs/SIMULATORS.md for the roadmap and current status."
        )),
        Shutdown(reason="Genesis backend not yet implemented"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("uav", default_value="x500"),
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("arm", default_value="none"),
        OpaqueFunction(function=_setup),
    ])
