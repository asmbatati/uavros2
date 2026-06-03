#!/usr/bin/env python3
"""Webots backend for uav_gz_sim.

Working scope: x500 base UAV, no arm. PX4 SITL via ``make px4_sitl webots``.
Arm integration with webots_ros2_control is deferred.

Owned topics: canonical sensor + state topics via webots_ros2_driver.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _check_deps():
    """Verify webots_ros2_driver is available; raise with install hint if not."""
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("webots_ros2_driver")
    except Exception as exc:
        raise RuntimeError(
            "webots_ros2_driver is not installed. Install with:\n"
            "  sudo apt install ros-jazzy-webots-ros2\n"
            "and ensure Webots R2024a or newer is on PATH.\n"
            f"Original error: {exc}"
        ) from exc


def _setup(context, *_args, **_kwargs):
    _check_deps()

    uav = LaunchConfiguration("uav").perform(context)
    arm = LaunchConfiguration("arm").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)

    if arm != "none":
        return [LogInfo(msg=(
            f"[webots] arm:={arm!r} is not yet supported in the Webots backend. "
            "Only x500 base ships in this pass. See docs/SIMULATORS.md."
        ))]

    if uav != "x500":
        return [LogInfo(msg=(
            f"[webots] uav:={uav!r} is not yet supported in the Webots backend. "
            "Only 'x500' ships in this pass."
        ))]

    # Real wiring: webots_ros2_driver node spawning the x500 PROTO with sensors
    # mapped to canonical topics. PX4 SITL must be launched separately:
    #   cd $DEV_DIR/PX4-Autopilot && make px4_sitl webots
    # The driver below assumes PX4 is already running and listening on UDP.
    driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        name="webots_driver",
        namespace=ns,
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return [
        LogInfo(msg="[webots] Start PX4 SITL in another terminal: cd $DEV_DIR/PX4-Autopilot && make px4_sitl webots"),
        driver,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("uav", default_value="x500"),
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("arm", default_value="none"),
        OpaqueFunction(function=_setup),
    ])
