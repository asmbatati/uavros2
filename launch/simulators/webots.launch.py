#!/usr/bin/env python3
"""Webots backend for uav_gz_sim.

Current scope: a working Webots demo that opens the simulator and
spawns a quadrotor, by wrapping the upstream `webots_ros2_crazyflie`
launch as a placeholder for the x500. A real x500 PROTO + matching
.wbt world is follow-up work; the goal here is that
`simulator:=webots` actually starts Webots end-to-end today.

Notes
-----
* PX4 SITL no longer has a Webots target (Tools/simulation/webots
  was removed upstream). The control story is therefore
  sim_control_bridge - same pattern as the mujoco backend - rather
  than PX4 + MAVROS.
* The Crazyflie placeholder publishes its own ROS 2 topics via
  webots_ros2_driver; sensor_relay can be configured later to
  remap them onto the canonical contract once we author a real
  x500 PROTO.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, LogInfo,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def _check_deps():
    """Verify webots_ros2 packages are installed; raise with install hint."""
    missing = []
    for pkg in ("webots_ros2_driver", "webots_ros2_crazyflie"):
        try:
            get_package_share_directory(pkg)
        except Exception:
            missing.append(pkg)
    if missing:
        raise RuntimeError(
            f"Missing Webots ROS 2 packages: {missing}\n"
            "Install with:\n"
            "  sudo apt install ros-jazzy-webots-ros2\n"
            "and ensure Webots R2024a or newer is on PATH (`which webots`)."
        )


def _setup(context, *_args, **_kwargs):
    _check_deps()

    uav = LaunchConfiguration("uav").perform(context)
    arm = LaunchConfiguration("arm").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)

    if arm != "none":
        return [LogInfo(msg=(
            f"[webots] arm:={arm!r} is not yet supported in the Webots backend. "
            "Only the Crazyflie placeholder ships in this pass. "
            "See docs/SIMULATORS.md for the roadmap."
        ))]

    # Wrap the upstream Crazyflie demo launch. Its world arg is
    # 'crazyflie_apartment.wbt'; it owns the WebotsLauncher,
    # ros2_supervisor, and the driver Node.
    cf_share = get_package_share_directory("webots_ros2_crazyflie")
    upstream_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cf_share, "launch", "robot_launch.py")
        ),
        launch_arguments={"world": "crazyflie_apartment.wbt"}.items(),
    )

    # sim_control_bridge in inert mode (until we wire a real x500 PROTO).
    # Today it just announces itself; the Crazyflie driver publishes its
    # own sensor topics under the global namespace.
    bridge = Node(
        package="uav_gz_sim", executable="sim_control_bridge",
        name="sim_control_bridge",
        namespace=ns,
        parameters=[
            {"use_sim_time": False},
            {"simulator": "webots"},
            {"uav": uav},
            {"arm": arm},
        ],
        output="screen",
    )

    return [
        LogInfo(msg=(
            "[webots] starting Webots with the upstream Crazyflie demo "
            "(placeholder for x500 PROTO). See docs/SIMULATORS.md."
        )),
        upstream_launch,
        bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("uav", default_value="x500"),
        DeclareLaunchArgument("world", default_value="crazyflie_apartment.wbt"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("arm", default_value="none"),
        OpaqueFunction(function=_setup),
    ])
