#!/usr/bin/env python3
"""RViz drone visualization for uavros2.

Launches:
- ``drone_markers`` (uavros2 node) under the drone namespace, rendering the
  full Gazebo model in RViz by parsing the UAV's ``model.sdf`` at startup
- (optional) rviz2 with ``rviz/drone_view.rviz`` pre-loaded

The marker publisher needs the model.sdf path; we resolve it from the
``uav:=`` argument by looking under PX4-Autopilot/Tools/simulation/gz/models/
(the install.sh copy target) and falling back to share/uavros2/models/.
"""

from __future__ import annotations

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _resolve_model_sdf(uav_name: str) -> tuple[str, str]:
    """Return (model_sdf_path, model_dir) for a UAV name.

    Search order:
      1. $PX4_DIR/Tools/simulation/gz/models/<uav>/model.sdf  (what runs in sim)
      2. $DEV_DIR/PX4-Autopilot/Tools/simulation/gz/models/<uav>/model.sdf
      3. <pkg_share>/models/<uav>/model.sdf                   (in-tree fallback)
    Returns ('', '') if not found; the marker node then falls back to its
    geometric quad rendering.
    """
    candidates = []
    px4_dir = os.environ.get("PX4_DIR")
    dev_dir = os.environ.get("DEV_DIR", os.path.expanduser("~/drone_arm_ws"))
    if px4_dir:
        candidates.append(os.path.join(px4_dir, "Tools/simulation/gz/models"))
    candidates.append(os.path.join(dev_dir, "PX4-Autopilot/Tools/simulation/gz/models"))
    candidates.append(os.path.join(
        get_package_share_directory("uavros2"), "models"))
    for base in candidates:
        sdf = os.path.join(base, uav_name, "model.sdf")
        if os.path.isfile(sdf):
            return sdf, base
    return "", ""


def _launch(context, *_args, **_kwargs):
    uav = LaunchConfiguration("uav").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    color_str = LaunchConfiguration("color").perform(context)
    rviz_enabled = LaunchConfiguration("use_rviz").perform(context) == "true"

    color = [float(c) for c in color_str.split(",")]
    if len(color) != 3:
        color = [0.1, 0.4, 1.0]

    model_sdf, model_dir = _resolve_model_sdf(uav)

    markers = Node(
        package="uavros2", executable="drone_markers",
        name="drone_markers",
        namespace=namespace,
        parameters=[{
            "frame_id": f"{namespace}/base_link",
            "marker_ns": namespace,
            "color": color,
            "model_sdf": model_sdf,
            "model_dir": model_dir,
        }],
        output="screen",
    )

    actions = [markers]

    if rviz_enabled:
        rviz_cfg = os.path.join(
            get_package_share_directory("uavros2"), "rviz", "drone_view.rviz")
        if os.path.isfile(rviz_cfg):
            actions.append(ExecuteProcess(
                cmd=["rviz2", "-d", rviz_cfg],
                output="screen",
            ))
        else:
            actions.append(ExecuteProcess(
                cmd=["rviz2"],
                output="screen",
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "uav", default_value="x500_stereo_cam_3d_lidar",
            description="UAV model name (matches models/<uav>/).",
        ),
        DeclareLaunchArgument(
            "namespace", default_value="drone",
            description="ROS namespace (markers publish under /<ns>/markers).",
        ),
        DeclareLaunchArgument(
            "color", default_value="0.1,0.4,1.0",
            description="Geometric fallback color as 'r,g,b' (0..1).",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Launch rviz2 with rviz/drone_view.rviz pre-loaded.",
        ),
        OpaqueFunction(function=_launch),
    ])
