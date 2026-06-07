#!/usr/bin/env python3
"""MuJoCo backend for uavros2.

Working scope: x500 base UAV (cylinder-chain arms can be composed
into the MJCF later). No PX4 - sim_control_bridge owns rotor mixing
and synthesizes MAVROS-shaped state topics so downstream code is
identical to the Gazebo path.

Two processes are started:
1. `python3 -m mujoco.viewer --mjcf=...` for an interactive GUI.
2. `sim_control_bridge` (with the mujoco adapter) for ROS topic
   integration. The bridge runs its own physics instance, so the
   viewer's poses and the published poses are not synchronized -
   the viewer is purely for visual sanity-checking. Synchronizing
   them requires running the viewer in-process via
   mujoco.viewer.launch_passive, which is a follow-up.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, LogInfo, ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def _check_deps():
    try:
        import mujoco  # noqa: F401
    except ImportError as exc:
        raise RuntimeError(
            "mujoco is not installed. pip install mujoco\n"
            "See docs/SIMULATORS.md for the recommended versions."
        ) from exc


def _setup(context, *_args, **_kwargs):
    _check_deps()

    uav = LaunchConfiguration("uav").perform(context)
    arm = LaunchConfiguration("arm").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)

    pkg_share = get_package_share_directory("uavros2")

    # Asset lookup: prefer composed UAV+arm MJCF, fall back to bare UAV.
    candidate_paths = []
    if arm and arm != "none":
        candidate_paths.append(
            os.path.join(pkg_share, "models", f"x500_with_{arm}",
                         "mjcf", f"x500_with_{arm}.xml")
        )
    candidate_paths.append(
        os.path.join(pkg_share, "models", uav, "mjcf", f"{uav}.xml")
    )

    mjcf_path = next((p for p in candidate_paths if os.path.isfile(p)), None)
    if mjcf_path is None:
        return [LogInfo(msg=(
            f"[mujoco] No MJCF found for uav={uav!r}, arm={arm!r}. "
            f"Tried: {candidate_paths}.\n"
            f"        Generate one via scripts/convert/urdf_to_mjcf.py, "
            f"or hand-author models/<name>/mjcf/<name>.xml."
        ))]

    viewer = ExecuteProcess(
        cmd=["python3", "-m", "mujoco.viewer", f"--mjcf={mjcf_path}"],
        output="screen",
    )

    bridge = Node(
        package="uavros2", executable="sim_control_bridge",
        name="sim_control_bridge",
        namespace=ns,
        parameters=[
            {"use_sim_time": False},
            {"simulator": "mujoco"},
            {"uav": uav},
            {"arm": arm},
        ],
        output="screen",
    )

    return [
        LogInfo(msg=f"[mujoco] viewer mjcf={mjcf_path}"),
        LogInfo(msg=f"[mujoco] bridge uav={uav} arm={arm} ns={ns}"),
        viewer,
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
