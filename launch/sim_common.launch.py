#!/usr/bin/env python3
"""Sim-agnostic nodes for uavros2.

Runs the parts of the ROS-side stack that don't depend on which simulator
is producing the canonical topics. Included by sim.launch.py after the
per-simulator launch file has been included.

Consumes canonical topics (per ``config/topics/canonical_topics.yaml``):
  /<ns>/mavros/local_position/pose (state)
  /<ns>/front_stereo/*/image_raw   (cameras)
"""

import math
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def _setup(context, *_args, **_kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    base = f"{ns}/base_link"
    odom = f"{ns}/odom"

    pkg_share = get_package_share_directory("uavros2")
    rviz_file = os.path.join(pkg_share, "rviz", "rviz_config.rviz")

    # TF tree: static transforms that are sim-agnostic. The new-style
    # named args (--x, --frame-id, ...) silence the "Old-style arguments
    # are deprecated" warning from tf2_ros static_transform_publisher.
    def _stp(name, x, y, z, yaw, pitch, roll, parent, child):
        return Node(
            package="tf2_ros", name=name,
            executable="static_transform_publisher",
            arguments=[
                "--x", str(x), "--y", str(y), "--z", str(z),
                "--yaw", str(yaw), "--pitch", str(pitch), "--roll", str(roll),
                "--frame-id", parent, "--child-frame-id", child,
            ],
            parameters=[{"use_sim_time": True}], output="log",
        )

    static_tfs = [
        _stp("map2global_tf_node", 0, 0, 0, 0, 0, 0, "global", "map"),
        _stp("map2map_frd_tf_node", 0, 0, 0, 1.5708, 0, 1.5708, "map", "map_frd"),
        _stp(f"map2px4_{ns}_tf_node", 0, 0, 0, 0, 0, 0, "map", odom),
        _stp("front_lidar_tf_node",
             0.0, 0.0, -0.12,
             math.radians(0), math.radians(90), math.radians(0),
             base, "front_lidar_link"),
    ]

    # Dynamic odom -> base_link via tf_relay (MAVROS pose -> TF).
    odom2base = Node(
        package="uavros2", executable="tf_relay",
        name="odom2base_tf_relay",
        parameters=[
            {"use_sim_time": True},
            {"source_topic": f"/{ns}/mavros/local_position/pose"},
            {"target_frame_id": odom},
            {"child_frame_id": base},
            {"queue_size": 50},
            {"publish_rate": 50.0},
        ],
        output="log",
    )

    # Camera fusion (auto-detects available cameras under /<ns>/).
    stitcher = Node(
        package="uavros2", executable="adaptive_image_stitcher",
        name="adaptive_image_stitcher",
        parameters=[
            {"use_sim_time": True},
            {"namespace_filter": f"/{ns}/"},
            {"output_topic": f"/{ns}/camera/stitched_image"},
            {"verbose": False},
            {"discovery_timeout": 10.0},
            {"stitch_rate": 10.0},
        ],
        output="log",
    )

    # Ground-truth path publisher for RViz.
    traj_pub = Node(
        package="uavros2", executable="trajectory_publisher",
        name="trajectory_publisher",
        parameters=[
            {"use_sim_time": True},
            {"pose_topic": f"/{ns}/mavros/local_position/pose"},
            {"path_topic": f"/{ns}/gt_path"},
            {"max_path_length": 5000},
            {"verbose": False},
        ],
        output="log",
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="log", arguments=["-d", rviz_file],
        parameters=[
            {"use_sim_time": True},
            {"tf_buffer_cache_time_ms": 60000},
            {"default_display_update_rate": 10.0},
            {"transform_tolerance": 5.0},
            {"message_filter_queue_size": 200},
            {"synchronize_time": True},
        ],
    )

    return static_tfs + [odom2base, stitcher, traj_pub, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("uav", default_value=""),
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("arm", default_value="none"),
        OpaqueFunction(function=_setup),
    ])
