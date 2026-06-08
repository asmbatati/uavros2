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


def _resolve_model_sdf(uav_name: str) -> tuple[str, str]:
    """Locate ``models/<uav>/model.sdf`` for the drone_markers node.

    Search order matches the install.sh copy target: PX4-Autopilot first
    (since that's what's actually loaded by Gazebo), then the package
    share dir (in-tree fallback).
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


def _setup(context, *_args, **_kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    uav = LaunchConfiguration("uav").perform(context)
    world = LaunchConfiguration("world").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context) in ("true", "True", "1")
    base = f"{ns}/base_link"
    odom = f"{ns}/odom"

    pkg_share = get_package_share_directory("uavros2")
    rviz_file = os.path.join(pkg_share, "rviz", "drone_view.rviz")

    # Map frame convention: the `map` origin is coincident with the drone's
    # spawn position in the Gazebo world. This means:
    #   - drone in map = MAVROS local_position (zeroed at spawn) → drone at
    #     (0, 0, 0) at spawn, no static-TF lift needed.
    #   - terrain in map = gazebo_xyz - spawn_xyz → terrain is centred near
    #     the origin so the default RViz orbit camera sees it.
    # The world_surface_publisher takes the spawn XYZ as `recenter_to` and
    # subtracts it from emitted points/marker poses.
    from uavros2.world_meta import spawn_pose as _spawn_pose
    sx, sy, sz, *_ = _spawn_pose(world) if world else (0.0,) * 6
    spawn_xyz = [float(sx), float(sy), float(sz)]

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
        # map ≡ spawn position. MAVROS local_position is zeroed at spawn, so
        # the drone shows at map(0,0,0) at takeoff. No Z lift here — the
        # terrain (republished by world_surface_publisher) gets shifted to
        # be relative to the spawn instead.
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

    # RViz body markers (resolves model.sdf for full-fidelity rendering).
    sdf_path, model_dir = _resolve_model_sdf(uav) if uav else ("", "")
    drone_markers = Node(
        package="uavros2", executable="drone_markers",
        name="drone_markers",
        namespace=ns,
        parameters=[{
            "use_sim_time": True,
            "frame_id": base,
            "marker_ns": ns,
            "model_sdf": sdf_path,
            "model_dir": model_dir,
        }],
        output="log",
    )

    # World terrain visualization: a Marker for COLLADA-based worlds
    # (urban1/2/3), or a coloured PointCloud2 for heightmap-based worlds
    # (urban4/5). The node reads its config from worlds/manifest.yaml's
    # `dem_viz:` block for the active world. Worlds without that block
    # (empty / warehouse) start the node in idle mode (no publishers).
    from uavros2.world_meta import dem_viz as _dem_viz
    viz_cfg = _dem_viz(world) if world else {}
    world_surface_params = {
        "use_sim_time": True,
        "frame_id": "map",
        "mode": viz_cfg.get("mode", ""),
        "mesh": viz_cfg.get("mesh", ""),
        "heightmap": viz_cfg.get("heightmap", ""),
        "texture":   viz_cfg.get("texture", ""),
        "pose":      list(viz_cfg.get("pose", [0.0, 0.0, 0.0])) + [0.0] * 3,
        "size":      list(viz_cfg.get("size", [100.0, 100.0, 10.0])),
        "decimation": int(viz_cfg.get("decimation", 4)),
        # Re-centre the published terrain on the spawn position so map(0,0,0)
        # coincides with where the drone takes off.
        "recenter_to": spawn_xyz,
    }
    world_surface = Node(
        package="uavros2", executable="world_surface_publisher",
        name="world_surface_publisher",
        parameters=[world_surface_params],
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

    actions = static_tfs + [odom2base, stitcher, traj_pub, drone_markers, world_surface]
    if use_rviz:
        actions.append(rviz_node)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("uav", default_value=""),
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("arm", default_value="none"),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Launch RViz with rviz/drone_view.rviz; set false "
                        "for headless / CI runs.",
        ),
        OpaqueFunction(function=_setup),
    ])
