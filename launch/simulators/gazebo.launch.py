#!/usr/bin/env python3
"""Gazebo (Harmonic) backend for uav_gz_sim.

Brings up:
- PX4 SITL + Gazebo (via launch/gz_sim.launch.py)
- MAVROS (via launch/mavros.launch.py)
- ros_gz_bridge configured to publish canonical topic names

Owned topics (per the canonical contract):
  /<ns>/imu, /<ns>/gps, /<ns>/air_pressure,
  /<ns>/front_stereo/{left,right}/{image_raw,camera_info},
  /<ns>/front_lidar/points,
  /<ns>/mavros/local_position/{pose,odom} (via MAVROS)
"""

import os
import math
import shutil
import tempfile
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction,
    SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory


def _materialize_live_models(pkg_share: str, namespace: str) -> str:
    """Write a "live" copy of every composed arm model with markers substituted.

    The composed arm SDFs reference controllers.yaml + namespace via markers
    because gz_ros2_control's <parameters> tag accepts only literal paths
    (no $(find ...), no package:// URI) and its <ros><namespace> doesn't
    expand env vars or any substitution either. Returns the directory to
    prepend to GZ_SIM_RESOURCE_PATH so Gazebo finds the substituted copies
    instead of the in-tree ones.
    """
    live_dir = os.path.join(
        tempfile.gettempdir(), f"uav_gz_sim_live_models_{os.getuid()}"
    )
    src_models_dir = os.path.join(pkg_share, "models")
    if not os.path.isdir(src_models_dir):
        return live_dir

    substitutions = {
        "@UAV_GZ_SIM_PKG_SHARE@": pkg_share,
        "@UAV_GZ_SIM_NAMESPACE@": namespace,
    }

    for model in os.listdir(src_models_dir):
        src = os.path.join(src_models_dir, model)
        if not os.path.isdir(src):
            continue
        sdf_path = os.path.join(src, "model.sdf")
        if not os.path.isfile(sdf_path):
            continue
        # Only materialize models that actually use a marker — cheap check
        # so we don't bloat the temp dir with copies of stock UAVs.
        with open(sdf_path) as f:
            text = f.read()
        if not any(marker in text for marker in substitutions):
            continue

        dst = os.path.join(live_dir, model)
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(src, dst)
        for marker, value in substitutions.items():
            text = text.replace(marker, value)
        with open(os.path.join(dst, "model.sdf"), "w") as f:
            f.write(text)

    return live_dir


# Map UAV model name -> default PX4 airframe ID.
# When you add a new UAV, register it here so the dispatcher knows which
# PX4 airframe to autostart.
UAV_AIRFRAME = {
    "x500": "4001",
    "x500_d435": "4020",
    "x3_uav": "4021",
    "x500_mono_cam_3d_lidar": "4022",
    "x500_stereo_cam_3d_lidar": "4023",
    "x500_twin_stereo_twin_velodyne": "4024",
    "x500_with_three_dof_arm": "4025",
    "x500_with_openmanip_x": "4026",
    "x500_with_panda": "4027",
    "x500_with_ur5": "4028",
}


def _build_ros_gz_bridge_args(uav: str, world: str, ns: str, instance: int = 0):
    """Build the ros_gz_bridge argument list for the given UAV model.

    The topic-string structure is Gazebo-specific
    (``/world/<world>/model/<uav>_<id>/link/.../sensor/.../...``) and is
    remapped to canonical names per ``config/topics/canonical_topics.yaml``.
    """
    m = f"{uav}_{instance}"
    args = [
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        f"/world/{world}/model/{m}/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        f"/world/{world}/model/{m}/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[ignition.msgs.FluidPressure",
        f"/world/{world}/model/{m}/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
    ]

    if "stereo" in uav or "twin_stereo" in uav:
        args += [
            f"/world/{world}/model/{m}/link/left_camera_link/sensor/left_camera_sensor/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            f"/world/{world}/model/{m}/link/right_camera_link/sensor/right_camera_sensor/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            f"/world/{world}/model/{m}/link/left_camera_link/sensor/left_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            f"/world/{world}/model/{m}/link/right_camera_link/sensor/right_camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ]

    if "3d_lidar" in uav or "velodyne" in uav:
        args.append(
            f"/world/{world}/model/{m}/link/lidar3d_link/sensor/velodyne_16/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked"
        )

    # Remappings to canonical names
    args += ["--ros-args"]
    args += [
        "-r", f"/world/{world}/model/{m}/link/base_link/sensor/imu_sensor/imu:=/{ns}/imu",
        "-r", f"/world/{world}/model/{m}/link/base_link/sensor/air_pressure_sensor/air_pressure:=/{ns}/air_pressure",
        "-r", f"/world/{world}/model/{m}/link/base_link/sensor/navsat_sensor/navsat:=/{ns}/gps",
    ]
    if "stereo" in uav or "twin_stereo" in uav:
        args += [
            "-r", f"/world/{world}/model/{m}/link/left_camera_link/sensor/left_camera_sensor/image:=/{ns}/front_stereo/left_cam/image_raw",
            "-r", f"/world/{world}/model/{m}/link/right_camera_link/sensor/right_camera_sensor/image:=/{ns}/front_stereo/right_cam/image_raw",
            "-r", f"/world/{world}/model/{m}/link/left_camera_link/sensor/left_camera_sensor/camera_info:=/{ns}/front_stereo/left_cam/camera_info",
            "-r", f"/world/{world}/model/{m}/link/right_camera_link/sensor/right_camera_sensor/camera_info:=/{ns}/front_stereo/right_cam/camera_info",
        ]
    if "3d_lidar" in uav or "velodyne" in uav:
        args += [
            "-r", f"/world/{world}/model/{m}/link/lidar3d_link/sensor/velodyne_16/scan/points:=/{ns}/front_lidar/points",
        ]
    return args


def _setup(context, *_args, **_kwargs):
    uav = LaunchConfiguration("uav").perform(context)
    world = LaunchConfiguration("world").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)

    # Materialize substituted-marker SDFs into a temp dir and prepend it
    # to GZ_SIM_RESOURCE_PATH so PX4/Gazebo finds these instead of the
    # raw in-tree ones (whose <parameters> still contains @MARKER@).
    pkg_share_early = get_package_share_directory("uav_gz_sim")
    live_models_dir = _materialize_live_models(pkg_share_early, ns)
    existing_resource = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path_value = (
        f"{live_models_dir}:{existing_resource}" if existing_resource else live_models_dir
    )
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=resource_path_value
    )

    airframe = UAV_AIRFRAME.get(uav)
    if airframe is None:
        raise RuntimeError(
            f"No PX4 airframe registered for uav={uav!r}. "
            f"Add an entry to UAV_AIRFRAME in {__file__}."
        )

    xpos, ypos, zpos = "0.0", "0.0", "0.1"
    instance = 0

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("uav_gz_sim"), "launch", "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "gz_ns": ns,
            "headless": "0",
            "gz_model_name": uav,
            "gz_world": world,
            "px4_autostart_id": airframe,
            "instance_id": f"{instance}",
            "xpos": xpos, "ypos": ypos, "zpos": zpos,
            "verbose": "true",
            "use_sim_time": "true",
        }.items(),
    )

    pkg_share = get_package_share_directory("uav_gz_sim")
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("uav_gz_sim"), "launch", "mavros.launch.py",
            ])
        ]),
        launch_arguments={
            "mavros_namespace": f"{ns}/mavros",
            "tgt_system": "1",
            "fcu_url": "udp://:14540@127.0.0.1:14557",
            "pluginlists_yaml": os.path.join(pkg_share, "config", "mavros", "drone_px4_pluginlists.yaml"),
            "config_yaml": os.path.join(pkg_share, "config", "mavros", "drone_px4_config.yaml"),
            "base_link_frame": f"{ns}/base_link",
            "odom_frame": f"{ns}/odom",
            "map_frame": "map",
            "use_sim_time": "true",
        }.items(),
    )

    # Pre-flight: kill any stale Gazebo / PX4 SITL processes left over from
    # a prior aborted launch. Without this, PX4 detects the orphan server
    # via gz_bridge ("gazebo already running") and refuses to spawn.
    #
    # IMPORTANT: the Gazebo server runs with comm=ruby (it's wrapped in a
    # Ruby launcher), so `pkill -x gz` does NOT match it. Match the full
    # argv with a regex anchored at the start of the command line - that
    # way the pkill command itself (which starts with "pkill", not "gz"
    # or "...px4") cannot self-match.
    kill_stale = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "pkill -9 -f '^gz sim' 2>/dev/null || true; "
            "pkill -9 -f '^[^ ]*px4_sitl_default/bin/px4' 2>/dev/null || true; "
            "sleep 0.5",
        ],
        output="log",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        name="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=_build_ros_gz_bridge_args(uav, world, ns, instance),
        parameters=[{"use_sim_time": True}, {"verbose": False}],
        output="log",
    )

    # PX4's gz_bridge launches both the Gazebo server AND a GUI client by
    # default when the pre-flight kill has cleared any prior orphan server.
    # We used to spawn `gz sim -g` ourselves, but that resulted in a
    # duplicate window. If you ever need to force a GUI client (e.g. when
    # PX4 detects an existing headless server and won't spawn one), bring
    # back the TimerAction below.

    # Sequence: kill_stale must FINISH before PX4/MAVROS/bridge start.
    # ROS 2 launch runs LaunchDescription actions in parallel by default,
    # so just appending in order is not enough — without the event handler,
    # `pkill -9 -x px4` would race with and SIGKILL the brand-new PX4.
    after_kill = RegisterEventHandler(
        OnProcessExit(
            target_action=kill_stale,
            on_exit=[gz_launch, mavros_launch, ros_gz_bridge],
        )
    )

    # Gazebo-specific TF: connect Gazebo's sensor link to our canonical front_lidar_link.
    # Only emit when the UAV actually has a 3D LiDAR.
    # set_resource_path must come first so child processes inherit it.
    actions = [set_resource_path, kill_stale, after_kill]
    if "3d_lidar" in uav or "velodyne" in uav:
        gazebo_lidar_link = f"{uav}_{instance}/lidar3d_link/velodyne_16"
        actions.append(Node(
            package="tf2_ros",
            name="front_lidar2gazebo_tf_node",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "front_lidar_link",
                "--child-frame-id", gazebo_lidar_link,
            ],
            parameters=[{"use_sim_time": True}],
            output="log",
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("uav", default_value="x500_stereo_cam_3d_lidar"),
        DeclareLaunchArgument("world", default_value="warehouse"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("arm", default_value="none"),
        SetEnvironmentVariable(name="ROS_PARAM_use_sim_time", value="true"),
        OpaqueFunction(function=_setup),
    ])
