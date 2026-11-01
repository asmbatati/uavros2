#!/usr/bin/env python3
"""Gazebo (Harmonic) backend for uavros2.

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
import shutil
import tempfile

from ament_index_python import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


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
        tempfile.gettempdir(), f"uavros2_live_models_{os.getuid()}"
    )
    src_models_dir = os.path.join(pkg_share, "models")
    if not os.path.isdir(src_models_dir):
        return live_dir

    substitutions = {
        "@UAVROS2_PKG_SHARE@": pkg_share,
        "@UAVROS2_NAMESPACE@": namespace,
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
    "x500_d435": "4420",
    "x3_uav": "4421",
    "x500_mono_cam_3d_lidar": "4422",
    "x500_stereo_cam_3d_lidar": "4423",
    "x500_twin_stereo_twin_velodyne": "4424",
    "x500_with_three_dof_arm": "4425",
    "x500_with_openmanip_x": "4426",
    "x500_with_openmanip_x_tool": "4429",
    "x500_with_panda": "4427",
    "x500_with_ur5": "4428",
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

    tool_ft = f"/world/{world}/model/{m}/joint/j_tool/sensor/tool_ft/forcetorque"
    tool_truth = f"/model/{m}/pose"
    if uav.endswith("_tool"):
        args.append(f"{tool_ft}@geometry_msgs/msg/Wrench[ignition.msgs.Wrench")
        args.append(f"{tool_truth}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V")

    # Remappings to canonical names
    args += ["--ros-args"]
    if uav.endswith("_tool"):
        args += ["-r", f"{tool_ft}:=/{ns}/tool/wrench", "-r", f"{tool_truth}:=/{ns}/tool/truth"]
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
    instance = int(LaunchConfiguration("instance").perform(context))
    headless = LaunchConfiguration("headless").perform(context)

    # Materialize substituted-marker SDFs into a temp dir and prepend it
    # to GZ_SIM_RESOURCE_PATH so PX4/Gazebo finds these instead of the
    # raw in-tree ones (whose <parameters> still contains @MARKER@).
    pkg_share_early = get_package_share_directory("uavros2")
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

    # Per-world spawn pose from worlds/manifest.yaml. DEM-based outdoor
    # worlds (urban1-5) have elevated terrain; default Z=0.1 would spawn
    # the drone underground. The manifest holds the right Z for each.
    from uavros2.world_meta import spawn_pose as _spawn_pose
    sx, sy, sz, *_rpy = _spawn_pose(world)
    xpos, ypos, zpos = f"{sx:g}", f"{sy:g}", f"{sz:g}"

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("uavros2"), "launch", "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "gz_ns": ns,
            "headless": headless,
            "gz_model_name": uav,
            "gz_world": world,
            "px4_autostart_id": airframe,
            "instance_id": f"{instance}",
            "xpos": xpos, "ypos": ypos, "zpos": zpos,
            "verbose": "true",
            "use_sim_time": "true",
        }.items(),
    )

    pkg_share = get_package_share_directory("uavros2")
    from uavros2.mavros_layout import mavros_namespace
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("uavros2"), "launch", "mavros.launch.py",
            ])
        ]),
        launch_arguments={
            # /<ns>/mavros/<topic> for both the fork and upstream MAVROS
            # (see uavros2/mavros_layout.py).
            "mavros_namespace": mavros_namespace(ns),
            # PX4 instance i: MAV_SYS_ID = i+1, offboard link on UDP
            # 14540+i (remote) <-> 14580+i (PX4 local), per px4-rc.mavlink.
            "tgt_system": f"{instance + 1}",
            "fcu_url": f"udp://:{14540 + instance}@127.0.0.1:{14580 + instance}",
            "pluginlists_yaml": os.path.join(pkg_share, "config", "mavros", "drone_px4_pluginlists.yaml"),
            "config_yaml": os.path.join(pkg_share, "config", "mavros", "drone_px4_config.yaml"),
            "base_link_frame": f"{ns}/base_link",
            "odom_frame": f"{ns}/odom",
            "map_frame": "map",
            "use_sim_time": "true",
        }.items(),
    )

    # Pre-flight: kill stale Gazebo / PX4 SITL processes left over from a
    # prior aborted launch of THIS sim. Without this, PX4 detects the orphan
    # server via gz_bridge ("gazebo already running") and refuses to spawn.
    # Only processes in our GZ_PARTITION, our PX4 tree and our PX4 instance
    # are touched (see uavros2/sim_cleanup.py), so other simulations on the
    # same machine survive. It runs synchronously here, before any of the
    # actions below exist, so it can never race with the new PX4.
    from uavros2.px4_paths import find_px4_dir
    from uavros2.sim_cleanup import kill_stale
    killed = kill_stale(find_px4_dir(), instance)

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        name="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=_build_ros_gz_bridge_args(uav, world, ns, instance),
        parameters=[{"use_sim_time": True}, {"verbose": False}],
        output="log",
    )

    # PX4's gz_bridge launches both the Gazebo server AND a GUI client
    # (headless:=1 starts a server only, see gz_sim.launch.py). We used to
    # spawn `gz sim -g` ourselves, but that resulted in a duplicate window.

    # Gazebo-specific TF: connect Gazebo's sensor link to our canonical front_lidar_link.
    # Only emit when the UAV actually has a 3D LiDAR.
    # set_resource_path must come first so child processes inherit it.
    actions = [set_resource_path, gz_launch, mavros_launch, ros_gz_bridge]
    if killed:
        actions.insert(0, LogInfo(msg=f"[gazebo] killed stale sim processes: {killed}"))
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
        DeclareLaunchArgument(
            "instance", default_value="0",
            description="PX4 SITL instance: sets MAV_SYS_ID, MAVLink ports "
                        "and the Gazebo model name <uav>_<instance>"),
        DeclareLaunchArgument(
            "headless", default_value="0",
            description="1 = no Gazebo GUI; the server renders sensors "
                        "through EGL (--headless-rendering)"),
        SetEnvironmentVariable(name="ROS_PARAM_use_sim_time", value="true"),
        OpaqueFunction(function=_setup),
    ])
