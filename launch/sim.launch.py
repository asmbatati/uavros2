#!/usr/bin/env python3
"""Top-level dispatcher for uavros2.

Reads the ``simulator:=`` argument and includes the matching
``launch/simulators/<simulator>.launch.py`` together with the sim-agnostic
``sim_common.launch.py``. Optionally chains ``arm_control.launch.py`` if
``arm`` is not ``none``.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare


SUPPORTED_SIMS = {"gazebo", "webots", "mujoco", "isaac", "pybullet", "genesis"}
SUPPORTED_ARMS = {"none", "three_dof", "openmanip_x", "panda", "ur5"}


def _dispatch(context, *_args, **_kwargs):
    sim = LaunchConfiguration("simulator").perform(context)
    arm = LaunchConfiguration("arm").perform(context)
    uav = LaunchConfiguration("uav").perform(context)
    world = LaunchConfiguration("world").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    use_moveit = LaunchConfiguration("use_moveit").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)

    if sim not in SUPPORTED_SIMS:
        raise RuntimeError(
            f"simulator:={sim!r} is not supported. "
            f"Choose from: {sorted(SUPPORTED_SIMS)}"
        )
    if arm not in SUPPORTED_ARMS:
        raise RuntimeError(
            f"arm:={arm!r} is not supported. "
            f"Choose from: {sorted(SUPPORTED_ARMS)}"
        )

    common_args = {
        "uav": uav,
        "world": world,
        "arm": arm,
        "namespace": namespace,
        "use_rviz": use_rviz,
    }

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("uavros2"),
                "launch", "simulators", f"{sim}.launch.py",
            ])
        ]),
        launch_arguments=common_args.items(),
    )

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("uavros2"),
                "launch", "sim_common.launch.py",
            ])
        ]),
        launch_arguments=common_args.items(),
    )

    actions = [sim_launch, common_launch]

    if arm != "none":
        arm_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("uavros2"),
                    "launch", "arm_control.launch.py",
                ])
            ]),
            launch_arguments={
                "arm": arm,
                "namespace": namespace,
                "simulator": sim,
                "use_moveit": use_moveit,
            }.items(),
        )
        actions.append(arm_launch)

    # NOTE: drone_markers + RViz are now part of sim_common.launch.py (gated
    # on its own `use_rviz` arg, threaded above via common_args). No separate
    # visualization include here — that avoids the double-RViz overlap.
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "simulator", default_value="gazebo",
            description=f"Physics backend. One of: {sorted(SUPPORTED_SIMS)}",
        ),
        DeclareLaunchArgument(
            "uav", default_value="x500_stereo_cam_3d_lidar",
            description="UAV model name (must exist under models/ for the chosen simulator)",
        ),
        DeclareLaunchArgument(
            "arm", default_value="none",
            description=f"Manipulator. One of: {sorted(SUPPORTED_ARMS)}",
        ),
        DeclareLaunchArgument(
            "world", default_value="warehouse",
            description="World name (must exist in worlds/manifest.yaml for the chosen simulator)",
        ),
        DeclareLaunchArgument(
            "namespace", default_value="drone",
            description="ROS namespace for canonical topics",
        ),
        DeclareLaunchArgument(
            "use_moveit", default_value="false",
            description="Launch MoveIt's move_group alongside the arm "
                        "(requires arms/<arm>/config/moveit/ to exist).",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Launch RViz with drone_view.rviz + drone_markers "
                        "(full-fidelity Gazebo-model rendering with spinning "
                        "rotors). Set false for headless / CI runs.",
        ),
        OpaqueFunction(function=_dispatch),
    ])
