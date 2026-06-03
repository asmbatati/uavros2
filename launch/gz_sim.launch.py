import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory


def _find_workspace_px4_dir():
    """Locate PX4-Autopilot as a sibling of the workspace containing ros2_ws.

    Always derived from the installed package share path — the ``$PX4_DIR``
    env var is intentionally ignored. This guarantees the launch picks up
    the PX4 sibling of the active ``ros2_ws``, not a stray copy elsewhere.

    Walks up from ``share/uav_gz_sim`` looking for a directory that contains
    both ``ros2_ws`` and ``PX4-Autopilot`` siblings. Returns the PX4 path
    or None if not found.
    """
    pkg_share = get_package_share_directory('uav_gz_sim')
    # pkg_share is .../<DEV_DIR>/ros2_ws/install/uav_gz_sim/share/uav_gz_sim
    cur = pkg_share
    for _ in range(8):
        cur = os.path.dirname(cur)
        if not cur or cur == '/':
            break
        candidate = os.path.join(cur, 'PX4-Autopilot')
        if (os.path.isdir(candidate)
                and os.path.isdir(os.path.join(cur, 'ros2_ws'))):
            return candidate
    return None


def generate_launch_description():
    PX4_DIR = _find_workspace_px4_dir()

    if not PX4_DIR:
        # Final fallback: DEV_DIR env or default workspace path.
        dev_dir = os.getenv('DEV_DIR',
                            os.path.join(os.path.expanduser('~'),
                                         'drone_arm_ws'))
        PX4_DIR = os.path.join(dev_dir, 'PX4-Autopilot')

    print(f'PX4_DIR resolved to: {PX4_DIR}')
    if not os.path.isdir(PX4_DIR):
        print(f'ERROR: PX4_DIR={PX4_DIR} does not exist. '
              f'Expected a PX4-Autopilot directory next to ros2_ws.')
        sys.exit(1)
    if not os.path.isfile(os.path.join(PX4_DIR, 'build/px4_sitl_default/bin/px4')):
        print(f'ERROR: PX4 binary not found at {PX4_DIR}/build/px4_sitl_default/bin/px4. '
              f'Run `cd {PX4_DIR} && make px4_sitl` first.')
        sys.exit(1)

    namespace = LaunchConfiguration('gz_ns')
    namespace_launch_arg = DeclareLaunchArgument(
        'gz_ns',
        default_value=''
    )

    headless = LaunchConfiguration('headless')
    headless_launch_arg = DeclareLaunchArgument(
        'headless',
        default_value='0'
    )

    gz_world = LaunchConfiguration('gz_world')
    gz_world_launch_arg = DeclareLaunchArgument(
        'gz_world',
        default_value='default'
    )

    gz_model_name = LaunchConfiguration('gz_model_name')
    gz_model_name_launch_arg = DeclareLaunchArgument(
        'gz_model_name',
        default_value='x500'
    )

    px4_autostart_id = LaunchConfiguration('px4_autostart_id')
    px4_autostart_id_launch_arg = DeclareLaunchArgument(
        'px4_autostart_id',
        default_value='4001'
    )

    instance_id = LaunchConfiguration('instance_id')
    instance_id_launch_arg = DeclareLaunchArgument(
        'instance_id',
        default_value='0'
    )

    xpos = LaunchConfiguration('xpos')
    xpos_launch_arg = DeclareLaunchArgument(
        'xpos',
        default_value='0.0'
    )

    ypos = LaunchConfiguration('ypos')
    ypos_launch_arg = DeclareLaunchArgument(
        'ypos',
        default_value='0.0'
    )

    zpos = LaunchConfiguration('zpos')
    zpos_launch_arg = DeclareLaunchArgument(
        'zpos',
        default_value='0.2'
    )

    # Update PX4 launch command with more verbose output and pre-flight checks
    px4_sim_process = ExecuteProcess(
        cmd=[[
            'echo "Starting PX4 with the following parameters:" && ',
            'echo "PX4_DIR: ', PX4_DIR, '" && ',
            'echo "Model: ', gz_model_name, '" && ',
            'echo "Autostart ID: ', px4_autostart_id, '" && ',
            'cd ', PX4_DIR, ' && ',
            'if [ ! -f ./build/px4_sitl_default/bin/px4 ]; then echo "ERROR: PX4 binary not found. Did you build PX4?"; exit 1; fi && ',
            'if [ ! -d ./ROMFS/px4fmu_common/init.d-posix/airframes ]; then echo "ERROR: PX4 airframes directory not found"; exit 1; fi && ',
            'echo "Checking for specific airframe file..." && ',
            'if [ ! -f ./ROMFS/px4fmu_common/init.d-posix/airframes/', px4_autostart_id, '_gz_', gz_model_name, ' ]; then ',
            'echo "ERROR: PX4 specific airframe file not found for model ', gz_model_name, ' with ID ', px4_autostart_id, '"; ',
            'echo "Looking for: ./ROMFS/px4fmu_common/init.d-posix/airframes/', px4_autostart_id, '_gz_', gz_model_name, '"; ',
            'ls -la ./ROMFS/px4fmu_common/init.d-posix/airframes/ | grep ', px4_autostart_id, '; ',
            'exit 1; ',
            'else echo "Airframe file found."; fi && ',
            'echo "Running PX4 with simulation parameters..." && ',
            # gz_ros2_control plugin lives in /opt/ros/jazzy/lib and is
            # not on Gazebo's default plugin search path. Prepend it so
            # the composed arm models can load `libgz_ros2_control-system.so`.
            'GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}" ',
            'PX4_SIM_SPEED_FACTOR=1 ',
            'PX4_SYS_AUTOSTART=', px4_autostart_id,
            ' PX4_GZ_MODEL=', gz_model_name,
            ' PX4_UXRCE_DDS_NS=', namespace,
            " PX4_GZ_MODEL_POSE='", xpos, ',', ypos, ',', zpos, "'",
            ' PX4_GZ_WORLD=', gz_world,
            ' ./build/px4_sitl_default/bin/px4 -i ', instance_id,
            ' -d'  # Add debug flag
        ]],
        shell=True
    )

    ld = LaunchDescription()

    ld.add_action(headless_launch_arg)
    ld.add_action(gz_world_launch_arg)
    ld.add_action(gz_model_name_launch_arg)
    ld.add_action(px4_autostart_id_launch_arg)
    ld.add_action(instance_id_launch_arg)
    ld.add_action(xpos_launch_arg)
    ld.add_action(ypos_launch_arg)
    ld.add_action(zpos_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(px4_sim_process)

    return ld