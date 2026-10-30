import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from uavros2.px4_paths import find_px4_dir


def generate_launch_description():
    PX4_DIR = find_px4_dir()

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

    def _px4_actions(context):
        """PX4 SITL, plus our own Gazebo server when running headless.

        With a GUI, PX4's rc script starts both the Gazebo server and client.
        Headless (no X display, CI, a shared machine), we start the server
        ourselves with ``--headless-rendering`` so camera / GPU-lidar sensors
        render through EGL, and PX4 attaches to it (PX4_GZ_STANDALONE=1 makes
        gz_bridge wait for the world instead of starting its own server).
        """
        is_headless = headless.perform(context).lower() in ('1', 'true')
        world = gz_world.perform(context)
        models = os.path.join(PX4_DIR, 'Tools/simulation/gz/models')
        worlds = os.path.join(PX4_DIR, 'Tools/simulation/gz/worlds')
        # Point Gazebo at THIS PX4 tree's models/worlds. PX4 builds the world
        # path as $PX4_GZ_WORLDS/<world>.sdf and the model spawn uses
        # GZ_SIM_RESOURCE_PATH; PX4's generated gz_env.sh only appends and may
        # carry a stale absolute path from a build done elsewhere, so set the
        # correct in-tree paths explicitly here.
        resource_path = ':'.join(
            p for p in (os.environ.get('GZ_SIM_RESOURCE_PATH', ''), models, worlds) if p)
        # gz_ros2_control plugin lives in /opt/ros/jazzy/lib and is not on
        # Gazebo's default plugin search path. Prepend it so the composed arm
        # models can load `libgz_ros2_control-system.so`.
        plugin_path = ':'.join(
            p for p in ('/opt/ros/jazzy/lib', os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
            if p)
        sim_env = {
            'PX4_GZ_MODELS': models,
            'PX4_GZ_WORLDS': worlds,
            'GZ_SIM_RESOURCE_PATH': resource_path,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': plugin_path,
        }

        actions = []
        headless_env = ''
        if is_headless:
            actions.append(ExecuteProcess(
                cmd=['gz', 'sim', '-r', '-s', '--headless-rendering',
                     os.path.join(worlds, f'{world}.sdf')],
                additional_env=sim_env,
                output='screen',
            ))
            headless_env = 'HEADLESS=1 PX4_GZ_STANDALONE=1 '

        actions.append(ExecuteProcess(
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
                # PX4 SITL resolves its startup script (etc/init.d-posix/rcS) and
                # working files relative to the CWD, so run it from the build dir.
                'cd ./build/px4_sitl_default && ',
                # exec: replace the shell with px4 so the launch's SIGINT on
                # shutdown reaches PX4 itself instead of orphaning it.
                'exec env ',
                headless_env,
                'PX4_SIM_SPEED_FACTOR=1 ',
                'PX4_SYS_AUTOSTART=', px4_autostart_id,
                ' PX4_GZ_MODEL=', gz_model_name,
                ' PX4_UXRCE_DDS_NS=', namespace,
                " PX4_GZ_MODEL_POSE='", xpos, ',', ypos, ',', zpos, "'",
                ' PX4_GZ_WORLD=', gz_world,
                ' ./bin/px4 -i ', instance_id,
                ' -d'  # Add debug flag
            ]],
            additional_env=sim_env,
            shell=True
        ))
        return actions

    px4_sim_process = OpaqueFunction(function=_px4_actions)

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