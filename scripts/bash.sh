# Source from ~/.bashrc to enable uavros2 aliases + environment.
# install.sh appends `source $DEV_DIR/bash.sh` (copied here) to ~/.bashrc.

: "${DEV_DIR:=$HOME/drone_arm_ws}"

################# Editor / shell #################
alias gd='gedit ~/.bashrc'
alias gs="gedit $DEV_DIR/bash.sh"
alias src='source ~/.bashrc'
alias zenoh='ros2 run rmw_zenoh_cpp rmw_zenohd'
alias sss='source install/setup.bash'
alias qgc="cd $DEV_DIR && ./QGroundControl.AppImage"

################# PX4 SITL (low-level) #################
alias px4="cd $DEV_DIR/PX4-Autopilot && make px4_sitl gz_x500_twin_stereo_twin_velodyne"
alias px4_tug="cd $DEV_DIR/PX4-Autopilot && PX4_GZ_MODEL_POSE='0,0,0.1,0,0,0' make px4_sitl gz_x500_stereo_cam_3d_lidar PX4_GZ_WORLD=tugbot_depot"

################# Build #################
alias cbuav="cd $DEV_DIR/ros2_ws && colcon build --packages-select uavros2"
alias cbsim="cd $DEV_DIR/ros2_ws && colcon build --packages-up-to uavros2"

################# ROS #################
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

################# Sim dispatcher aliases #######################
# Top-level multi-simulator launch dispatcher.
alias warehouse='ros2 launch uavros2 sim.launch.py simulator:=gazebo world:=warehouse'
alias sim_gz='ros2 launch uavros2 sim.launch.py simulator:=gazebo'
alias sim_wb='ros2 launch uavros2 sim.launch.py simulator:=webots'
alias sim_mj='ros2 launch uavros2 sim.launch.py simulator:=mujoco'
alias sim_isaac='ros2 launch uavros2 sim.launch.py simulator:=isaac'
alias sim_pb='ros2 launch uavros2 sim.launch.py simulator:=pybullet'

# Arm-mounted shortcuts (Gazebo only — other sims pinned per arms/manifest.yaml).
alias arm_three_dof='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_with_three_dof_arm arm:=three_dof'
alias arm_openmanip='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_with_openmanip_x arm:=openmanip_x'
alias arm_panda='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_with_panda arm:=panda'
alias arm_ur5='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_with_ur5 arm:=ur5'

# MuJoCo Panda sweet spot.
alias mujoco_panda='ros2 launch uavros2 sim.launch.py simulator:=mujoco uav:=x500 arm:=panda'

################# Urban / outdoor world shortcuts (Gazebo) #######################
# Heightmap-based outdoor worlds. Each needs git-lfs to pull the matching
# models/urban<N>_terrain heightmap binaries.
alias mono_urban1='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_mono_cam_3d_lidar world:=urban1'
alias mono_urban2='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_mono_cam_3d_lidar world:=urban2'
alias mono_urban3='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_mono_cam_3d_lidar world:=urban3'
alias mono_urban4='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_mono_cam_3d_lidar world:=urban4'
alias mono_urban5='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_mono_cam_3d_lidar world:=urban5'
alias stereo_urban1='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_stereo_cam_3d_lidar world:=urban1'
alias stereo_urban2='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_stereo_cam_3d_lidar world:=urban2'
alias stereo_urban3='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_stereo_cam_3d_lidar world:=urban3'
alias stereo_urban4='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_stereo_cam_3d_lidar world:=urban4'
alias stereo_urban5='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_stereo_cam_3d_lidar world:=urban5'
alias twin_urban2='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_twin_stereo_twin_velodyne world:=urban2'
alias twin_urban5='ros2 launch uavros2 sim.launch.py simulator:=gazebo uav:=x500_twin_stereo_twin_velodyne world:=urban5'

################# Github Repos #################

export GIT_USER=
export GIT_TOKEN=
