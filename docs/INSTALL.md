# Install

## Full install (host, all features)

```bash
git clone https://github.com/asmbatati/uav_gz_sim.git ~/drone_arm_ws/ros2_ws/src/uav_gz_sim
cd ~/drone_arm_ws/ros2_ws/src/uav_gz_sim
git submodule update --init --recursive
export DEV_DIR=~/drone_arm_ws
./install.sh
```

This installs ROS 2 Jazzy, Gazebo Harmonic, PX4 SITL, MAVROS (custom fork), YOLOv8 ROS, RMW Zenoh, GeographicLib, and QGroundControl. After completion:

```bash
source ~/drone_arm_ws/ros2_ws/install/setup.bash
zenoh &                                                 # in one terminal
ros2 launch uav_gz_sim sim.launch.py                    # in another
```

## Minimal install (CI / headless)

Skip YOLOv8, GeographicLib, QGroundControl downloads:

```bash
./install.sh --minimal
```

## Multi-simulator install

```bash
./install.sh --simulators=gazebo,webots,mujoco
```

Each simulator installs its own dependencies:
- `gazebo` → `ros-jazzy-ros-gz`
- `webots` → `ros-jazzy-webots-ros2`
- `mujoco` → `pip install mujoco`, `ros-jazzy-ros2-control`
- `isaac` → printed install pointer (Omniverse Launcher required)
- `pybullet` → `pip install pybullet`
- `genesis` → printed pointer

Without `--simulators`, only `gazebo` deps install (the only sim with a complete PX4 path).

## Docker

The Docker setup is ported from `gps_denied_navigation_docker` (a known-optimized configuration with NVIDIA GPU support, X11 forwarding, SSH-agent passthrough, and shared-volume bind-mount). It now lives **in-tree** under `docker_setup/` — no submodule.

Available images (see `docker_setup/docker/Makefile`):

| Make target | Base | Notes |
|---|---|---|
| `px4-dev-simulation-ubuntu22` | Ubuntu 22.04 + ROS Humble | CPU-only |
| `px4-simulation-cuda11.7.1-ubuntu22` | + CUDA 11.7 | NVIDIA GPU compute |
| `px4-simulation-cuda12.2.0-ubuntu22` | + CUDA 12.2 | Default; what `docker_run.sh` uses |
| `px4.simulation.ubuntu22.wsl` | Ubuntu 22 + WSL2 tweaks | Windows host via WSL |

Build + run:

```bash
cd ros2_ws/src/uav_gz_sim/docker_setup/docker
make px4-simulation-cuda12.2.0-ubuntu22       # builds mzahana/px4-simulation-cuda12.2.0-ubuntu22

cd ..                                          # back to docker_setup/
./docker_run.sh                                # default container name "gpsdnav"
# (or `./docker_run.sh myname` for a different name; that controls the
#  shared-volume dir at ~/${name}_shared_volume)

# inside the container:
cd ~/shared_volume/ros2_ws/src/uav_gz_sim
./install.sh
```

The container working directory `/home/user/shared_volume` is bind-mounted to `~/${CONTAINER_NAME}_shared_volume` on the host. Inside the container, `DEV_DIR=/home/user/shared_volume` is exported automatically.

> **Note on host OS / ROS version:** The Dockerfiles target Ubuntu 22.04 + ROS 2 Humble (matching the gps_denied_navigation_docker upstream). The host workspace here is on Ubuntu 24.04 + ROS 2 Jazzy. The two stacks coexist: host-side dev uses Jazzy directly; container-side dev uses the Humble image with the same source tree bind-mounted. A Jazzy-host-native Dockerfile is a roadmap item.

## Environment variables

| Variable | Default | Purpose |
|---|---|---|
| `DEV_DIR` | `$HOME/drone_arm_ws` | Parent of `PX4-Autopilot/` and `ros2_ws/` |
| `GIT_USER` | unset | Optional, for authenticated cloning |
| `GIT_TOKEN` | unset | Optional, for authenticated cloning |
| `RMW_IMPLEMENTATION` | `rmw_zenoh_cpp` (set by `bash.sh`) | RMW for ROS 2 discovery |
| `ROS_DOMAIN_ID` | `18` (set by `bash.sh`) | DDS domain |

## Troubleshooting

**`make px4_sitl` fails with submodule errors:** Re-run `cd $PX4_DIR && git submodule update --init --recursive`.

**`colcon build` OOMs on mavros:** The installer already uses `MAKEFLAGS='j1 -l1' --executor sequential` for mavros. If you're rebuilding manually, do the same.

**`No transport available` on launch:** You forgot to start `zenoh` first, or `RMW_IMPLEMENTATION` is unset/wrong.

**Wrong `DEV_DIR` baked into `~/.bashrc`:** Edit the `source $DEV_DIR/bash.sh` line in `~/.bashrc` to your actual `DEV_DIR`, or remove it and re-run `install.sh` with the correct `DEV_DIR` exported.
