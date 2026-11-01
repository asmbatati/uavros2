# uavros2: PX4 UAV and aerial-manipulator simulation for ROS 2

<div align="center">

<img src="media/px4_logo.png" alt="PX4" width="80" />
<img src="media/ros2_logo.png" alt="ROS 2" width="80" />
<img src="media/gazebo_logo.png" alt="Gazebo" width="80" />
<img src="media/docker_logo.png" alt="Docker" width="80" />
<img src="media/mavros_logo.png" alt="MAVROS" width="80" />

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-green.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![PX4](https://img.shields.io/badge/PX4-v1.15.4-red.svg)](https://px4.io/)

</div>

`uavros2` simulates PX4 multirotors in Gazebo Harmonic with ROS 2 Jazzy and
MAVROS: a catalogue of x500 sensor variants, an indoor warehouse and five
outdoor terrain worlds, four manipulator arms mounted under the x500 (one of
them a mass-accurate ROBOTIS OpenManipulator-X), a YAML asset generator for
models and PX4 airframes, RViz visualisation, and a dispatcher that can start
the same scene in other simulators. Downstream code talks to a fixed set of
canonical topics, whichever simulator runs.

| Part | Status (Ubuntu 24.04, Jazzy, PX4 v1.15.4) |
|---|---|
| Gazebo + PX4 SITL + MAVROS, all UAVs | Working; flights verified headless |
| Arms under the x500 (`gz_ros2_control`, MoveIt configs) | Working; all four fly the arm experiment |
| Aerial-arm experiment + figures | Working ([docs/AERIAL_ARM.md](docs/AERIAL_ARM.md)) |
| Docker images | Ubuntu 24 / Jazzy dev image and a ready-to-run image ([uavros2_docker](uavros2_docker)) |
| Contact against a wall | Tool with force sensor, wall world, baseline experiment: plain position control gives unsteady force ([results](docs/AERIAL_ARM.md#contact-experiment)) |
| MuJoCo | x500 flies to position setpoints (no PX4); no arm |
| Webots, Isaac, PyBullet, Genesis | **Unsupported**: code kept, not maintained ([docs/SIMULATORS.md](docs/SIMULATORS.md)) |

![OpenManipulator-X moving under a hovering x500](docs/figures/aerial_arm/omx_2_s_motions_flight.png)

## Install

### On Ubuntu 24.04

`install.sh` expects the workspace layout `$DEV_DIR/{PX4-Autopilot,ros2_ws}`
(default `DEV_DIR=~/drone_arm_ws`). It installs ROS 2 Jazzy, Gazebo Harmonic
and the PX4 toolchain if missing (sudo), clones PX4 **v1.15.4** (override with
`PX4_REF`) and `uavros2_msgs`, installs MAVROS from apt, copies this package's models,
worlds and airframes into PX4, builds PX4 SITL and the workspace, and adds
`source $DEV_DIR/bash.sh` to `~/.bashrc`.

```bash
mkdir -p ~/drone_arm_ws/ros2_ws/src && cd ~/drone_arm_ws/ros2_ws/src
git clone --recursive https://github.com/asmbatati/uavros2.git
cd uavros2 && ./install.sh --minimal     # --minimal: skip YOLOv8 and QGC downloads
```

The models are stored with git-lfs; run `git lfs pull` if the terrain meshes
come out as small text files.

### With Docker

```bash
cd uavros2/uavros2_docker/docker
make px4-dev-simulation-ubuntu24     # ROS 2 Jazzy + Gazebo + PX4 toolchain + MoveIt + ros2_control
make uavros2                         # + PX4 v1.15.4 SITL and this package, built (uavros2:jazzy)
```

`docker_run.sh` starts an interactive dev container from the first image with
X11 and the GPU; the second runs experiments with no display. See
[uavros2_docker/README.md](uavros2_docker/README.md).

## Run

```bash
source ~/drone_arm_ws/ros2_ws/install/setup.bash
ros2 launch uavros2 sim.launch.py                         # x500_stereo_cam_3d_lidar in warehouse
ros2 launch uavros2 sim.launch.py uav:=x500_with_openmanip_x arm:=openmanip_x
ros2 launch uavros2 sim.launch.py uav:=x500_d435 world:=urban5 headless:=1 use_rviz:=false
```

| Argument | Default | Meaning |
|---|---|---|
| `simulator` | `gazebo` | `gazebo`, `mujoco` (`webots`, `isaac`, `pybullet`, `genesis` are unsupported) |
| `uav` | `x500_stereo_cam_3d_lidar` | model under `models/` (table below) |
| `arm` | `none` | `three_dof`, `openmanip_x`, `panda`, `ur5`; pair with `uav:=x500_with_<arm>` |
| `world` | `warehouse` | `warehouse`, `urban1` … `urban5` (spawn heights in `worlds/manifest.yaml`) |
| `namespace` | `drone` | ROS namespace of MAVROS, sensors and the arm |
| `instance` | `0` | PX4 SITL instance N: MAV_SYS_ID N+1, MAVLink UDP 14540+N / 14580+N, Gazebo model `<uav>_N` |
| `headless` | `0` | `1`: no Gazebo window; sensors render through EGL, no X display needed |
| `use_rviz` | `true` | RViz with the drone model and world terrain |
| `use_moveit` | `false` | start `move_group` for the arm |

Fly it with the bash helpers from `scripts/bash.sh` (`arm`, `takeoff`, `land`,
`offboard`, …) or programmatically through MAVROS; see
[docs/FLYING.md](docs/FLYING.md).

### Several simulations on one machine

Each simulation needs its own `GZ_PARTITION` (Gazebo transport), its own
`ROS_DOMAIN_ID`, and its own PX4 `instance:=`; set `PX4_UXRCE_DDS_PORT` too if
another sim runs a uXRCE-DDS agent. Before starting, the launch kills stale
Gazebo / PX4 / bridge processes of *its own* partition, PX4 tree and instance
only (`uavros2/sim_cleanup.py`), so other users' simulations are never
touched.

```bash
export GZ_PARTITION=mysim ROS_DOMAIN_ID=24 PX4_UXRCE_DDS_PORT=8924
ros2 launch uavros2 sim.launch.py instance:=4 headless:=1 use_rviz:=false
```

## Aerial-arm experiment

```bash
scripts/aerial_arm_experiment.sh openmanip_x 4 ~/uavros2_results   # launch, fly, stop (headless)
scripts/plot_aerial_arm.py ~/uavros2_results                       # figures + summary table
```

The drone takes off in OFFBOARD, hovers at 2 m, moves the arm through four
poses (forward reach, side reach, fold, stow) and lands, logging hover error,
joint tracking and the end effector's world position. The OpenManipulator-X
(0.59 kg) displaces the x500 by 2-5 times its hover noise (peaks up to 0.5 m
for 2 s arm motions, roughly halved at 6 s); the light scaffold arms do not
measurably. Protocol, numbers and figures:
[docs/AERIAL_ARM.md](docs/AERIAL_ARM.md).

## Models

| UAV | PX4 airframe | Sensors / payload |
|---|---|---|
| `x500` | 4001 (stock) | none |
| `x500_d435` | 4420 | RealSense D435 |
| `x3_uav` | 4421 | IMU, baro, magnetometer, GPS (the iHunter interceptor airframe) |
| `x500_mono_cam_3d_lidar` | 4422 | mono camera, 3D lidar |
| `x500_stereo_cam_3d_lidar` | 4423 | stereo camera, 3D lidar |
| `x500_twin_stereo_twin_velodyne` | 4424 | two stereo pairs, two Velodynes |
| `x500_with_three_dof_arm` | 4425 | 3-DOF scaffold arm |
| `x500_with_openmanip_x` | 4426 | OpenManipulator-X (real kinematics and masses) |
| `x500_with_panda` | 4427 | Panda-named 7-DOF scaffold |
| `x500_with_ur5` | 4428 | UR5-named 6-DOF scaffold |
| `standard_vtol_with_cam`, `fixed_wing_basic_with_cam` | 4450, 4451 | camera (asset generator examples) |

UAVs are described in `assets/uavs/*.yaml`; `uavros2-asset build <uav>`
generates the Gazebo model and PX4 airframe ([docs/ASSETS.md](docs/ASSETS.md)).
PX4 only sees what is copied into its tree: after editing `models/`,
`worlds/` or `config/px4/`, run `scripts/sync_px4_assets.sh` and
`make px4_sitl` in PX4-Autopilot.

## Develop and test

```bash
cd ~/drone_arm_ws/ros2_ws
colcon build --packages-select uavros2 && source install/setup.bash
cd src/uavros2 && python3 -m pytest test/ -q
```

The suite covers the asset schemas and generators, xacro parsing, the
canonical topic contract, the `sim_control_bridge` mixer/PID, the runtime
sensor manager, the stale-process sweep, and URDF-vs-Gazebo kinematics of
every arm. `test_flake8` / `test_pep257` currently fail on style (double
quotes vs ament's single-quote rule).

## Documentation

| Doc | Content |
|---|---|
| [INSTALL.md](docs/INSTALL.md) | Installer details and flags |
| [FLYING.md](docs/FLYING.md) | Arming, takeoff, OFFBOARD, bash helpers |
| [MANIPULATORS.md](docs/MANIPULATORS.md) | Arms, mount, OpenManipulator-X, commanding, adding an arm |
| [AERIAL_ARM.md](docs/AERIAL_ARM.md) | Flight experiment protocol and results |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | Dispatcher, canonical topics, `sim_control_bridge` |
| [COMMUNICATION.md](docs/COMMUNICATION.md) | MAVROS vs uXRCE-DDS |
| [SIMULATORS.md](docs/SIMULATORS.md) | Per-simulator status |
| [ASSETS.md](docs/ASSETS.md), [MODELS.md](docs/MODELS.md), [WORLDS.md](docs/WORLDS.md) | Asset descriptors, models, worlds |

## Troubleshooting

- **`gazebo already running` / PX4 does not spawn:** a stale server in your
  partition. The launch sweeps it; if you started Gazebo by hand, stop it.
- **A node dies with "A module that was compiled using NumPy 1.x cannot be
  run in NumPy 2.x":** a pip-installed numpy 2 in `~/.local` shadows Ubuntu's
  numpy 1.26 that ROS's `cv_bridge` is built against. Uninstall it or run
  with `PYTHONNOUSERSITE=1`.
- **`uavros2-asset` fails with `field_validator`:** it needs pydantic 2
  (`pip install --break-system-packages "pydantic>=2"`); Ubuntu ships 1.10.
- **Terrain meshes grey in RViz:** RViz cannot read TIFF; `sync_px4_assets.sh`
  converts the textures to PNG.

## Acknowledgments

- [RIOTU Lab](https://github.com/riotu-lab)
- [Dr. Mohamed Abdelkader Zahana](https://github.com/mzahana): PX4 integration, multi-agent systems, drone control
- [Eng. Khaled Gabr](https://github.com/khaledgabr77): autonomous navigation and ROS 2

Parts of this package were refactored from
[d2dtracker_sim](https://github.com/mzahana/d2dtracker_sim) and
[gps_denied_navigation_sim](https://github.com/riotu-lab/gps_denied_navigation_sim).
The OpenManipulator-X parameters come from
[ROBOTIS open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) (Apache-2.0).

## License

MIT, see [LICENSE](LICENSE). The OpenManipulator-X parameters in `arms/openmanip_x/` are from ROBOTIS (Apache-2.0).
