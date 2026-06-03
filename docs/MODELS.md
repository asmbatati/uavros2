# UAV Models

x500 quadcopter variants ship as SDF assets (Gazebo PX4 path). Each composes the upstream PX4 `x500` base with different sensor payloads.

| Model | Sensors / Payload | PX4 airframe |
|---|---|---|
| `x500` | none (bare quad) | 4001 (PX4 default) |
| `x500_d435` | Intel RealSense D435 (depth) | 4020 |
| `x3_uav` | x3 quadrotor variant | 4021 |
| `x500_mono_cam_3d_lidar` | mono camera + Velodyne-16 | 4022 |
| `x500_stereo_cam_3d_lidar` | stereo camera + Velodyne-16 | 4023 |
| `x500_twin_stereo_twin_velodyne` | dual stereo + dual Velodyne | 4024 |
| `x500_with_three_dof_arm` | custom 3-DOF arm scaffold | 4025 |
| `x500_with_openmanip_x` | 4-DOF scaffold (OpenManipulator-X joint names) | 4026 |
| `x500_with_panda` | 7-DOF scaffold (Panda joint names) | 4027 |
| `x500_with_ur5` | 6-DOF scaffold (UR5 joint names) | 4028 |
| `x500_depth`, `x500_gimbal` | variants kept for reference | — |

The arm variants (4025–4028) are cylinder-chain scaffolds sized to fit under the x500 landing gear — see [MANIPULATORS.md](MANIPULATORS.md) for the per-arm DOF / mass / joint-name details.

Non-x500 platforms (`x3_uav`, `parrot_bebop_2`) are kept as legacy references.

## Adding a new UAV variant

1. Add SDF under `models/<name>/{model.config,model.sdf}` (Gazebo path).
2. Add a PX4 airframe under `config/px4/<id>_gz_<name>` (next free ID is 4029+).
3. Re-run `install.sh` so the model + airframe are copied into `$PX4_DIR`.
4. (Optional, for multi-sim) Author a URDF/Xacro under `sim_assets/uav/<name>.urdf.xacro` and run `scripts/convert/regen_assets.sh` to produce MJCF/USD/PROTO.

## Mount points for arms

Composed `x500_with_<arm>` mounts live under `models/x500_with_<arm>/`. The mount xacro at `arms/mounts/x500_<arm>.urdf.xacro` xacro-includes the x500 base URDF and the arm URDF, joining them with a rigid joint at the configured mount frame (default: `x500/base_link` z = -0.05 m, pointing down).

Each arm declares its mount frame and any required ballast in its `asset.yaml`. The composer reads both and emits a single URDF that ros2_control + sim_control_bridge consume.

## Sensor topic remapping

Sensor topic names come from `config/topics/canonical_topics.yaml` and are produced by the per-simulator launch (Gazebo: `ros_gz_bridge` remaps; Webots: `webots_ros2_driver`; MuJoCo: `sim_control_bridge` direct publication; etc.). The model's SDF/URDF declares the sensor; the launch layer connects it to the canonical name.
