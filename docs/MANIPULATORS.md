# Manipulators

`uavros2` ships four arms, each mounted under an x500 quadcopter and driven
by `gz_ros2_control` in Gazebo while PX4 flies the drone.

| Arm | DOF | Model | Arm mass | PX4 airframe | Status (Gazebo + PX4 v1.15.4) |
|---|---|---|---|---|---|
| `openmanip_x` | 4 | **Real ROBOTIS OpenManipulator-X**: upstream kinematics, masses, inertias, joint limits; box geometry | 0.594 kg | 4026 | Flies the full experiment; MoveIt config |
| `three_dof` | 3 | Cylinder-chain scaffold (planar pitch chain) | 0.12 kg | 4025 | Flies the full experiment; MoveIt config |
| `panda` | 7 | Scaffold with Franka Panda joint names (planar pitch chain) | 0.20 kg | 4027 | Flies the full experiment; MoveIt config |
| `ur5` | 6 | Scaffold with UR5 joint names (planar pitch chain) | 0.20 kg | 4028 | Flies the full experiment; MoveIt config |

The three scaffolds are light placeholders that fit under the landing gear in
any pose: good for controller-stack and MoveIt testing, too light to disturb
the drone (see [AERIAL_ARM.md](AERIAL_ARM.md)). The OpenManipulator-X is the
physically meaningful one: at 0.59 kg (30 % of the x500) its motion visibly
moves the drone. A real Panda (18 kg) or UR5 (18 kg) cannot fly on an x500,
so those two stay scaffolds.

## Files per arm

```
arms/<arm>/
├── urdf/<arm>.urdf.xacro     # kinematics + ros2_control block; read by robot_state_publisher,
│                             # gz_ros2_control (joint list) and MoveIt
├── config/controllers.yaml   # joint_state_broadcaster + joint_trajectory_controller (/**: wildcard)
├── config/moveit/            # SRDF, kinematics, OMPL, controllers, joint limits
├── mjcf/ usd/ proto/         # per-simulator conversions (REGEN_PENDING for now)
└── asset.yaml                # mount, joints, EE link, mass, notes
models/x500_with_<arm>/model.sdf   # what Gazebo simulates: x500 + the arm's links and joints
config/px4/40NN_gz_x500_with_<arm> # PX4 airframe (generated from assets/uavs/x500_with_<arm>.yaml)
```

The URDF and the composed SDF describe the same chain twice, once for ROS
(TF, MoveIt) and once for Gazebo (physics). `test/test_arm_kinematics.py`
checks that their end-effector forward kinematics agree in 21 joint
configurations per arm, and that each arm's spawn pose sits above the gear
feet. Run it after editing either file.

## Mount

Every arm hangs 5 cm below `base_link`, flipped (roll = π) so the chain
points down. The mount is part of each arm's URDF (the first joint's origin,
or a fixed `arm_mount` joint for the OpenManipulator-X) and of the SDF (the
first link's `<pose relative_to='base_link'>`), and is declared in
`asset.yaml` (`mount_xyz`, `mount_rpy`).

The x500 gear feet are 0.2195 m below `base_link`. The scaffolds reach at most
0.20 m, so they hang straight down at spawn. The OpenManipulator-X reaches
0.45 m, so it **spawns folded in a stow pose** (`joint2 = -1.5`,
`joint3 = 1.4`, `joint4 = 1.5`), set by `initial_value` on the position
state interfaces in its URDF; gz_ros2_control applies it at startup. Land
with the arm stowed.

## OpenManipulator-X details

From ROBOTIS-GIT/open_manipulator (Apache-2.0),
`open_manipulator_x_arm.urdf.xacro`:

| Joint | Axis | Origin (from parent) | Limits (rad) |
|---|---|---|---|
| joint1 | Z (yaw) | 0.012, 0, 0 from the base plate | ±π |
| joint2 | Y (pitch) | 0, 0, 0.0595 | ±1.5 |
| joint3 | Y | 0.024, 0, 0.128 | −1.5 … 1.4 |
| joint4 | Y | 0.124, 0, 0 | −1.7 … 1.97 |
| end effector | – | 0.126, 0, 0 | – |

Link masses 0.079 / 0.098 / 0.139 / 0.133 / 0.145 kg (base plate to
wrist, gripper fingers folded into the wrist). Effort limit 4.1 N m
(Dynamixel XM430-W350 stall torque at 12 V), velocity 4.8 rad/s. The
gripper's two prismatic fingers are not modelled yet. The airframe's
`MPC_THR_HOVER` is 0.72 (PX4's hover-thrust estimator refines it in flight).

## Launching

```bash
ros2 launch uavros2 sim.launch.py simulator:=gazebo \
    uav:=x500_with_openmanip_x arm:=openmanip_x
```

UAV names are `x500_with_<arm>`, except `three_dof` → `x500_with_three_dof_arm`.
Add `use_moveit:=true` for `move_group`, `headless:=1 use_rviz:=false` for a
run without windows, and `instance:=N` to use PX4 instance N (MAVLink ports
14540+N / 14580+N) when another simulation already uses instance 0.

## Commanding the arm

All under the `drone` namespace:

- `/drone/joint_states` — `sensor_msgs/JointState`
- `/drone/joint_trajectory_controller/follow_joint_trajectory` — `control_msgs/action/FollowJointTrajectory`
- `/drone/joint_trajectory_controller/controller_state` — reference, feedback and error
- MoveIt: `move_group` under `/drone` when `use_moveit:=true`

```bash
ros2 action send_goal /drone/joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{ trajectory: { joint_names: [joint1, joint2, joint3, joint4],
                   points: [{positions: [0.0, 0.8, -0.3, 0.4],
                             time_from_start: {sec: 2}}] } }"
```

Joint names per arm:
- `three_dof`: `j_shoulder, j_elbow, j_wrist`
- `openmanip_x`: `joint1, joint2, joint3, joint4`
- `panda`: `panda_joint1` … `panda_joint7`
- `ur5`: `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`

The flight experiment that takes off, moves the arm and measures the effect is
described in [AERIAL_ARM.md](AERIAL_ARM.md).

## Replacing a scaffold with a real arm

This is how the OpenManipulator-X was upgraded:

1. Rewrite `arms/<arm>/urdf/<arm>.urdf.xacro` with the real joint origins,
   axes, limits and inertials (keep the joint names, so `controllers.yaml`
   and the MoveIt groups keep working), with the mount as the first joint's
   origin.
2. Regenerate the arm part of `models/x500_with_<arm>/model.sdf` from it
   (link `<pose relative_to=parent>` = the URDF joint origin; keep the
   `gz_ros2_control` plugin block and its `@UAVROS2_PKG_SHARE@` /
   `@UAVROS2_NAMESPACE@` markers).
3. If the arm reaches past the gear, give it a stow pose with
   `initial_value` params in the URDF's `ros2_control` block.
4. Update `config/moveit/<arm>.srdf` (link names, group states, collision
   disables) and `asset.yaml` / `assets/arms/<arm>.yaml` (mass, links).
5. Set `px4.hover_thrust` in `assets/uavs/x500_with_<arm>.yaml`, run
   `asset build x500_with_<arm>`, then `scripts/sync_px4_assets.sh` and
   `make px4_sitl`.
6. `pytest test/test_arm_kinematics.py`.

## Adding a new arm

1. Author the URDF/Xacro in `arms/<name>/urdf/` (mount as the first joint).
2. Write `asset.yaml` and `assets/arms/<name>.yaml`.
3. Add `config/controllers.yaml` (wrap params under `/**:` so the namespaced
   controller_manager picks them up).
4. Optionally ship `config/moveit/`; `arm_control.launch.py` auto-detects it.
5. Author `models/x500_with_<name>/model.sdf` with the arm links and the
   `gz_ros2_control` plugin block.
6. Add `assets/uavs/x500_with_<name>.yaml` (next free airframe ID is 4029),
   `asset build`, register the UAV in `UAV_AIRFRAME` in
   `launch/simulators/gazebo.launch.py`, the arm in `SUPPORTED_ARMS`
   (`launch/sim.launch.py`) and `arms/manifest.yaml`, and the arm's poses in
   `uavros2/aerial_arm_demo.py`.
7. Add it to `ARMS` in `test/test_arm_kinematics.py`.

## Known limitations

- **No arm-aware flight control.** PX4 treats the arm as an unknown
  disturbance; the OpenManipulator-X's reach moves the drone 15-40 cm (see
  AERIAL_ARM.md). Feed-forward compensation is open work.
- **No gripper yet** on the OpenManipulator-X.
- **Gazebo only.** Webots ships the x500 base only; MuJoCo / Isaac / PyBullet /
  Genesis arm conversions are `REGEN_PENDING` placeholders.
