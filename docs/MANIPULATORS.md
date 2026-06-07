# Manipulators

`uavros2` ships four floating-base manipulator configurations. Each composes an x500 quadcopter with a different robotic arm, mounted underneath the fuselage.

| Arm | DOF | Scaffold mass | Best for | Status |
|---|---|---|---|---|
| `three_dof` | 3 | ~0.10 kg | All-sim baseline, control prototyping | Working in Gazebo |
| `openmanip_x` | 4 | ~0.16 kg | Scaffold matching ROBOTIS OpenManipulator-X joint names | Working in Gazebo |
| `panda` | 7 | ~0.18 kg | Scaffold matching Franka Panda joint names | Working in Gazebo |
| `ur5` | 6 | ~0.18 kg | Scaffold matching Universal Robots UR5 joint names | Working in Gazebo |

All four ship today as **cylinder-chain scaffolds** sized to fit under the x500 landing gear (gear feet at z = −0.2195 m below `base_link`; total arm drop ≤ 0.20 m). The joint names exactly match the upstream arm's convention, so a real description package can be dropped in later by overwriting just `urdf/<arm>.urdf.xacro` and `models/x500_with_<arm>/model.sdf` — controllers, MoveIt configs, and the PX4 airframe stay unchanged.

## Per-arm directory layout

```
arms/<arm>/
├── urdf/<arm>.urdf.xacro           # scaffold URDF/Xacro; ros2_control block included
├── meshes/                         # placeholder (cylinder geometry is inline)
├── sdf/                            # optional; PX4 path uses models/x500_with_<arm>/model.sdf
├── mjcf/<arm>.xml                  # generated for MuJoCo (REGEN_PENDING for the scaffolds)
├── usd/<arm>.usd                   # generated for Isaac (REGEN_PENDING)
├── proto/<arm>.proto               # generated for Webots (REGEN_PENDING)
├── config/controllers.yaml         # ros2_control: joint_state_broadcaster + joint_trajectory_controller
├── config/moveit/                  # full MoveIt config (SRDF, kinematics, OMPL, controllers, joint limits)
└── asset.yaml                      # mount point, joint names, EE link, default controller
```

## Mount composition

`arms/mounts/x500_<arm>.urdf.xacro` xacro-includes the x500 base URDF and the arm URDF, joining them with a rigid joint at the arm's declared mount frame. Composed Gazebo models live under `models/x500_with_<arm>/`. Each gets its own PX4 airframe (4025–4028) that exposes the arm joints as additional simulated actuators.

The total arm drop below `base_link` per arm:

| Arm | Mount + N × link_len + EE | Drop |
|---|---|---|
| `three_dof`   | 0.05 + 3 × 0.040 + 0.015 | **0.185 m** |
| `openmanip_x` | 0.05 + 4 × 0.033 + 0.015 | **0.197 m** |
| `panda`       | 0.05 + 7 × 0.019 + 0.015 | **0.198 m** |
| `ur5`         | 0.05 + 6 × 0.022 + 0.015 | **0.197 m** |

All four sit 21–34 mm above the landing gear feet, so the drone rests on the gear at spawn.

## Launching the arm

End-to-end Gazebo + controller stack:

```bash
ros2 launch uavros2 sim.launch.py simulator:=gazebo \
    uav:=x500_with_three_dof_arm arm:=three_dof
```

With MoveIt (any of the four arms — all ship a complete config):

```bash
ros2 launch uavros2 sim.launch.py simulator:=gazebo \
    uav:=x500_with_panda arm:=panda use_moveit:=true
```

UAV names follow `x500_with_<arm>` (except `three_dof` which is `x500_with_three_dof_arm` — the `_arm` suffix). `use_moveit:=true` is plumbed through the dispatcher and triggers `move_group` startup once `config/moveit/` is present.

## Sending arm commands

The arm exposes a standard ROS 2 control interface (all under the `drone` namespace):

- `/drone/joint_states` — `sensor_msgs/JointState` (canonical contract)
- `/drone/joint_command` — `trajectory_msgs/JointTrajectory` (canonical contract)
- `/drone/joint_trajectory_controller/follow_joint_trajectory` — action (`control_msgs/FollowJointTrajectory`)
- MoveIt: standard `move_group` action/services under `/drone/move_group`

Example direct trajectory action (`three_dof`):

```bash
ros2 action send_goal /drone/joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{ trajectory: { joint_names: [j_shoulder, j_elbow, j_wrist],
                   points: [{positions: [0.5, -0.8, 0.3],
                             time_from_start: {sec: 2}}] } }"
```

Joint names per arm:
- `three_dof`: `j_shoulder, j_elbow, j_wrist`
- `openmanip_x`: `joint1, joint2, joint3, joint4`
- `panda`: `panda_joint1` … `panda_joint7`
- `ur5`: `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`

## Replacing a scaffold with the real arm

The scaffolds are placeholders for real meshes / kinematics. To upgrade an arm to its upstream-accurate description (e.g., the real Franka Panda from `franka_description`):

1. Install the upstream description package.
2. Rewrite `arms/<arm>/urdf/<arm>.urdf.xacro` to xacro:include the upstream URDF (the `arms/mounts/x500_<arm>.urdf.xacro` already does this).
3. Author or copy a Gazebo SDF for `models/x500_with_<arm>/` with the real meshes; keep the `gz_ros2_control` plugin block (and the `@UAVROS2_PKG_SHARE@` + `@UAVROS2_NAMESPACE@` markers) so the launch-time materializer still works.
4. Update `config/controllers.yaml` if the upstream's joint names differ (they shouldn't — the scaffolds intentionally match the upstream conventions).
5. Update `arms/<arm>/config/moveit/<arm>.srdf` to reference the real link names.

Step 4 is usually a no-op because the scaffolds already use the upstream joint names. The asset.yaml's `upstream_pkg:` field documents which package to install.

## Adding a new arm

1. Author the URDF/Xacro in `arms/<name>/urdf/`.
2. Write `asset.yaml` (mount frame, joint names, EE link, default controller, `moveit: true/false`).
3. Add `config/controllers.yaml` (wrap params under `/**:` so the namespaced controller_manager picks them up).
4. Optionally ship `config/moveit/` — `arm_control.launch.py` auto-detects it.
5. Hand-author a Gazebo SDF at `models/x500_with_<name>/model.sdf` with the `gz_ros2_control` plugin block (markers `@UAVROS2_PKG_SHARE@` + `@UAVROS2_NAMESPACE@`).
6. Add a PX4 airframe at `config/px4/40NN_gz_x500_with_<name>` (next free ID is 4029) and append it to `PX4-Autopilot/ROMFS/.../airframes/CMakeLists.txt`; `install.sh` handles the latter on re-run.
7. Write `arms/mounts/x500_<name>.urdf.xacro` and register the arm in `arms/manifest.yaml`.

## Known limitations

- **Cylinder scaffolds** — current arm geometry is placeholder cylinders, not the real arm. Fine for controller-stack and MoveIt testing; less useful for visual or contact-rich experiments.
- **No floating-base controller for arm reaction torques** — the x500 controller (PX4) treats the arm as fixed inertia; aggressive arm motion will perturb the quad. Future work: a coupled controller in `sim_control_bridge`.
- **Webots arm integration deferred** — `webots_ros2_control` is fiddly; only the x500 base ships for Webots in this pass.
- **MuJoCo / Isaac / PyBullet / Genesis arms** — `arms/<arm>/{mjcf,usd,proto}/REGEN_PENDING` placeholders exist; per-sim conversion is a follow-up.
