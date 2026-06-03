# Manipulators

`uav_gz_sim` ships four floating-base manipulator configurations. Each composes an x500 quadcopter with a different robotic arm, mounted underneath the fuselage.

| Arm | DOF | Approx mass | Best for | Status |
|---|---|---|---|---|
| `three_dof` | 3 | 0.3 kg | All-sim baseline, control prototyping | Working in all sims |
| `openmanip_x` | 4 | 0.5 kg | Realistic payload for x500 | Working in Gazebo |
| `panda` | 7 | 17 kg* | Rich controller stack; MuJoCo arm tests | Working in Gazebo + MuJoCo |
| `ur5` | 6 | 18 kg* | Industrial baseline | Working in Gazebo |

\* Panda and UR5 are physically unrealistic on an x500 — they are included for controller-stack testing, not realistic flight dynamics.

## Per-arm directory layout

```
arms/<arm>/
├── urdf/<arm>.urdf.xacro         # canonical kinematics + inertia + visual + collision
├── meshes/                       # shared STL/DAE/OBJ
├── sdf/model.sdf                 # Gazebo (hand-maintained for PX4 path)
├── mjcf/<arm>.xml                # MuJoCo (generated)
├── usd/<arm>.usd                 # Isaac (generated)
├── proto/<arm>.proto             # Webots (generated)
├── config/controllers.yaml       # ros2_control: joint_trajectory_controller + state broadcaster
├── config/moveit/                # MoveIt config (panda, ur5, openmanip_x)
└── asset.yaml                    # mount point, joint names, EE link, default controller
```

## Mount composition

`arms/mounts/x500_<arm>.urdf.xacro` xacro-includes the x500 base URDF and the arm URDF, joining them with a rigid joint at the arm's declared mount frame. Composed models live under `models/x500_with_<arm>/`. Each gets its own PX4 airframe (4025–4028) that exposes the arm joints as additional simulated actuators.

## Launching the arm

```bash
ros2 launch uav_gz_sim sim.launch.py simulator:=gazebo uav:=x500_d435 arm:=panda
ros2 launch uav_gz_sim arm_control.launch.py arm:=panda use_moveit:=true
```

`arm_control.launch.py` spawns `ros2_control_node` with the per-arm `controllers.yaml`, then starts `joint_state_broadcaster` and `joint_trajectory_controller`. With `use_moveit:=true` it also brings up `move_group`.

## Sending arm commands

The arm exposes a standard ROS 2 control interface:

- `/<ns>/joint_states` — `sensor_msgs/JointState` (canonical contract)
- `/<ns>/joint_command` — `trajectory_msgs/JointTrajectory` (canonical contract)
- `/<ns>/joint_trajectory_controller/follow_joint_trajectory` — action (standard `ros2_control`)
- MoveIt: standard `move_group` action/services under `/<ns>/move_group`

## Adding a new arm

1. Author the URDF/Xacro in `arms/<name>/urdf/`.
2. Write `asset.yaml` (mount frame, joint names, EE link, default controller).
3. Add `config/controllers.yaml` (ros2_control YAML).
4. Run `scripts/convert/regen_assets.sh` to produce MJCF/USD/PROTO.
5. For Gazebo PX4: hand-author `sdf/model.sdf` and add a PX4 airframe at `config/px4/40NN_gz_x500_with_<name>`.
6. Write `arms/mounts/x500_<name>.urdf.xacro` and add to `arms/manifest.yaml`.
7. Re-run `install.sh`.

## Known limitations

- **No floating-base controller for arm reaction torques** — the x500 controller (PX4) treats the arm as fixed inertia; aggressive arm motion will perturb the quad. Future work: a coupled controller in `sim_control_bridge`.
- **MoveIt configs are scaffolded only** — real-arm parameters (joint limits from manufacturer spec, collision matrix tuning) need engineering.
- **Webots arm integration deferred** — `webots_ros2_control` is fiddly; only x500 base ships for Webots in this pass.
- **Panda / UR5 are payload-unrealistic** — they are included as controller-stack testbeds, not for realistic UAV-manipulation experiments.
