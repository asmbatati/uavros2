# Architecture

`uavros2` is built around three pillars: a **simulator-dispatching launch system**, a **canonical ROS 2 topic contract**, and a **per-simulator adapter layer**. Downstream code (vision, control, recording) interacts only with the canonical contract, so swapping the physics engine never requires touching the application layer.

```
                ┌───────────────────────────────────────────────────┐
                │           ros2 launch uavros2 sim.launch.py   │
                │    simulator:= uav:= arm:= world:=               │
                └────────────────┬──────────────────────────────────┘
                                 │ OpaqueFunction dispatch
        ┌────────────┬───────────┼───────────┬───────────┬─────────┐
        ▼            ▼           ▼           ▼           ▼         ▼
   gazebo.       webots.    mujoco.     isaac.     pybullet.   genesis.
   launch.py     launch.py  launch.py   launch.py  launch.py   launch.py
   (PX4 SITL +   (PX4       (sim_ctrl_  (STUB +    (STUB +     (STUB)
    MAVROS +     webots +   bridge +    Pegasus    placeholder
    ros_gz_      ros2 dr.)  mujoco_     docs)      controller)
    bridge)                 ros2_ctrl)
        │            │           │           │           │         │
        └────────────┴───────────┴─────┬─────┴───────────┴─────────┘
                                       ▼
                       canonical_topics.yaml (contract)
                                       ▼
                       sim_common.launch.py
                       (tf_relay, stitcher, gt_traj, RViz)
                                       ▼
                       Downstream: vision, control, recording
```

## Asset format conventions

Single source of truth: **Xacro-URDF** for kinematics, inertia, visual, and collision. Each asset directory carries pre-generated per-simulator outputs that are **committed to git** (not regenerated at launch):

```
arms/<arm>/
├── urdf/<arm>.urdf.xacro     # canonical
├── meshes/                   # shared STL/DAE/OBJ
├── sdf/model.sdf             # hand-maintained for Gazebo PX4 path
├── mjcf/<arm>.xml            # generated for MuJoCo
├── usd/<arm>.usd             # generated for Isaac
├── proto/<arm>.proto         # generated for Webots
├── config/controllers.yaml   # ros2_control
├── config/moveit/            # MoveIt config (where upstream exists)
└── asset.yaml                # mount-point, joint names, EE link, default controller
```

The same convention applies to UAVs (`models/<uav>/`) and composed `x500_with_<arm>/` mounts.

**Why pre-generate?** URDF→USD requires running inside the Isaac Python interpreter (not pip-installable). URDF→MJCF (`urdf2mjcf` + hand patch) and URDF→PROTO (`webots_ros2_importer`) are slow and sometimes interactive. Regenerating at launch would make `ros2 launch` unpredictable. Conversion lives in `scripts/convert/` and is a CI step.

## Canonical topic contract

`config/topics/canonical_topics.yaml` declares every topic downstream nodes consume. Each `simulators/<sim>.launch.py` is responsible for *producing* those names. Sensors and state estimates appear under the same names regardless of simulator:

- `/<ns>/imu`, `/<ns>/gps`, `/<ns>/air_pressure`
- `/<ns>/front_stereo/{left,right}/{image_raw,camera_info}`
- `/<ns>/front_lidar/points`
- `/<ns>/mavros/local_position/{pose,odom}` (synthesized by `sim_control_bridge` on non-PX4 sims)
- `/<ns>/joint_states`, `/<ns>/joint_command` (arm)
- `/<ns>/setpoint_{position/pose,velocity/cmd_vel,raw/attitude}` (control input)

`sensor_relay.py` exists as a one-hop fallback for sims where the native bridge can't be configured to emit canonical names directly (likely Webots and possibly community Isaac graphs).

## `sim_control_bridge`

Unified ROS 2 node with a pluggable per-simulator backend in `uavros2/adapters/`:

| Simulator | Backend behavior |
|---|---|
| gazebo, webots, isaac | **Inert** — PX4/MAVROS or the native sim bridge already publishes MAVROS-shaped topics. |
| mujoco, pybullet, genesis | **Active** — steps the physics engine from setpoints, runs a rotor mixer + attitude/position PID, synthesizes MAVROS-shaped pose/odom/IMU so downstream code sees identical topics. |

The public ROS 2 interface is the canonical topic contract above. Backends never expose simulator-specific topics outward.

## Arm control

`launch/arm_control.launch.py` brings up `ros2_control_node` with the per-arm `controllers.yaml`, spawning `joint_state_broadcaster` and `joint_trajectory_controller`. MoveIt's `move_group` launches optionally (`use_moveit:=true`) where the arm ships a MoveIt config (Panda, UR5, OpenManipulator-X).

For Gazebo, the arm's hardware interface is `gz_ros2_control/IgnitionSystem`. For MuJoCo, `mujoco_ros2_control`. PyBullet/Genesis use `MockHardware` as a placeholder pending native integrations.

## Cleanup invariant

When you edit a UAV or arm asset:

1. Edit the URDF/Xacro source.
2. Run `scripts/convert/regen_assets.sh` (regenerates MJCF/USD/PROTO).
3. For Gazebo PX4 path: also edit the hand SDF and re-run `install.sh` (or copy manually to `$PX4_DIR/Tools/simulation/gz/models/`).
4. Commit both the source and the regenerated outputs.
