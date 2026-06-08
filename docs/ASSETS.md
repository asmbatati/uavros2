# Asset descriptor system

> **Status**: v1 (Gazebo SDF + PX4 airframe generators landed). MuJoCo /
> Webots / Isaac generators and the runtime mutation API are roadmap.

The asset descriptor system is `uavros2`'s answer to "how do I add a
sensor / swap a chassis / spec a new UAV without hand-editing SDF?". A
YAML descriptor under `assets/` is the *source of truth*; Gazebo SDF,
PX4 airframe files, MuJoCo MJCF, etc. are *generated artefacts*.

## Mental model

```
assets/chassis/x500.yaml ───┐
assets/sensors/*.yaml  ─────┼──→ uavros2-asset build ──→ models/<uav>/model.sdf
assets/uavs/<uav>.yaml ─────┘                          ──→ config/px4/<id>_gz_<uav>
                                                       ──→ (future: MJCF, USD, PROTO)
```

A UAV descriptor (`assets/uavs/<name>.yaml`) is an **assembly** that
references primitives from the catalog by name. Primitives are
self-contained descriptors:

| Catalog dir | Primitive | What it describes |
|---|---|---|
| `assets/chassis/` | `chassis` | Airframe family: multirotor, VTOL, fixed-wing — geometry, rotors, mass, mount points |
| `assets/sensors/` | `sensor`  | Sensor model dir + ROS topic contract + per-instance default params |
| `assets/mounts/`  | `mount`   | Fixed / 1-3 axis gimbal mounts |
| `assets/arms/`    | `arm`     | Manipulator: DOF, joints, controllers, MoveIt config path |
| `assets/airfoils/`| `airfoil` | Aero coefficients for fixed-wing / VTOL wings |
| `assets/uavs/`    | `uav`     | The assembly file users mostly touch |

## CLI

The CLI installs as a ROS 2 console script. Three equivalent ways to invoke it:

```bash
# After `source install/setup.bash` — preferred:
ros2 run uavros2 uavros2-asset list --root src/uavros2

# Directly from source tree, no install needed:
cd ros2_ws/src/uavros2
PYTHONPATH=. python3 -m uavros2.asset_cli list

# Add a shell alias for convenience (drop into scripts/bash.sh):
alias asset='ros2 run uavros2 uavros2-asset --root $DEV_DIR/ros2_ws/src/uavros2'
```

The commands:

```bash
ros2 run uavros2 uavros2-asset list                              # every primitive + UAV
ros2 run uavros2 uavros2-asset show x500_stereo_cam_3d_lidar     # dump one descriptor
ros2 run uavros2 uavros2-asset validate                          # schema + cross-checks
ros2 run uavros2 uavros2-asset build x500_stereo_cam_3d_lidar    # regen one UAV's artefacts
ros2 run uavros2 uavros2-asset build --all                       # regen everything
ros2 run uavros2 uavros2-asset diff  --all                       # what would change vs committed
ros2 run uavros2 uavros2-asset build x500_d435 --target gazebo   # one generator only
```

All commands default to `--root .` so run them from the package source
dir (or pass `--root /path/to/uavros2/`).

## Writing a UAV descriptor

Minimal example (one chassis ref, three sensors, no arm):

```yaml
# assets/uavs/my_uav.yaml
version: 1
kind: uav
name: my_uav
description: x500 with a single forward camera.
chassis: { ref: x500 }
sensors:
  - { name: imu,  ref: imu_default, mount: base_link }
  - { name: gps,  ref: gps_default, mount: base_link }
  - name: front_cam
    ref: stereo_camera
    mount: base_link
    pose: [0.20, 0.0, -0.10, 0, 0.17453, 0]    # x y z roll pitch yaw, radians
px4:
  airframe_id: 4030              # must be unique across all UAVs
  spawn_pose_m: [0, 0, 0.1, 0, 0, 0]
  hover_thrust: 0.55
```

Build it:

```bash
uavros2-asset validate
uavros2-asset build my_uav
ls models/my_uav/ config/px4/4030_gz_my_uav   # generated artefacts
```

## Writing a chassis primitive

Currently supported chassis families (`type:` discriminator):

| `type` | Schema | Status |
|---|---|---|
| `multirotor`      | quad_x, quad_plus, hex_x, hex_plus, oct_x, y6, coaxial_quad | **Gazebo + PX4 generators implemented** |
| `vtol_standard`   | multirotor lift + cruise rotor + wing + control surfaces    | Schema only |
| `tailsitter`      | rotors + wing + transition                                  | Schema only |
| `tiltrotor`       | rotors + wing + transition                                  | Schema only |
| `fixed_wing`      | cruise rotor + wing + control surfaces                      | Schema only |

Example (multirotor):

```yaml
version: 1
kind: chassis
name: hex_x_research
description: Generic hex-X testbed.
type: multirotor
layout: hex_x
arm_length_m: 0.30
rotors:
  fl:  { name: rotor_fl,  pose: [ 0.15,  0.26, 0.02, 0,0,0], spin: ccw, max_thrust_N: 12 }
  fr:  { name: rotor_fr,  pose: [ 0.15, -0.26, 0.02, 0,0,0], spin: cw,  max_thrust_N: 12 }
  f:   { name: rotor_f,   pose: [ 0.30,  0.00, 0.02, 0,0,0], spin: ccw, max_thrust_N: 12 }
  r:   { name: rotor_r,   pose: [-0.30,  0.00, 0.02, 0,0,0], spin: cw,  max_thrust_N: 12 }
  rl:  { name: rotor_rl,  pose: [-0.15,  0.26, 0.02, 0,0,0], spin: cw,  max_thrust_N: 12 }
  rr:  { name: rotor_rr,  pose: [-0.15, -0.26, 0.02, 0,0,0], spin: ccw, max_thrust_N: 12 }
body: { collision: { type: cylinder, radius: 0.10, length: 0.06 } }
mass: { total_kg: 2.4, inertia: { auto: true, density_kg_per_m3: 800 } }
landing_gear: { type: skids, feet: [...] }
mount_points:
  - { name: base_link, pose: [0,0,0,0,0,0] }
```

## Per-simulator overlays

Targets often need small tweaks the descriptor schema doesn't (yet) cover.
The `overlays:` section of a UAV descriptor handles this:

```yaml
overlays:
  gazebo:
    sensor_plugins:
      front_lidar: { update_rate_hz: 20 }    # overrides catalog default
    extra_xml:                               # last-resort splice; emits a warning
      - |
        <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
          <render_engine>ogre2</render_engine>
        </plugin>
  px4:
    params:
      MPC_THR_HOVER: 0.62      # appended to the generated airframe file
  mujoco:
    not_supported: true        # tells the (future) MuJoCo generator to skip this UAV
  webots:
    not_supported: true
```

Use overlays sparingly. Anything you find yourself writing more than
twice should be lifted into the core schema.

## Adding a new sensor

1. If the sensor maps to a model dir under `models/` (most do), point the
   descriptor at it:

```yaml
# assets/sensors/livox_mid40.yaml
version: 1
kind: sensor
name: livox_mid40
description: Livox Mid-40 solid-state LiDAR.
parameters:
  range_max_m: 90
  hfov_deg: 38.4
  vfov_deg: 38.4
gazebo:
  gz_model: livox_mid40        # → <include><uri>model://livox_mid40</uri></include>
  body_link: livox_link        # → joint binding to this link
ros:
  topic_template: "/{ns}/{sensor_name}/points"
  msg_type: sensor_msgs/PointCloud2
  default_rate_hz: 10
```

2. Reference it from a UAV:

```yaml
sensors:
  - name: front_lidar
    ref: livox_mid40
    mount: base_link
    pose: [0.20, 0.0, 0.0, 0, 0, 0]
```

3. `uavros2-asset build <uav>` regenerates the UAV's SDF with the new
   include.

## CI invariant

`uavros2-asset build --all` followed by `git diff --exit-code assets/ models/ config/px4/`
must produce no output. CI runs this on every push. If it fails, either:

1. You edited a descriptor without regenerating — run `uavros2-asset build --all`.
2. You hand-edited a generated artefact — undo it, update the descriptor instead.

The descriptor is the source of truth; the artefacts are downstream of it.

## Runtime sensor hot-swap

The descriptor's *runtime* counterpart is the `runtime_sensor_manager`
node + three ROS 2 services. It lets you add / remove sensors on a
running UAV without restarting the simulation.

```bash
ros2 run uavros2 runtime_sensor_manager \
    --ros-args -p namespace:=drone -p world:=warehouse

# add a downward LiDAR while the sim is flying
ros2 service call /drone/runtime/add_sensor uavros2_msgs/srv/AddSensor \
    "{uav_namespace: drone, name: down_lidar, sensor_ref: velodyne_16,
      mount: base_link, pose: [0.0, 0.0, -0.15, 0.0, 0.0, 0.0]}"

# list what's live
ros2 service call /drone/runtime/list_sensors uavros2_msgs/srv/ListSensors \
    "{uav_namespace: drone}"

# remove it
ros2 service call /drone/runtime/remove_sensor uavros2_msgs/srv/RemoveSensor \
    "{uav_namespace: drone, name: down_lidar}"
```

The service inputs mirror the `assets/uavs/<uav>.yaml` `sensors:` list
entry, so the same descriptor fragment used at build time is reusable
at runtime. The node wraps Gazebo Sim's transport services
(`/world/<world>/create` and `/remove`) under the hood; the spawned
sensor uses the same `model://<gz_model>` include the asset generator
would emit.

Limitations (v1):

- Gazebo only. MuJoCo / Webots / Isaac return "not supported".
- The new sensor is parented to the world, not rigidly to the UAV body
  (would need a wrapper SDF + Gazebo `<joint>` rejoin to bind).
- The ros_gz_bridge YAML isn't auto-rewritten — for the new topic to
  surface in ROS 2, launch a `parameter_bridge` for it, or restart the
  bridge with an updated config.

## What's not done yet (v1 → v2 roadmap)

| Item | Status |
|---|---|
| Gazebo SDF generator (multirotor) | ✅ shipped |
| PX4 airframe generator (multirotor / VTOL standard / fixed-wing) | ✅ shipped |
| Catalog: 4 chassis (x500, x3_uav, standard_vtol, fixed_wing_basic) | ✅ shipped |
| Catalog: 10 sensors, 4 arms, 2 airfoils | ✅ shipped |
| Every existing UAV authored as a descriptor (11 UAVs) | ✅ shipped |
| Runtime sensor hot-swap (Gazebo) | ✅ shipped |
| URDF / ros2_control generator | 🟡 not yet (composed-arm Gazebo plugin is in the SDF template) |
| MJCF generator (MuJoCo) | 🟡 schema in place, no generator |
| Webots PROTO generator | 🟡 schema in place, no generator |
| USD generator (Isaac) | 🟡 schema in place, no generator |
| Tailsitter / tiltrotor PX4 generator | 🟡 schema in place, no generator |
| Runtime hot-swap: rigid-attach + bridge auto-rewire | 🟡 wrapper SDF + Gazebo rejoin TBD |
| Runtime hot-swap on non-Gazebo sims | 🔴 roadmap |
| Web GUI on top of the descriptor | 🔴 roadmap |
