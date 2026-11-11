# Aerial-arm flight experiments

The arm is for jobs that need a manipulator in the air: painting a wall or an
aircraft's body, cleaning nests from high buildings, precision maintenance on
a tower. These are contact jobs, so what matters is end-effector accuracy and
steady force against a surface.

Two experiments, both PX4 v1.15.4 SITL in Gazebo Harmonic:

- [Free flight](#free-flight-experiment) (30 Oct 2026): how much a moving arm
  disturbs a hovering x500 and how far off that puts the end effector.
- [Compensation](#disturbance-compensation) (1 Nov 2026): two feed-forward
  controllers that cancel most of it.
- [Contact](#contact-experiment) (1 Nov 2026): pressing a tool against a wall,
  under position control and under force control.
- [Holding the tool in the air](#holding-the-tool-at-a-point) (2 Nov 2026): RTK, and
  the arm cancelling the drone's error.

# Free-flight experiment

## Protocol

`ros2 run uavros2 aerial_arm_demo -p arm:=<arm>` (`uavros2/aerial_arm_demo.py`):

1. Wait for MAVROS, the local position and the arm's `controller_state`.
2. Stream a position setpoint 2 m above the take-off point at 20 Hz, switch
   to OFFBOARD and arm; climb.
3. **hover**: hold 8 s with the arm still.
4. Four arm phases, each a 2 s joint trajectory (`move_s`) then a 5 s hold
   (`hold_s`), sent to `joint_trajectory_controller/follow_joint_trajectory`:
   **reach_a**, **reach_b**, **fold**, **stow**. For the OpenManipulator-X:
   forward reach (end effector ~0.31 m ahead, arm CoM 0.13 m forward), the
   same reach yawed 90° to the side, the arm swung back under the body (CoM
   0.13 m behind the mount), and the stow pose it takes off and lands in.
   All four are free of self-collision, so MoveIt can plan between them.
5. AUTO.LAND, wait for disarm.

Logged at 50 Hz to `<arm>_flight.csv`: drone position (`mavros/local_position/pose`)
and setpoint, the arm's reference / measured joints and error
(`controller_state`), and the end effector's map-frame position (drone pose
composed with the `base_link -> <ee>` TF from robot_state_publisher).

Metrics per phase: horizontal RMS and max of the position error, mean
vertical error, joint tracking RMS, and **end-effector world error**: actual
EE position vs the setpoint composed with the forward kinematics of the
reference joints (where the EE would be if the drone held its setpoint
level).

`arm:=none` flies the same phases with nothing moving (baseline).

## Running it

```bash
export GZ_PARTITION=uavsim ROS_DOMAIN_ID=24          # isolate from other sims
scripts/aerial_arm_experiment.sh openmanip_x 4 ~/results/omx           # 2 s motions
scripts/aerial_arm_experiment.sh openmanip_x 4 ~/results/omx_slow -p move_s:=6.0
scripts/aerial_arm_experiment.sh none 4 ~/results/baseline
scripts/plot_aerial_arm.py docs/figures/aerial_arm \
    "x500 (no arm)=$HOME/results/baseline/none_flight.csv" \
    "OMX 2 s motions=$HOME/results/omx/openmanip_x_flight.csv"
```

The runner launches the sim headless on the given PX4 instance, waits for the
arm controller, flies the demo, stops the sim and sweeps anything left in its
partition; if PX4 gives up at startup (an occasional gz_bridge clock
timeout) it restarts the sim once. One run takes about 80 s. `UAV=<model>` flies another airframe
(e.g. `UAV=x500_d435` with `none`).

## Results

Gazebo Harmonic, `warehouse` world, PX4 v1.15.4, headless, PX4 instance 4.
Mean ± SD over repeated runs (n in the table). Raw logs:
`~/drone_arm_ws/results/aerial_arm/` (see PROJECT.md).

![Hover error per phase](figures/aerial_arm/hover_error_by_arm.png)

| run | n | phase | horiz RMS (cm) | horiz max (cm) | vert mean (cm) | EE RMS (cm) | joint err RMS (rad) |
|---|---|---|---|---|---|---|---|
| x500 (no arm) | 4 | hover | 5.1 ± 0.3 | 8.8 ± 1.7 | -5.4 ± 2.4 | - | - |
| x500 (no arm) | 4 | reach_a | 5.5 ± 0.7 | 9.2 ± 1.5 | -1.8 ± 6.0 | - | - |
| x500 (no arm) | 4 | reach_b | 5.1 ± 0.8 | 9.0 ± 1.8 | -0.8 ± 7.1 | - | - |
| x500 (no arm) | 4 | fold | 5.0 ± 1.1 | 9.2 ± 1.7 | 2.2 ± 7.5 | - | - |
| x500 (no arm) | 4 | stow | 5.8 ± 1.2 | 10.0 ± 2.1 | 0.5 ± 9.1 | - | - |
| OMX scaffold 0.16 kg | 1 | hover | 4.0 | 6.5 | -8.9 | - | 0.000 |
| OMX scaffold 0.16 kg | 1 | reach_a | 7.5 | 11.5 | -9.2 | - | 0.016 |
| OMX scaffold 0.16 kg | 1 | reach_b | 7.7 | 12.7 | -13.4 | - | 0.032 |
| OMX scaffold 0.16 kg | 1 | fold | 6.5 | 11.4 | -10.8 | - | 0.025 |
| OMX scaffold 0.16 kg | 1 | stow | 4.0 | 9.5 | 0.9 | - | 0.017 |
| OMX 2 s motions | 4 | hover | 6.5 ± 0.7 | 10.9 ± 1.8 | -5.2 ± 10.8 | 13.5 ± 4.3 | 0.000 ± 0.000 |
| OMX 2 s motions | 4 | reach_a | 10.6 ± 1.5 | 21.5 ± 3.3 | 0.2 ± 11.7 | 15.0 ± 2.4 | 0.030 ± 0.000 |
| OMX 2 s motions | 4 | reach_b | 17.7 ± 1.0 | 34.3 ± 1.6 | 0.1 ± 15.6 | 22.2 ± 4.0 | 0.019 ± 0.000 |
| OMX 2 s motions | 4 | fold | 24.8 ± 1.6 | 50.3 ± 4.6 | 0.8 ± 18.8 | 30.2 ± 4.0 | 0.036 ± 0.000 |
| OMX 2 s motions | 4 | stow | 13.5 ± 2.0 | 27.0 ± 4.3 | 1.3 ± 18.3 | 20.3 ± 6.2 | 0.034 ± 0.000 |
| OMX 6 s motions | 3 | hover | 5.4 ± 0.1 | 9.2 ± 1.4 | -8.6 ± 4.2 | 10.4 | 0.000 ± 0.000 |
| OMX 6 s motions | 3 | reach_a | 6.0 ± 0.7 | 10.5 ± 1.7 | -10.6 ± 11.0 | 9.2 | 0.014 ± 0.000 |
| OMX 6 s motions | 3 | reach_b | 10.1 ± 0.8 | 15.9 ± 1.4 | -10.3 ± 12.0 | 11.1 | 0.009 ± 0.000 |
| OMX 6 s motions | 3 | fold | 17.4 ± 0.9 | 29.3 ± 2.4 | -3.4 ± 10.4 | 17.5 | 0.017 ± 0.000 |
| OMX 6 s motions | 3 | stow | 8.5 ± 1.1 | 14.8 ± 3.0 | -1.5 ± 8.6 | 8.8 | 0.016 ± 0.000 |

The 6 s end-effector error is from one run: in the other two the demo's
TF lookup of the end effector failed for the whole flight (cause not
found; hover and joint data are complete). The demo now also logs the
attitude so the end effector can be recomputed offline.

Full tables: [summary.md](figures/aerial_arm/summary.md) (and
[scaffolds/summary.md](figures/aerial_arm/scaffolds/summary.md) for the
three_dof / Panda / UR5 scaffolds and the x500_d435 control, flown before its D435
was added back in November 2026).

### What it shows

- **Noise floor.** With nothing moving, PX4 holds the x500 within
  5.1 ± 0.3 cm horizontal RMS (peaks ~9 cm). Height wanders slowly by up to
  ±10-15 cm in every configuration, arm or not (compare the vertical SDs),
  so per-run vertical means say little.
- **The scaffold arms do not matter.** At 0.12-0.20 kg the three_dof,
  Panda-named and UR5-named scaffolds, and the old 0.16 kg OpenManipulator-X
  scaffold, stay within 1-3 cm of the baseline. They are fine for testing
  controllers and MoveIt, useless for studying coupling.
- **The real OpenManipulator-X does.** At 0.594 kg (30 % of the x500), a 2 s
  arm motion pushes the drone 21-50 cm off its setpoint at the peak and
  10.6-24.8 cm RMS over the phase, 2-5 times the noise floor; PX4 brings it
  back within a few seconds of the motion ending (see the time series). The
  backward reach (the "fold" slot: the arm swings from the side to 0.13 m
  behind the mount) is the worst, then the side reach; even returning to
  stow from the backward reach costs 13.5 cm RMS.
- **The end effector misses by more.** During the moves the end effector is
  15-30 cm RMS from where it should be in the world, so a fast arm motion
  cannot place the gripper precisely without compensation.
- **Slower motion helps a lot.** Moving the arm over 6 s instead of 2 s
  cuts the peak excursion to 10-29 cm and the phase RMS to 6-17 cm (the RMS
  is over a longer phase; the peak is the fairer comparison). Most of the
  disturbance is the reaction to accelerating the arm; the static
  centre-of-mass shift (up to 0.13 m of 0.594 kg) is absorbed by PX4's
  integrators within seconds.
- **Joint tracking is not the problem**: 0.009-0.036 rad RMS, all goals
  succeed.

![Real OpenManipulator-X, 2 s motions](figures/aerial_arm/omx_2_s_motions_flight.png)

### Height reference note

On 30 Oct the generated airframes used the barometer as EKF2 height reference
(`EKF2_HGT_REF 0`); stock x500 (4001) uses GPS height. Flying x500_d435 (no arm)
against x500 (4001) showed the same ±10 cm height wander for both, so it is not caused
by the arm or by that setting. Since 1 Nov the arm airframes (4425-4429) use GPS height.

### Earlier pose set

A first round flew the OpenManipulator-X with a stow pose that MoveIt found
self-colliding and a compact fold instead of the backward reach (raw data
in `results/aerial_arm/omx_v1/`). Its forward/side-reach numbers agree with
the table above (13-20 cm RMS at 2 s, 6.5-11 cm at 6 s).

# Disturbance compensation

The main disturbance is the moment of the arm's centre of mass: `r x m g`, about
0.76 N m at the forward reach. PX4's rate integrator absorbs it in about a second,
and until then the drone tilts and accelerates. Both controllers feed that moment
forward, plus the reaction of accelerating the arm. Both use the arm's model (from
the URDF) and `joint_trajectory_controller`'s reference. The gains come from PX4's
default parameters and the motor model, not from tuning against these results.

- **Outer loop, PX4 unchanged** (`-p controller:=outer_ff`,
  `uavros2/arm_aware_control.py`). Replaces PX4's position controller with the
  same gains (`controller:=outer` is that loop alone) and sends attitude + thrust
  in OFFBOARD. An attitude setpoint cannot carry a torque, so the moment becomes
  an attitude bias through a washout that matches PX4's integrator (0.75 s). The
  reaction force becomes an acceleration.
- **PX4 patch** (`-p controller:=px4_ff`,
  [config/px4/patches](../config/px4/patches)). PX4 position control stays; a small
  change to `mc_rate_control` adds a torque sent as MAVLink `DEBUG_VECT` "ARMFF"
  to the rate controller's output, gain `MC_ARM_FF_GAIN`. It stops 0.5 s after the
  last message. With no messages it changes nothing: one flight on the patched
  build without the torque gave the unpatched numbers (peaks 23 / 36 / 49 / 24 cm).
  The patch is local (`asmbatati/arm-ff` in PX4-Autopilot), not for upstream:

```bash
git -C PX4-Autopilot am ros2_ws/src/uavros2/config/px4/patches/*.patch   # or git apply
make -C PX4-Autopilot px4_sitl
scripts/aerial_arm_experiment.sh openmanip_x 4 ~/results/ff -p controller:=px4_ff
```

## Results

Same protocol, model and world as the free-flight experiment, re-flown on 1 Nov
2026 with GPS height on the arm airframes (`EKF2_HGT_REF 1`, the owner's decision;
the earlier barometer flights gave the same horizontal picture). Other sessions'
simulations were running (load 30-60 on 32 cores), and starts that failed to arm were
re-flown. Controls at load 9-17 gave the same numbers within the spread (PX4 position
peaks 23 / 34 / 49 / 26 cm, outer loop 13 / 14 / 14 / 10 cm), so load did not bias them.

All these numbers are errors against the **EKF's estimate**, what the controllers
see. The contact flights below measured the truth too, and the estimate itself is
3-16 cm off.

![Peak hover error per arm phase](figures/compensation/compensation.png)

Horizontal error, RMS / peak (cm):

| Motions | Controller | n | hover | forward | side | back | stow |
|---|---|---|---|---|---|---|---|
| 2 s | PX4 position | 4 | 5.8 / 9.6 | 8.6 / 16.7 | 21.8 / 40.2 | 26.9 / 51.8 | 16.2 / 30.3 |
| 2 s | **Outer loop + feed-forward** | 4 | 5.7 / 10.1 | 6.8 / 11.2 | 7.7 / 13.4 | 7.8 / 15.0 | 7.9 / 12.8 |
| 2 s | **PX4 + torque patch** | 4 | 5.6 / 9.4 | 5.3 / 8.5 | 6.5 / 10.3 | 7.2 / 13.2 | 5.9 / 10.5 |
| 6 s | PX4 position | 3 | 6.2 / 10.7 | 6.3 / 11.1 | 12.1 / 19.9 | 18.1 / 29.9 | 7.8 / 14.2 |
| 6 s | **Outer loop + feed-forward** | 3 | 6.9 / 12.2 | 6.3 / 11.2 | 6.4 / 11.6 | 8.7 / 14.3 | 6.3 / 10.3 |
| 6 s | **PX4 + torque patch** | 3 | 5.1 / 9.0 | 6.2 / 10.1 | 5.8 / 10.3 | 6.9 / 11.5 | 6.5 / 11.6 |
| - | No arm, PX4 position | 4 | 5.1 / 8.8 | 5.5 / 9.2 | 5.1 / 9.0 | 5.0 / 9.2 | 5.8 / 10.0 |
| - | No arm, outer loop | 3 | 5.5 / 9.7 | 5.9 / 10.3 | 5.6 / 9.8 | 5.3 / 8.8 | 5.8 / 11.1 |

The outer loop alone, without feed-forward, matches PX4 position control
(barometer flights, 2 s: peaks 22 / 43 / 59 / 30 cm).

**One crash, not in the table.** One 2 s PX4-patch flight of 16 patch flights fell
during the side swing: front motor at 1.00, thrust setpoint at its limit, 1.4 m
lost in about 2 s. It is the same thrust-margin failure as in contact, on the
stock 8.5 N motors (loaded thrust-to-weight 1.26). None of 15 outer-loop flights
and none of the PX4 position flights fell, but the sample is too small to say the
patch is the cause.

Spread and height in [summary.md](figures/compensation/summary.md).

## What it shows

- **Most of the disturbance is the predictable moment, and both controllers
  remove most of it.** At 2 s the worst peak falls from 52 cm to 15 cm (outer loop)
  or 13 cm (patch), against a no-arm floor of about 10 cm. The patch at 2 s beats PX4
  position at 6 s.
- **The PX4 patch is a little better horizontally.** It gives a true torque
  immediately, and PX4's pitch integrator barely moves (within +-0.04, where the
  moment alone would need about 0.16).
- **The outer loop holds height much better**: 2-4 cm RMS, against 7-20 cm for
  PX4's own height control, with or without the patch. So the +-10-15 cm wander
  reported above comes from PX4's height control, at least in part.
- **GPS height did not help height hold** under PX4's own control (7-20 cm, about
  the same as the barometer's).
- **End-effector error** at 2 s: from 11-28 cm RMS (PX4 position) to 7-9 cm (outer
  loop) or 10-13 cm (patch).

The arm's reference joints and mass are known exactly here. A real arm's model
error and a real link's latency will reduce this gain; neither was tested.

# Contact experiment

`wall_contact_demo` (`uavros2/wall_contact_demo.py`) flies
`x500_with_openmanip_x_tool` in the `wall` world. The tool is a 0.50 m rod
(0.22 kg with its 0.1 kg head) with a force/torque sensor at its base
(`/<ns>/tool/wrench`), standing in for a roller or brush. Its head is 0.20 m wide and
touches at two points; a passive yaw pivot at the rod's base and a damped spring along
the rod make it compliant. Those came from the failures below: the first version was
one rigid ball. The drone model has
13 N motors; **these are a modelling assumption, not a real part.**

1. Take off, hover, and move the arm to the contact pose: rod level, tip 0.51 m
   ahead of `base_link`, 0.21 m past the propeller tips.
2. Fly to 0.20 m from the wall and hold 8 s; the hold tares the sensor.
3. Creep forward at 5 cm/s until the sensor reads 0.5 N.
4. Press for 20 s. Under plain position control, move the setpoint `press_depth`
   past where the drone is. With `-p force_target:=<N>` under the outer loop, use
   **hybrid force control**: along the wall normal the thrust component commands
   the force directly (the target plus an integral on the measured force, velocity
   damping, and a 5 cm/s creep 0.1 s after contact is lost), while position control
   holds the other two axes.
5. Retract, stow, fly back, land.

```bash
export GZ_PARTITION=uavsim ROS_DOMAIN_ID=24 PYTHONNOUSERSITE=1
export UAVROS2_RTK=1   # needs the PX4 patches in config/px4/patches
UAV=x500_with_openmanip_x_tool WORLD=wall DEMO=wall_contact_demo \
  scripts/aerial_arm_experiment.sh openmanip_x 4 ~/results/contact \
  -p controller:=outer_torque -p nm_per_normalized:=5.88 -p hover_thrust:=0.67 \
  -p press_depth:=0.02 -p force_target:=3.0 -p press_s:=60.0
scripts/plot_wall_contact.py docs/figures/wall_contact "13 N, press 5 cm=$HOME/results/contact*"
```

## First results: rigid tool

All on the 13 N motors. Statistics are over the last 17 s of the press, for every
run that reached contact, including the part before any fall. "Tool SD" is the
estimated tool position along the wall. All groups:
[all_groups.json](figures/wall_contact/all_groups.json). Raw logs:
`~/drone_arm_ws/results/compensation/` and `results/contact/`.

| Tool | Control | Runs | Fell | Force median (N) | Force IQR (N) | Contact lost (%) | Tool SD y / z (cm) | Yaw SD (deg) |
|---|---|---|---|---|---|---|---|---|
| Ball | PX4 position, 5 cm | 3 | 0 | 1.1 | 1.9 | 26 | 8.0 / 8.7 | 8.4 |
| Ball | PX4 position, 15 cm | 3 | 1 | 2.0 | 3.5 | 29 | 16.9 / 15.0 | 11.2 |
| Ball | PX4 position, 30 cm | 3 | 3 | 3.1 | 3.1 | 1 | 19.9 / 8.0 | 25.7 |
| Ball | Outer loop + feed-forward, 5 cm | 3 | 0 | 1.2 | 0.7 | 12 | 8.7 / 5.7 | 5.3 |
| Ball | PX4 + patch, 5 cm | 3 | 0 | 0.9 | 1.2 | 42 | 7.6 / 2.9 | 7.1 |
| Ball | PX4 + setpoint admittance, 3 N | 3 | 1 | 1.5 | 2.1 | 24 | 12.8 / 12.1 | 15.3 |
| Ball | Hybrid, 3 N | 4 | 3 | 2.9 | 3.7 | 8 | 20.5 / 9.2 | 23.3 |
| Ball | Hybrid, 6 N | 3 | 3 | 6.2 | 5.3 | 5 | 53.9 / 14.9 | 69.0 |
| **Head** | PX4 position, 5 cm | 3 | 0 | 2.3 | 2.6 | 26 | 6.3 / 7.0 | 6.2 |
| **Head** | PX4 + patch, 5 cm | 3 | 0 | 0.9 | 1.6 | 46 | 3.8 / 5.7 | 2.5 |
| **Head** | **Hybrid, 3 N** | 3 | 1 | 2.8 | 2.4 | 3 | 10.9 / 5.9 | 13.8 |
| **Head** | Hybrid, 6 N | 3 | 3 | 6.3 | 5.4 | 7 | 10.7 / 11.6 | 10.2 |

Stock 8.5 N motors, ball, 5 cm: one of two runs fell, median under 1 N (commit
a45c348). Falls happened after 1.7-19 s. Three runs per group is few, so read
this as rough.

What the rigid tool showed:

- **Force control does what position control cannot.** Hybrid control reaches its
  target (median 2.8-2.9 N at 3 N, 6.2-6.3 N at 6 N) and stays in contact 92-97 % of
  the time, against about 74 %. Under position control the force is whatever the
  position error and PX4's integrator happen to produce, and it creeps up.
- **Moving a position setpoint does not regulate force** (admittance: median 1.5 N
  for a 3 N target), at least with these gains.
- **Free-flight feed-forward alone does not help contact.** Both controllers stay
  in the air at 5 cm, but contact is no better; the patch loses contact most often,
  though its tool wanders least.
- **The limit at higher force is yaw.** See below.

## Why the head is wide and the rod pivots

Every hybrid fall with the single ball was preceded by yaw running away (-19, -35,
-22 deg). A rigid rod pushed against a wall buckles sideways like a pushed stick: at
yaw `psi` the force `F` through the tip makes a yaw moment `F L sin(psi)`, which grows
with the error. With F = 3 N and L = 0.5 m that is about 1.5 N m/rad, while a
quadrotor's yaw authority here is only about 0.4 N m (rotor drag torque), so control
goes at about 15 deg, as seen. This estimate comes from the logs and the geometry;
no separate yaw experiment was run.

Three fixes, tried in this order:

1. **Two contact points** 0.20 m apart give a restoring moment within about +-11
   deg, which is also what a real roller has. 3 N went from 3 falls in 4 to 1 in 3,
   and yaw spread from 23 to 14 deg, but 6 N still fell.
2. **Arm's yaw joint keeping the tool square to the wall: worse.** With the tip
   gripped by friction, a stiff joint turns any sideways drift into yaw torque on the
   body (-36 to -65 deg in the one flight, then stopped). This is still in the demo
   as `yaw_servo`.
3. **A passive yaw pivot at the rod's base.** A pin cannot carry a moment, so the
   force reaches the arm only 0.10 m ahead of the body, not 0.50 m: about 0.3 N
   m/rad, inside the yaw authority. No fall at 3 N in 20 s runs (6 of 6), yaw SD 1.4-5
   deg.
4. **The pivot on the body's yaw axis.** Folding the arm and lengthening the rod to
   0.50 m puts the pivot over the mount, so the force has no yaw lever, and the arm and
   rod balance. With a softer, longer spring at the head (80 N/m, +-8 cm: the first,
   300 N/m, turned small position wobbles into force swings), that was the tool until
   B18, which found the simulated spring much softer than set (next sections).

## Position source: RTK through PX4's EKF

Measuring against Gazebo's truth showed the EKF's height off by 6-16 cm (SD) while it
reports itself steady. The cause is the simulated receiver: PX4's `sensor_gps_sim` adds
0.2 m horizontal and 0.5 m vertical white noise to every sample. No controller holds 2
cm on that.

`UAVROS2_RTK=1` (airframe 4429, local PX4 patch 0002) switches the simulated receiver to
RTK-grade noise (1.5 cm, 2 cm, fix type 6, 5 Hz) and tells the EKF. PX4's EKF and every
controller run unchanged, so latency and filtering are real. The estimate's error falls
to about 1 cm against truth. **It is a noise model, not a full RTK receiver**: no
multipath or lost fix near a wall, which a real one would have.

## Results: passive tool and hybrid force control, against truth

Gazebo's true head position is logged for measuring only. Controller `outer_torque`
(outer loop with the PX4 torque patch), 3 N target, load 2-20, real-time factor 0.94-1.0.
Per-run numbers, including earlier tool versions:
[pivot_runs.txt](figures/wall_contact/pivot_runs.txt).

| Position source, press | Runs | Fell | Force median (N) | Force IQR (N) | Contact lost (%) | Tool SD y / z (cm) | Within 2 cm (% of time) |
|---|---|---|---|---|---|---|---|
| EKF on plain GPS, 60 s | 3 | 0 | 1.9-2.2 | 1.8-2.4 | 13-20 | 5-12 / 13-26 | 0 |
| RTK, 20 s | 3 | 0 | 2.1-2.9 | 0.6-1.5 | 0-13 | 0.1-2 / 0.4-3 | 12-100 |
| **RTK, 60 s** | 10 | 1 | 1.3-2.7 | 0.8-2.5 | 0-31 | 0.2-22 / 2-6 | 3-84 |

In the RTK 60 s group, the fall came at 46.6 s (its truth log is missing, so 9 runs
give the other columns). Seven of those 9 have a force median of 2.2-2.7 N and tool SD
of 4.5 cm or less; one had a yaw excursion (SD 21 deg) and drifted 22 cm.

- **The target (a set force for 60 s, 10 of 10 runs, tool within 2 cm) is not met,** but
  it is close at the best: 2.85 N, IQR 0.6 N, tool SD 0.1 / 0.4 cm, 100 % within 2 cm
  over 20 s; 2.72 N, 84 % within 2 cm, over 60 s. Other runs slip and resettle.
- **The position source was the main limit, as suspected.** Tool height SD: 13-26 cm on
  plain GPS, 2-6 cm on RTK, with the same controller and tool.
- **Force settles a little under target** (2.2-2.9 N for 3 N). Short loss of contact pulls
  the median down.
- **6 N is not held**: 2 of 3 fell (9-10 s) even with the centred pivot. This was put down
  to pitching over; B18 found otherwise (sideways instability of the rod, next section).
  The force acts 0.23 m below the centre of mass of drone and arm (0.28 m below
  `base_link`).
- Short force spikes (20-110 N, single samples) were the head hitting its travel stop
  (next section).

## Steadier contact (B18)

Per-run numbers of every group below: [b18_runs.txt](figures/wall_contact/b18_runs.txt).
Same protocol, RTK, 3 N, 60 s.

| Tool and force loop | Runs | Fell (after) | Force median (N) | Force IQR (N) | Contact lost (%) | Tool SD y / z (cm) | Within 2 cm (%) |
|---|---|---|---|---|---|---|---|
| Before: 20 g head, PI loop | 10 | 1 (46.6 s) | 1.29-2.73 | 0.81-2.49 | 0-31 | 0.2-22.4 / 1.7-6.0 | 3-84 |
| Damped head, new loop | 5 | 0 | 2.52-3.09 | 0.89-3.60 | 1-13 | 1.4-4.8 / 1.1-3.5 | 9-73 |
| **Same, plus the hover integral (defaults now)** | 10 | 2 (26.7, 44.0 s) | 2.96-3.11 | 0.66-1.23 | 0-3 | 0.1-3.8 / 1.4-3.1 | 1-92 |
| Damped head, new loop, **6 N** | 3 | 3 (7.8-9.0 s) | - | - | - | - | - |

Ranges are over the runs that held 60 s. The force now sits on its target (medians within
0.11 N of 3 N, against 0.3-1.7 N short before) and contact is lost 0-3 % of the time
instead of up to 31 %. The 10 of 10 target is still not met: 2 runs fell after a yaw
excursion, and the tool's height wanders 1.4-3.1 cm (SD), mostly the body's pitch times
the 0.5 m rod.

![Force against time, 60 s press, 3 N target, before and after B18](figures/wall_contact/contact_force.png)

What changed, in the order found:

1. **The simulated spring was far softer than set.** Gazebo's solver does not give a
   prismatic joint its `spring_stiffness`: a light head settles deeper, by an amount that
   depends on its mass and damping. `scripts/bench_tool_head.py` measures it (a 2.8 kg
   carriage on a slide, 3 N on the head, dropped 4.5 cm):

   | Head (kg) | Damping set (N s/m) | Stiffness set (N/m) | Measured (N/m) | Overshoot |
   |---|---|---|---|---|
   | 0.02 | 6 | 80 | 35 | 94 % |
   | 0.10 | 10 | 80 | 56 | 75 % |
   | **0.10** | **50** | **150** | **48** | **16 %** |

   The old 20 g head therefore reached its 8 cm stop at 3 N and spent much of each press on
   it, the stop's impacts were the 20-110 N spikes, and 50 of 87 contact losses
   came within 1 s of one. The head is now 0.1 kg with 150 N/m and 50 N s/m set (48 N/m,
   16 % overshoot), and +-15 cm of travel (6 N at 12.5 cm).
2. **Spikes went into the torque feed-forward.** The controllers now take the median of
   the last 0.1 s of force, capped at twice the target.
3. **A soft head bounced.** With a real 50 N/m the drone hit the head, was thrown off, kept
   pushing at the target force while off the wall and came back at about 0.4 m/s, about
   once a second. The force loop now creeps at 5 cm/s 0.1 s after a loss, and has no
   proportional term (through the attitude loop's lag it fed the bounce) and more velocity
   damping (3 instead of 1.8 per second); the damped head absorbs the impact.
4. **The contact moment is taken about the combined centre of mass** (the tip is 0.23 m
   below it; 0.28 m was below `base_link`), and with the body's pitch, which tilts the wall's force in the body
   frame.
5. **The position loop's integral is kept under force control.** Holding position needed
   1-2 deg of lean (attitude and thrust biases), held by the x integral. Freezing and
   dropping it under force control left the creep law to balance that bias, at 4 cm/s
   away from the wall: in one batch 7 of 8 runs drifted off after the first bounce and
   never came back.

**6 N: the rod is a strut.** With the head held by friction, a sideways drift `d` leans
the rod and its compression pushes the drone further by about `F d / L`: 12 N/m at 6 N
with L = 0.5 m, against 4.8 N/m from the hover position gains and 2 N/m from the pivot's
spring. At 3 N it is marginal (6 against 6.8 N/m), which fits the sideways tool wander and
the two yaw falls. Three fixes failed:

- **Cancelling the measured sideways push** (the sensor reads it): worse, roll SD 2-3 deg
  against 1.2-1.5, and 3 of 5 runs fell at 3 N. The measured force is too noisy and late to
  feed forward at 50 Hz. (The sensor's axes, found from flights: the push on the drone is
  `(fx, fy, -fz)` in body axes.)
- **Stiffer cross-wall position gains** (8 per second squared): roll oscillated, SD 10-14 deg.
  The attitude loop is too slow for them.
- **A stiffer pivot spring** (3 N m/rad, 12 N/m at the head): yaw SD 3.6-7 deg against
  1.5-2.3 and 3 of 5 falls at 3 N; the spring passes yaw moment to the body.

So 6 N needs restoring stiffness across the wall that does not go through the attitude
loop and does not twist the body, or a head that does not pin the rod sideways.

**The two falls at 3 N start with a bounce.** In both, the force swung +-1.5 N about the
target and dipped to zero once. The loop then crept back and hit the wall at about 8 N.
The head slipped 5-7 cm sideways, and the drone spun or rolled away within a second.
Tolerating a longer loss (0.3 s instead of 0.1 s) and creeping slower (2 instead of
5 cm/s) is worse. 6 of 10 runs fell, two of them before contact settled (3.7 and 4.7 s).
With the B18 settings, 3 of 15 fell (2 of 10 in B18 and 1 of 5 in a control batch flown
alongside). The prompt creep stays: an unloaded head drifting along the wall is the
worse case.

## Holding the tool at a point

`tool_hold_demo`: 25 s hover with the arm still, then 25 s with the arm correcting the
drone's error in the air (damped least squares on the tool position, from the estimated
pose, smoothed over 0.5 s). Truth-measured scatter about the mean, RMS (cm), n=3:

| Position source | Arm still | Arm correcting | Within 2 cm, still / correcting |
|---|---|---|---|
| EKF on plain GPS | 13-24 | 10-16 | 0-1 / 0 % |
| RTK | 2.0-2.7 | 3.4-3.8 | 25-61 / 13-24 % |

The target (tool within 2 cm) is not reached. **On RTK the correction makes the tool
worse**: the arm's own motion pushes the drone, and that costs more than it corrects. On
plain GPS it helps, because the error is large and slow; most of that error is the
estimate's own, which no correction based on it can remove. A better position source is
worth far more than correcting with the arm. Without smoothing the RTK figure was
4.2-5.3 cm.

## Failed starts

About half of the starts failed to arm for a while (`could not enter OFFBOARD + arm`, or
no local position), and it looked like lack of GPS. Checked: 18 of 18 starts at load 3-20
were fine (6 with PX4's GPS checks passing at 50 s; 12 armed in the real demo), and the
failures only came at load 30-68, with other simulations on the machine. So it is
machine load, not GPS. The exact mechanism was not isolated, since it did not reproduce.
Failed starts leave no flight data, and controls at low load match the earlier numbers.
Flight summaries now record the real-time factor and load, and a start that fails is
re-flown by the batch script.

## Why the drone fell, and the design changes it forced

1. **Arm at full reach (0.20 m rod): fell on first contact.** The PX4 log shows
   the front motors at 0.99 in free hold, because the arm and rod put
   0.122 kg m ahead of the body. On contact the attitude controller took the
   remaining authority out of collective thrust: thrust setpoint at 1.0, motors
   falling, 1.8 m to the ground in about 2 s. Fix: the 0.40 m rod with the arm
   folded close in, 0.022 kg m. Front motors drop to about 0.90.
2. **Stock motors with the long rod: fell at 2-3 N, or under 1 N in the run
   with the corrected press target.** The x500 model has 8.5 N per rotor. Loaded
   (2.78 kg, 27 N) that is a thrust-to-weight of 1.26 and about 1.7 N of margin
   per front rotor. The tip sits 0.244 m below `base_link`, so pushing makes a
   nose-down moment that uses up that margin quickly.
3. **13 N per rotor (thrust-to-weight 1.9): holds at 5 cm in 3 of 3 runs.** The
   tool model uses this. **The 13 N motors are a modelling assumption, not a real
   part**: the plugin's motor constant is simply raised. A real build should aim
   for a thrust-to-weight of at least 2. The stock-motor runs are from commit
   a45c348.

Two more things seen here:

- With a position setpoint moved past the wall at 5 cm, the first version pressed
  15-20 cm too deep because the setpoint leads the drone while creeping; the demo
  now presses from where the drone is.
- The EKF's local origin is up to about 0.2 m from the spawn point, so the demo
  measures the wall's distance from its take-off position, and contact has to be
  detected by force, not assumed from position.
- After landing, some contact flights show estimated height drifting to -1 to
  -3 m with the drone on the ground (barometer height reference, decision 3 in
  ROADMAP.md). Cause not investigated.

Limits of this model: the machine was loaded (30-60 on 32 cores) by other
simulations during the earliest of these flights, and the arm's joints are stiff
position-controlled joints; the tool's only compliance is its head's spring and damper.
A real arm would soften the spikes, so the spikes here are likely an upper bound. Contact is
balls on a flat wall with one friction value (0.5, high for a roller), and there is
no rotor wash near the wall (ground and wall effects are not simulated).

## What would come next

- The rod's sideways instability (B18): restoring stiffness that does not go through
  the attitude loop, for 6 N and for the yaw falls left at 3 N.
- A camera on a marker or the wall as the position source, and a fuller RTK model.
- Trajectory shaping: minimum-jerk arm trajectories under a hover-error budget.
- A pick that holds: see the next section.

# Pick and place from hover (B11)

The OpenManipulator-X now has its gripper: two fingers on prismatic joints, the right
one the mimic of the left, driven by `gripper_controller`
(`parallel_gripper_action_controller`). ROBOTIS's frames, limits and 1 g fingers;
4.43 cm pads 2.0 cm apart closed and 7.8 cm open. The `pick` world has two posts with
tops 1.0 m up and a 50 g part (4 x 3 x 5 cm, 3 cm across the fingers) on the first.
`pick_demo` should carry it to the second post. It does not yet: in 22 flights on
simulated RTK no grasp held.

    UAVROS2_RTK=1 WORLD=pick DEMO=pick_demo \
        scripts/aerial_arm_experiment.sh openmanip_x 4 results/pick -p controller:=outer_ff

How it flies:
1. It hovers with the gripper pointing straight down under the body, where the arm
   moves its centre of mass least (`omx_gripper_down_ik`: 0.32-0.48 m down).
2. It ties the world frame to its estimate at take-off: the posts' places are known,
   as on a surveyed site.
3. It finds the post's top by touch beside the part.
4. Over the part it lowers the open fingers until the tips stall on the post, backs up
   2 cm and closes.
5. If a pad lands on the part, or the fingers close on nothing, it tries again
   (3 tries).

Nothing senses the part. Gazebo's truth of the drone, its links and the part is logged
only to measure.

Why the grasp misses, measured against truth:

- **The gripper's tolerance is +-2.4 cm across the fingers**, and about +-3 cm along
  them before a pad misses the part.
- **The estimate is off by more than that.** After the take-off tie, the EKF's position
  is 1.0-1.2 cm SD from truth per axis, and its 2 s average swings by 4-5 cm within a
  flight. For 4 s before one grasp the estimate put the gripper within 1 cm of the part
  while it was 4-5 cm off. A single pose sample at take-off was up to 3 cm off (now a
  5 s average).
- **The hover wanders 1.3-2.2 cm SD** (truth) at 0.2-0.5 Hz, so even a well-timed
  descent moves 2-4 cm in its 1.5 s.
- **The height estimate reads 3-4 cm low after take-off** and drifts about 2 cm in 30 s,
  against a +-2 cm vertical tolerance (pads on the part's top 3 cm, tips 2 cm above the
  post). The touch-off fixes this: it measured the post -1.7 to +8.5 cm from where planned.
- **A touch hardly loads the wrist.** The drone gives way upwards, so the tips resting
  on the post change the wrist force by 0.2-0.3 N, as much as the arm's own motion.
  Contact is found instead by the tips stalling while the arm still lowers them. In
  the servoed descent this also fires when the arm lags or hits its reach, and some
  "pad on the part" retries were such false stalls.
- **Correcting with the arm makes the drone worse**, as for the tool (B16). With the arm
  cancelling the estimated error fore and aft and vertically, and sideways through
  joint 1 (`reach_x:=0.07`), the corrections pushed the drone, which the correction then
  chased. The gripper ended up 5-13 cm off and the servo at its limits.

Flight notes. This drone and arm hover at 0.85 thrust on the stock motors, so there is
little margin. Giving the outer loop the airframe's 0.72 `MPC_THR_HOVER` left the
integrator to make up the rest, wound up near saturation and crashed one flight. About
1 start in 4 never received the arm's TF, so the demo computes the gripper's place from
`/joint_states` and the URDF.

What would make it hold: a relative position source for the last 10 cm (a camera on
the part or a marker by it), or a gripper with a wider capture than the estimate's
error, for example longer, flared fingers.

## The marker approach (started)

The owner chose a fiducial marker beside the part, read by a camera on the arm, and
the ROBOTIS gripper unchanged. Built so far:
- `models/pick_marker`: ArUco DICT_4X4_50 id 0, 4.4 cm, flat on the first post 5.25 cm
  ahead of the part, visual only.
- A wrist camera on link5 (640 x 480, 15 Hz, 86 deg), 3 cm to the marker's side of the
  gripper axis and looking along it. It renders headless like the D435. It is bridged
  to `/<ns>/wrist_camera/*`.
- `uavros2/marker_locator.py`: detection and `solvePnP` on the square's corners;
  within 2 mm sideways on synthetic images under OpenCV 4.6 and 4.11.
- `pick_demo` logs the part's place relative to the finger tips as the camera sees it
  (`marker_d*`) and keeps sample frames. It does not fly on it yet.

First flights, against truth: the part's offset from the tips is within about 1 cm
sideways (biases -1.4 to +1.0 cm, scatter 0.2-1.2 cm by phase) and reads about 3 cm
short in height. The gripper's own visual ball hid the marker until it was taken out
of the Gazebo model. Detections are sparse (a few per phase at 5 Hz), which is still to
explain. In the same flights the arm swinging down in 6 s saturated the motors and
dropped the drone 27 cm once; it now takes 10 s. One of these flights lifted and
carried the part (on the estimate, second try) and set it down 2.9 cm from the second
post's centre. It lay on its side, because the place's contact test fired on the
swinging part's force and let go 8 cm up; the place now needs a 1.5 N push.

Flying on it (flights try13-19, 3 each):
- **The camera reads the part against the tips within 0.6-1.1 cm** (x, y and height,
  against truth).
- **What it took:**
  - The detector had dropped the marker's square as a near-duplicate of its white
    card's outline; it now retries with nearer candidates allowed.
  - It takes the largest square within 5-40 cm; one wrong square had read 40 cm deep.
  - The marker lies on the post, so the part's centre is 2.5 cm above it.
- **How the demo uses it:**
  - It trims the drone over the part, height included, on the camera. With the marker
    in view this replaces the touch-off, which can knock the part.
  - It waits for the camera to see the tips within 1.5 cm.
  - It lowers the arm by the height the camera measures, in one move.
  - It closes only if the camera sees the tips within 2.2 cm across the fingers and
    2.5 cm along them. At the bottom the part can hide the marker; the last sighting
    within 1 s stands in.
- **The outer loop steps on simulated time now.** Simulator stalls of 1-2 s (wall
  clock, with simulated time stopped) preceded every thrust saturation in one batch.
- **No grasp has held yet.** The drone wanders about 2 cm in the second after the
  camera's go, which leaves the closing fingers at the edge of the gripper's +-2.4 cm.
  Attempts that miss push the part, which the camera, reading the fixed marker, does
  not see. Drops at the stock motors' limit still end some flights.
