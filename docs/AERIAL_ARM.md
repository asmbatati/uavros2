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
- [Holding the tool in the air](#holding-the-tool-at-a-point) (2 Nov 2026): the arm
  cancelling the drone's error.

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
three_dof / Panda / UR5 scaffolds and the x500_d435 control).

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
simulations were running (load 30-60 on 32 cores), and starts where the EKF never
got GPS were re-flown.

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
`x500_with_openmanip_x_tool` in the `wall` world. The tool is a 0.40 m rigid rod
(0.12 kg) with a force/torque sensor at its base (`/<ns>/tool/wrench`), standing in
for a roller or brush. Its head is 0.20 m wide and touches at two points; a passive
yaw pivot at the rod's base and a spring along the rod make it compliant. Those came
from the failures below: the first version was one rigid ball. The drone model has
13 N motors; **these are a modelling assumption, not a real part.**

1. Take off, hover, and move the arm to the contact pose: rod level, tip 0.50 m
   ahead of `base_link`, 0.20 m past the propeller tips.
2. Fly to 0.20 m from the wall and hold 8 s; the hold tares the sensor.
3. Creep forward at 5 cm/s until the sensor reads 0.5 N.
4. Press for 20 s. Under plain position control, move the setpoint `press_depth`
   past where the drone is. With `-p force_target:=<N>` under the outer loop, use
   **hybrid force control**: along the wall normal the thrust component commands
   the force directly (PI on the measured force, velocity damping, a 5 cm/s creep
   if contact is lost), while position control holds the other two axes.
5. Retract, stow, fly back, land.

```bash
export GZ_PARTITION=uavsim ROS_DOMAIN_ID=24 PYTHONNOUSERSITE=1
UAV=x500_with_openmanip_x_tool WORLD=wall DEMO=wall_contact_demo \
  scripts/aerial_arm_experiment.sh openmanip_x 4 ~/results/contact -p press_depth:=0.05
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
   deg. That is the tool now, with a spring at the head (300 N/m, damped).

## Results: passive tool and hybrid force control, measured against truth

Gazebo's true head position is logged for measuring only. Controller `outer_torque`
(outer loop with the PX4 torque patch), 3 N target. "1 cm position" flies on Gazebo's
truth plus 1 cm noise, standing in for RTK or a visual marker: it shows what a better
sensor would give, and **is not a real sensor**. Per-run numbers:
[pivot_runs.txt](figures/wall_contact/pivot_runs.txt).

| Position source, press | Runs | Fell | Force median (N) | Force IQR (N) | Contact lost (%) | Tool SD y / z (cm) | Within 2 cm (% of time) |
|---|---|---|---|---|---|---|---|
| EKF, 20 s | 3 | 0 | 1.6-2.0 | 2.3-2.9 | 19-32 | 8-14 / 7-12 | 0-1 |
| 1 cm, 20 s | 3 | 0 | 2.6-3.0 | 0.5-1.6 | 0-22 | 0.1-6 / 2-6 | 0-75 |
| EKF, 60 s | 5 | 1 | 1.2-2.3 | 2.4-2.8 | 15-42 | 6-19 / 6-27 | 0-1 |
| 1 cm, 60 s | 5 | 1 | 1.2-3.1 | 0.5-2.9 | 4-36 | 2-7 / 3-9 | 0-73 |

At a 6 N target on the 1 cm position (20 s, n=3): one fall after 14.9 s, force median
0.7-4.3 N, yaw SD up to 43 deg. 6 N is not held.

![Force against time, 60 s press, 3 N target](figures/wall_contact/contact_force.png)

- **Yaw buckling is solved at 3 N** by the passive pivot; at 6 N it is not.
- **The sensor is now the main limit.** The tool's true height wanders 6-27 cm on the
  EKF and 3-9 cm on 1 cm position. The EKF's height is off from truth by 6-16 cm (SD)
  while it reports itself steady. A real job needs better than GPS-plus-baro near the
  wall: RTK, or a marker or the surface itself seen by a camera.
- **The target (a set force for 60 s, 10 of 10 runs, tool within 2 cm) is not met.**
  The best flights hold about 3 N with an IQR of 0.5 N and stay within 2 cm for about
  73 % of the time; others slip, lose contact and drift. One flight in five still falls.
- This simulator's slips may be pessimistic: friction is one number (0.5, high for a
  roller), and short force spikes (40-110 N, single samples) come from contact between
  rigid balls and a rigid wall.

## Holding the tool at a point

`tool_hold_demo`: 25 s hover with the arm still, then 25 s with the arm correcting the
drone's error in the air (damped least squares on the tool position, from the estimated
pose). Truth-measured scatter about the mean, RMS (cm), `outer_torque`, n=3:

| Position source | Arm still | Arm correcting | Within 2 cm, correcting |
|---|---|---|---|
| EKF | 12.8-15.8 | 8.5-12.7 | 0 % |
| 1 cm (not a real sensor) | 2.7-4.7 | 3.4-5.6 | 26-33 % |

The arm helps a little when the error is large (about 14 to 11 cm) and gains nothing
clear when it is already small. Most of the 14 cm is the estimate's own error, which
no correction based on that estimate can remove. Moving to a better position sensor is
worth far more than the arm correction. The contact pose also has joint 2 close to its
limit, which restricts corrections.

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
simulations during these flights, the arm's joints are stiff position-controlled joints and
the tool is rigid, with no compliance anywhere. A real arm and a sprung tool
would soften the spikes, so the spikes here are likely an upper bound. Contact is
balls on a flat wall with one friction value (0.5, high for a roller), and there is
no rotor wash near the wall (ground and wall effects are not simulated).

## What would come next

- A real sensor model for the 1 cm position: RTK in the EKF, or a marker seen by a camera.
- Fewer slips: lower friction on the head, softer contact, force-loop tuning; then 6 N.
- A contact pose further from joint limits, so the arm can correct.
- Trajectory shaping: minimum-jerk arm trajectories under a hover-error budget.
- Gripper and a pick task from hover.
