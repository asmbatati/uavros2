# Aerial-arm flight experiment

How much does a manipulator moving under a hovering x500 disturb it, and how
far off does that put the end effector? This page describes the experiment
that measures it and the results from PX4 v1.15.4 SITL (30 Oct 2026).

## Protocol

`ros2 run uavros2 aerial_arm_demo -p arm:=<arm>` (`uavros2/aerial_arm_demo.py`):

1. Wait for MAVROS, the local position and the arm's `controller_state`.
2. Stream a position setpoint 2 m above the take-off point at 20 Hz, switch
   to OFFBOARD and arm; climb.
3. **hover**: hold 8 s with the arm still.
4. Four arm phases, each a 2 s joint trajectory (`move_s`) then a 5 s hold
   (`hold_s`), sent to `joint_trajectory_controller/follow_joint_trajectory`:
   **reach_a**, **reach_b**, **fold**, **stow**. For the OpenManipulator-X:
   forward reach (end effector ~0.31 m ahead), the same reach yawed 90° to the
   side, folded under the body, and the stow pose it takes off and lands in.
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
partition. One run takes about 80 s. `UAV=<model>` flies another airframe
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
| OMX scaffold 0.16 kg | 1 | reach_a | 7.5 | 11.5 | -9.2 | - | 0.016 |
| OMX scaffold 0.16 kg | 1 | reach_b | 7.7 | 12.7 | -13.4 | - | 0.032 |
| OMX scaffold 0.16 kg | 1 | fold | 6.5 | 11.4 | -10.8 | - | 0.025 |
| OMX 2 s motions | 4 | hover | 5.4 ± 1.0 | 9.3 ± 2.0 | -4.3 ± 6.8 | 8.3 ± 2.9 | 0.000 |
| OMX 2 s motions | 4 | reach_a | 13.3 ± 2.6 | 26.8 ± 4.7 | 0.1 ± 12.4 | 18.9 ± 7.2 | 0.036 |
| OMX 2 s motions | 4 | reach_b | 19.5 ± 1.5 | 34.8 ± 2.3 | -1.5 ± 12.2 | 24.0 ± 3.1 | 0.019 |
| OMX 2 s motions | 4 | fold | 17.5 ± 1.2 | 36.3 ± 2.8 | -3.3 ± 7.9 | 19.4 ± 1.1 | 0.037 |
| OMX 2 s motions | 4 | stow | 6.1 ± 1.6 | 10.1 ± 2.2 | -2.1 ± 5.6 | 9.3 ± 1.1 | 0.007 |
| OMX 6 s motions | 3 | hover | 5.5 ± 0.4 | 9.8 ± 0.9 | -3.5 ± 6.3 | 10.0 ± 1.7 | 0.000 |
| OMX 6 s motions | 3 | reach_a | 6.5 ± 1.6 | 12.3 ± 3.4 | 0.4 ± 8.3 | 10.8 ± 2.5 | 0.017 |
| OMX 6 s motions | 3 | reach_b | 11.4 ± 1.8 | 21.1 ± 2.6 | 2.4 ± 4.2 | 13.5 ± 0.1 | 0.009 |
| OMX 6 s motions | 3 | fold | 11.6 ± 0.6 | 19.4 ± 1.6 | 3.1 ± 1.1 | 12.2 ± 0.6 | 0.017 |
| OMX 6 s motions | 3 | stow | 5.6 ± 0.4 | 10.1 ± 1.7 | -2.3 ± 9.4 | 11.7 ± 4.9 | 0.003 |

Full tables: [summary.md](figures/aerial_arm/summary.md) (and
[scaffolds/summary.md](figures/aerial_arm/scaffolds/summary.md) for the
three_dof / Panda / UR5 scaffolds and the x500_d435 control).

### What it shows

- **Noise floor.** With nothing moving, PX4 holds the x500 within
  5.1 ± 0.3 cm horizontal RMS (peaks ~9 cm). Height wanders slowly by up to
  ±10 cm in every configuration, arm or not (compare the vertical SDs), so
  per-run vertical means say little.
- **The scaffold arms do not matter.** At 0.12-0.20 kg the three_dof,
  Panda-named and UR5-named scaffolds, and the old 0.16 kg OpenManipulator-X
  scaffold, stay within 1-3 cm of the baseline. They are fine for testing
  controllers and MoveIt, useless for studying coupling.
- **The real OpenManipulator-X does.** At 0.594 kg (30 % of the x500), a 2 s
  arm motion pushes the drone 27-36 cm off its setpoint at the peak and
  13-20 cm RMS over the phase, 2.5-4 times the noise floor; PX4 brings it
  back within about 3 s of the motion ending (see the time series). The side
  reach (reach_b) and the fold, which swing the most mass, are the worst.
- **The end effector misses by more.** During the reaches the end effector is
  19-24 cm RMS from where it should be in the world, so a 2 s arm motion
  cannot place the gripper precisely without compensation.
- **Slower motion halves it.** Moving the arm over 6 s instead of 2 s cuts the
  peak excursion to 12-21 cm and the end-effector error to 11-14 cm RMS
  (the RMS is over a longer phase, the peak is the fairer comparison). The
  disturbance is mostly the reaction to accelerating the arm, plus the
  centre-of-mass shift PX4's integrators then absorb.
- **Joint tracking is not the problem**: 0.003-0.04 rad RMS, all goals
  succeed.

![Real OpenManipulator-X, 2 s motions](figures/aerial_arm/omx_2_s_motions_flight.png)

### Height reference note

The generated airframes (4020-4028) use the barometer as EKF2 height reference
(`EKF2_HGT_REF 0`, PX4's simulated baro); stock x500 (4001) uses GPS height.
Flying x500_d435 (4020, no arm) against x500 (4001) showed the same ±10 cm
height wander for both, so it is not caused by the arm or by that setting.

## What would come next

- Compensation: feed the arm's centre-of-mass shift and reaction torque
  forward (e.g. an arm-aware outer loop sending attitude + thrust in
  OFFBOARD), then rerun this protocol.
- Trajectory shaping: minimum-jerk / time-optimal arm trajectories under a
  hover-error budget.
- Gripper and a pick task from hover.
