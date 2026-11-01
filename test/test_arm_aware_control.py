"""Tests for the arm-aware outer loop (uavros2.arm_aware_control)."""

import math
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from uavros2.arm_aware_control import (
    ArmFeedForward, ArmModel, OuterLoop, WallForceControl, attitude_from_acceleration,
    matrix_to_quaternion, rotation_vector_to_matrix,
)

REPO = Path(__file__).resolve().parent.parent
JOINTS = ["joint1", "joint2", "joint3", "joint4"]
STOW = dict(zip(JOINTS, [0.0, -1.1, 1.2, 0.9]))
REACH_A = dict(zip(JOINTS, [0.0, 0.8, -0.3, 0.4]))
ZERO = dict.fromkeys(JOINTS, 0.0)


@pytest.fixture(scope="module")
def model():
    xacro = shutil.which("xacro")
    if xacro is None:
        pytest.skip("xacro not on PATH (source /opt/ros/jazzy/setup.bash)")
    urdf = subprocess.check_output(
        [xacro, str(REPO / "arms/openmanip_x/urdf/openmanip_x.urdf.xacro"), "sim_mode:=mock"],
        stderr=subprocess.DEVNULL).decode()
    return ArmModel(urdf)


def test_model_mass_and_com(model):
    assert model.mass == pytest.approx(0.594, abs=0.005)
    moment, p, ang = model.state(REACH_A, ZERO)
    # docs: this pose puts the arm's centre of mass about 0.13 m forward.
    assert moment[0] / model.mass == pytest.approx(0.13, abs=0.02)
    assert np.allclose(p, 0.0) and np.allclose(ang, 0.0)


def test_forward_reach_biases_nose_up(model):
    ff = ArmFeedForward(model, body_mass=2.06)
    ff.update(STOW, ZERO, 0.02)
    bias, _ = ff.update(REACH_A, ZERO, 0.02)
    # A forward centre of mass pushes the nose down, +y in FLU: bias against it.
    assert bias[1] < -0.01
    assert abs(bias[0]) < 1e-6


def test_bias_washes_out_for_a_held_pose(model):
    ff = ArmFeedForward(model, body_mass=2.06)
    ff.update(STOW, ZERO, 0.02)
    for _ in range(500):
        bias, acc = ff.update(REACH_A, ZERO, 0.02)
    assert np.linalg.norm(bias) < 1e-3
    assert np.linalg.norm(acc) < 1e-3


def test_level_hover_at_hover_thrust():
    R, thrust = attitude_from_acceleration(np.zeros(3), 0.0, 0.8)
    assert np.allclose(R, np.eye(3))
    assert thrust == pytest.approx(0.8)


def test_forward_acceleration_pitches_nose_down_and_is_limited():
    R, _ = attitude_from_acceleration(np.array([1.0, 0.0, 0.0]), 0.0, 0.8)
    assert R[0, 2] > 0.05          # thrust axis leans forward
    R, _ = attitude_from_acceleration(np.array([100.0, 0.0, 0.0]), 0.0, 0.8)
    assert math.degrees(math.acos(R[2, 2])) == pytest.approx(25.0, abs=0.1)


def test_quaternion_round_trip():
    q = matrix_to_quaternion(rotation_vector_to_matrix([0.0, 0.3, 0.0]))
    assert q[1] == pytest.approx(math.sin(0.15))
    assert q[3] == pytest.approx(math.cos(0.15))


def test_outer_loop_pushes_towards_setpoint():
    loop = OuterLoop(0.8)
    q, thrust, _, _ = loop.update([1.0, 0.0, 1.0], 0.0, np.zeros(3), np.zeros(3), {}, {}, 0.02)
    assert thrust > 0.8      # climb
    assert q[1] > 0.0        # lean forward


def test_px4_torque_counters_forward_reach(model):
    ff = ArmFeedForward(model, body_mass=2.06)
    torque = ff.px4_torque(REACH_A, ZERO, 0.02)
    # A forward centre of mass pushes the nose down; nose up is +y in FRD.
    assert torque[1] == pytest.approx(0.76 / 4.67, abs=0.04)


def test_contact_force_below_the_body_is_a_nose_down_torque():
    # r x F with the tip 0.244 m below and the wall pushing back, +y in FLU.
    r = np.array([0.502, 0.0, -0.244])
    expected = np.cross(r, [-2.0, 0.0, 0.0])
    assert expected[1] == pytest.approx(0.488)
    pytest.importorskip("geometry_msgs")  # wall_contact_demo needs ROS
    from uavros2.wall_contact_demo import contact_torque
    assert np.allclose(contact_torque(2.0), expected)


def test_force_control_holds_target_when_at_target():
    fc = WallForceControl(total_mass=2.0, target_n=3.0)
    fc.measured = 3.0
    assert fc.acceleration(0.0, 0.02) == pytest.approx(1.5)


def test_force_control_pushes_harder_when_force_is_low_and_integrates():
    fc = WallForceControl(total_mass=2.0, target_n=3.0)
    fc.measured = 1.0
    first = fc.acceleration(0.0, 0.02)
    assert first > 1.5
    for _ in range(100):
        last = fc.acceleration(0.0, 0.02)
    assert last > first


def test_force_control_creeps_without_contact():
    fc = WallForceControl(total_mass=2.0, target_n=3.0)
    fc.measured = 0.0
    assert fc.acceleration(0.05, 0.02) == pytest.approx(0.0)
    assert fc.acceleration(0.5, 0.02) < 0.0


def test_outer_loop_uses_force_control_along_x_only():
    loop = OuterLoop(0.8)
    loop.force_control = WallForceControl(2.0, 3.0)
    loop.force_control.measured = 3.0
    # A big x position error must not matter; force control owns that axis.
    q, _, _, _ = loop.update([5.0, 0.0, 0.0], 0.0, np.zeros(3), np.zeros(3), {}, {}, 0.02)
    tilt_small = q[1]
    assert 0.0 < tilt_small < math.sin(math.atan(1.5 / 9.81) / 2) * 1.2
