"""Unit tests for the pure helpers in sim_control_bridge.

The ROS node itself is covered separately by integration tests; here we
verify the rotor mixer and attitude PID behave sanely with no ROS spinning.
"""

import math
import pytest

# Import from control_math (no rclpy dependency) so tests are collectable
# without a sourced ROS environment.
from uavros2.control_math import quad_attitude_pid, rotor_mixer


# ----- quad_attitude_pid -----

def test_attitude_pid_zero_error_gives_zero_torque():
    torques = quad_attitude_pid(
        target_attitude=(0.0, 0.0, 0.0),
        current_attitude=(0.0, 0.0, 0.0),
    )
    assert torques == (0.0, 0.0, 0.0)


def test_attitude_pid_positive_roll_error_gives_positive_torque():
    torques = quad_attitude_pid(
        target_attitude=(0.1, 0.0, 0.0),
        current_attitude=(0.0, 0.0, 0.0),
    )
    assert torques[0] > 0, "positive roll error -> positive roll torque"
    assert torques[1] == pytest.approx(0.0)
    assert torques[2] == pytest.approx(0.0)


def test_attitude_pid_rate_damping_opposes_motion():
    # Zero attitude error, positive yaw rate -> torque should oppose.
    torques = quad_attitude_pid(
        target_attitude=(0.0, 0.0, 0.0),
        current_attitude=(0.0, 0.0, 0.0),
        target_rate=(0.0, 0.0, 0.0),
        current_rate=(0.0, 0.0, 0.5),
    )
    assert torques[2] < 0, "positive yaw rate -> negative damping torque"


# ----- rotor_mixer -----

def test_mixer_pure_thrust_is_equal_on_all_rotors():
    rotors = rotor_mixer(thrust=0.5, torques=(0.0, 0.0, 0.0))
    assert len(rotors) == 4
    for r in rotors:
        assert r == pytest.approx(0.5)


def test_mixer_output_clipped_to_unit_interval():
    rotors = rotor_mixer(thrust=10.0, torques=(0.0, 0.0, 0.0))
    for r in rotors:
        assert 0.0 <= r <= 1.0


def test_mixer_output_clipped_at_zero():
    rotors = rotor_mixer(thrust=-1.0, torques=(0.0, 0.0, 0.0))
    for r in rotors:
        assert r == 0.0


def test_mixer_roll_torque_asymmetric_left_right():
    """A positive roll torque should asymmetrically tilt rotors."""
    base = rotor_mixer(thrust=0.5, torques=(0.0, 0.0, 0.0))
    rolled = rotor_mixer(thrust=0.5, torques=(0.1, 0.0, 0.0))
    # At least one rotor should change relative to baseline.
    assert any(b != r for b, r in zip(base, rolled)), \
        "non-zero torque should change rotor distribution"
