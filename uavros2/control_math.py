"""Pure-Python control helpers for sim_control_bridge.

Lives in its own module so unit tests can import these without pulling
rclpy at collection time.
"""

from __future__ import annotations


def quad_attitude_pid(
    target_attitude: tuple,
    current_attitude: tuple,
    target_rate: tuple = (0.0, 0.0, 0.0),
    current_rate: tuple = (0.0, 0.0, 0.0),
    kp: tuple = (6.0, 6.0, 2.0),
    kd: tuple = (0.3, 0.3, 0.2),
) -> tuple:
    """Cascaded P-PD attitude controller.

    Returns body torques (mx, my, mz). Inputs are roll/pitch/yaw (rad)
    and body rates (rad/s).
    """
    err = tuple(t - c for t, c in zip(target_attitude, current_attitude))
    rate_err = tuple(t - c for t, c in zip(target_rate, current_rate))
    return tuple(kp[i] * err[i] + kd[i] * rate_err[i] for i in range(3))


def rotor_mixer(
    thrust: float,
    torques: tuple,
    arm_length: float = 0.25,
    k_torque: float = 0.05,
) -> tuple:
    """Quad-X mixer.

    Returns 4 rotor thrust commands (front-left, front-right, rear-right,
    rear-left), clipped to [0, 1].

    thrust   in [0, 1]
    torques  body (mx, my, mz)
    """
    mx, my, mz = torques
    l = arm_length
    k = k_torque
    f1 = thrust + (my - mx) / (4 * l) - mz / (4 * k)
    f2 = thrust + (-my - mx) / (4 * l) + mz / (4 * k)
    f3 = thrust + (-my + mx) / (4 * l) - mz / (4 * k)
    f4 = thrust + (my + mx) / (4 * l) + mz / (4 * k)

    def _clip(v: float) -> float:
        return max(0.0, min(1.0, v))

    return (_clip(f1), _clip(f2), _clip(f3), _clip(f4))
