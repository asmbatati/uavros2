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


def position_control(
    pos_sp,
    yaw_sp: float,
    pos,
    vel,
    rot,
    omega,
    mass: float,
    inertia,
    gravity: float = 9.81,
    kp_pos: float = 3.0,
    kd_pos: float = 2.5,
    kp_att: float = 40.0,
    kd_att: float = 10.0,
    max_acc: float = 5.0,
):
    """Hold a position and yaw: geometric controller on SO(3).

    Inputs are numpy arrays in the world frame except ``omega`` (body rates)
    and ``rot`` (body-to-world rotation matrix). Returns (thrust_N, torque_Nm),
    the torque in the body frame.
    """
    import numpy as np

    acc = kp_pos * (np.asarray(pos_sp) - pos) - kd_pos * vel
    norm = np.linalg.norm(acc)
    if norm > max_acc:
        acc = acc * max_acc / norm
    force = mass * (acc + np.array([0.0, 0.0, gravity]))

    z_des = force / np.linalg.norm(force)
    y_des = np.cross(z_des, [np.cos(yaw_sp), np.sin(yaw_sp), 0.0])
    y_des /= np.linalg.norm(y_des)
    rot_des = np.column_stack((np.cross(y_des, z_des), y_des, z_des))

    err = 0.5 * (rot_des.T @ rot - rot.T @ rot_des)
    err_vec = np.array([err[2, 1], err[0, 2], err[1, 0]])
    torque = np.asarray(inertia) * (-kp_att * err_vec - kd_att * np.asarray(omega))
    return float(force @ rot[:, 2]), torque


def allocation_matrix(rotor_xy, thrust_gain, yaw_gain):
    """Map rotor commands to [thrust, mx, my, mz] (body frame, z up).

    A rotor at (x, y) producing force f gives mx = y f, my = -x f.
    """
    import numpy as np

    xy = np.asarray(rotor_xy)
    gain = np.asarray(thrust_gain)
    return np.vstack((gain, xy[:, 1] * gain, -xy[:, 0] * gain, np.asarray(yaw_gain)))
