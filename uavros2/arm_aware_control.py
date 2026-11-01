"""Arm-aware hover control: what the arm does to the drone, and how to cancel it.

No ROS here, so it can be tested alone. ``aerial_arm_demo`` runs it under
``-p controller:=outer`` or ``outer_ff``. Output is what PX4's offboard attitude
mode takes: an attitude and a normalized thrust, with PX4 itself unchanged.

The outer loop uses PX4's own default position gains, so ``outer`` (no
feed-forward) is comparable with PX4 position control, and ``outer_ff`` adds:

* **Moment.** The arm's centre of mass away from the thrust line makes a torque
  ``r x m g``. PX4's rate integrator absorbs it in about a second; until then the
  drone tilts and accelerates. Bias the attitude setpoint against the torque
  through a washout, because the integrator soon takes over the steady part.
* **Reaction.** Accelerating the arm pushes back on the drone: a force
  ``-m a`` and a torque ``-dL/dt`` from the arm's momentum.

Frames: world ENU, body FLU (mavros's ``map`` and ``base_link``).
"""

import math
import xml.etree.ElementTree as ET

import numpy as np

from .arm_kinematics import _floats, _joint_motion, transform

GRAVITY = 9.81
# PX4 v1.15 defaults for the outer loop, as acceleration gains
# (MPC_XY_VEL_P_ACC * MPC_XY_P, ...).
KP = np.array([1.71, 1.71, 4.0])
KD = np.array([1.8, 1.8, 4.0])
KI = np.array([0.38, 0.38, 2.0])
I_LIMIT = np.array([3.0, 3.0, 4.0])  # m/s^2
MAX_TILT = math.radians(25.0)
# How PX4 turns attitude error into torque before its integrator acts:
# MC_PITCH_P 6.5 * MC_PITCHRATE_P 0.15 of normalized torque per rad.
ATTITUDE_GAIN = 0.975
# 1.0 of normalized torque for the x500 at hover carrying the OpenManipulator-X:
# four rotors at 0.13 m, 0.707 of command, 12.7 N of thrust per unit command.
# It scales with the motors' thrust slope (5.88 for the tool model's 13 N motors).
NM_PER_NORMALIZED = 4.67
# How fast that integrator takes over: MC_PITCHRATE_P / MC_PITCHRATE_I, in s.
WASHOUT_S = 0.75
# Low-pass on the differentiated arm momentum, in s.
FILTER_S = 0.04


class ArmModel:
    """Masses and kinematics of the arm chain from a URDF."""

    def __init__(self, urdf_xml, root="base_link"):
        robot = ET.fromstring(urdf_xml)
        by_parent = {}
        for joint in robot.findall("joint"):
            by_parent.setdefault(joint.find("parent").get("link"), []).append(joint)
        self.links = []
        self.mass = 0.0
        links = {link.get("name"): link for link in robot.findall("link")}

        def walk(parent, chain):
            for joint in by_parent.get(parent, []):
                child = joint.find("child").get("link")
                origin = joint.find("origin")
                xyz = _floats(origin.get("xyz") if origin is not None else None, 3)
                rpy = _floats(origin.get("rpy") if origin is not None else None, 3)
                axis = joint.find("axis")
                step = (joint.get("name"), joint.get("type"), transform(xyz, rpy),
                        _floats(axis.get("xyz") if axis is not None else "1 0 0", 3))
                sub = chain + [step]
                inertial = links[child].find("inertial")
                if inertial is not None:
                    o = inertial.find("origin")
                    com = np.array(_floats(o.get("xyz") if o is not None else None, 3))
                    m = float(inertial.find("mass").get("value"))
                    i = inertial.find("inertia")
                    inertia = np.array([[float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
                                        [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
                                        [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]])
                    self.links.append((sub, m, com, inertia))
                    self.mass += m
                walk(child, sub)

        walk(root, [])

    def _poses(self, q):
        """(rotation, centre of mass) of every link in the root frame."""
        out = []
        for chain, _m, com, _i in self.links:
            T = np.eye(4)
            for name, jtype, fixed, axis in chain:
                T = T @ fixed
                if name in q:
                    T = T @ _joint_motion(jtype, axis, q[name])
            out.append((T[:3, :3], T[:3, :3] @ com + T[:3, 3]))
        return out

    def state(self, q, qd):
        """Arm mass moment m*r (kg m), momentum m*v and angular momentum L."""
        eps = 1e-4
        poses = self._poses(q)
        ahead = self._poses({k: v + eps * qd.get(k, 0.0) for k, v in q.items()})
        moment, p, ang = np.zeros(3), np.zeros(3), np.zeros(3)
        for (_c, m, _com, inertia), (R, r), (R2, r2) in zip(self.links, poses, ahead):
            v = (r2 - r) / eps
            dR = (R2 @ R.T - np.eye(3)) / eps
            w = np.array([dR[2, 1], dR[0, 2], dR[1, 0]])
            moment += m * r
            p += m * v
            ang += m * np.cross(r, v) + R @ inertia @ R.T @ w
        return moment, p, ang


def attitude_from_acceleration(acc, yaw, hover_thrust):
    """Attitude (rotation matrix, body to world) and normalized thrust for ``acc``."""
    total = np.array([acc[0], acc[1], acc[2] + GRAVITY])
    total[2] = max(total[2], 0.3 * GRAVITY)
    horizontal = math.hypot(total[0], total[1])
    limit = total[2] * math.tan(MAX_TILT)
    if horizontal > limit:
        total[:2] *= limit / horizontal
    z = total / np.linalg.norm(total)
    y = np.cross(z, [math.cos(yaw), math.sin(yaw), 0.0])
    y /= np.linalg.norm(y)
    R = np.column_stack((np.cross(y, z), y, z))
    thrust = hover_thrust * np.linalg.norm(total) / GRAVITY
    return R, float(np.clip(thrust, 0.05, 1.0))


def rotation_vector_to_matrix(v):
    angle = float(np.linalg.norm(v))
    if angle < 1e-9:
        return np.eye(3)
    k = np.asarray(v) / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def matrix_to_quaternion(R):
    """(x, y, z, w) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return x, y, z, w


class ArmFeedForward:
    """Attitude bias and acceleration that cancel what the arm does to the drone."""

    def __init__(self, model, body_mass, nm_per_normalized=NM_PER_NORMALIZED,
                 washout_s=WASHOUT_S):
        self.model = model
        self.total_mass = body_mass + model.mass
        self.nm_per_normalized = nm_per_normalized
        self.rad_per_nm = 1.0 / (ATTITUDE_GAIN * nm_per_normalized)
        self.washout_s = washout_s
        # Other known torque on the body, FLU N m (a measured contact force).
        self.external_torque = np.zeros(3)
        self._slow = None
        self._prev = None
        self._reaction_f = np.zeros(3)
        self._reaction_t = np.zeros(3)

    def px4_torque(self, q, qd, dt):
        """Normalized FRD torque for PX4's rate controller: cancels the arm's."""
        moment, p, ang = self.model.state(q, qd)
        arm = GRAVITY * np.array([-moment[1], moment[0], 0.0])
        if self._prev is None:
            self._prev = (p, ang)
        self._reaction_t += ((self._prev[1] - ang) / dt - self._reaction_t) * min(1.0, dt / FILTER_S)
        self._prev = (p, ang)
        arm = arm + self._reaction_t + self.external_torque
        return np.array([-arm[0], arm[1], arm[2]]) / self.nm_per_normalized

    def update(self, q, qd, dt):
        """Returns (attitude bias: body rotation vector, body-frame acceleration)."""
        moment, p, ang = self.model.state(q, qd)
        gravity_torque = GRAVITY * np.array([-moment[1], moment[0], 0.0])  # r x (0, 0, -mg)
        gravity_torque = gravity_torque + self.external_torque
        if self._slow is None:
            self._slow = gravity_torque.copy()
            self._prev = (p, ang)
        self._slow += (gravity_torque - self._slow) * min(1.0, dt / self.washout_s)
        alpha = min(1.0, dt / FILTER_S)
        self._reaction_f += ((self._prev[0] - p) / dt - self._reaction_f) * alpha
        self._reaction_t += ((self._prev[1] - ang) / dt - self._reaction_t) * alpha
        self._prev = (p, ang)
        torque = gravity_torque - self._slow + self._reaction_t
        # The body must push back with what the arm pushes on it: the reaction
        # force on the drone is _reaction_f, so ask for the opposite.
        return -self.rad_per_nm * torque, -self._reaction_f / self.total_mass


class WallForceControl:
    """Acceleration along a wall's normal that holds a contact force.

    Commands the force directly (the wall stops the drone, so force = thrust
    component along the normal), PI on the measured force, damping on velocity.
    Without contact it creeps forward, so regaining contact does not become an
    impact.
    """

    KP = 0.5
    KI = 0.5
    I_LIMIT_N = 4.0
    KD = 1.8
    CONTACT_N = 0.2
    CREEP_M_S = 0.05

    def __init__(self, total_mass, target_n):
        self.mass = total_mass
        self.target = target_n
        self.measured = 0.0
        self.integral = 0.0

    def acceleration(self, velocity, dt):
        if self.measured < self.CONTACT_N:
            return self.KD * (self.CREEP_M_S - velocity)
        err = self.target - self.measured
        self.integral = float(np.clip(self.integral + self.KI * err * dt,
                                      -self.I_LIMIT_N, self.I_LIMIT_N))
        force = self.target + self.KP * err + self.integral
        return force / self.mass - self.KD * velocity


class OuterLoop:
    """Position hold giving an attitude and thrust, with optional feed-forward."""

    def __init__(self, hover_thrust, feed_forward=None):
        self.hover_thrust = hover_thrust
        self.feed_forward = feed_forward
        self.integral = np.zeros(3)
        # Freeze an axis's integrator (x, against a wall it would wind up).
        self.freeze = np.zeros(3, dtype=bool)
        # When set, it replaces position control along world x.
        self.force_control = None

    def update(self, setpoint, yaw, pos, vel, q, qd, dt):
        err = np.asarray(setpoint) - pos
        grow = np.where(self.freeze, 0.0, KI * err * dt)
        self.integral = np.clip(self.integral + grow, -I_LIMIT, I_LIMIT)
        acc = KP * err - KD * vel + self.integral
        if self.force_control is not None:
            acc[0] = self.force_control.acceleration(vel[0], dt)
        bias = np.zeros(3)
        body_acc = np.zeros(3)
        if self.feed_forward is not None:
            bias, body_acc = self.feed_forward.update(q, qd, dt)
            c, s = math.cos(yaw), math.sin(yaw)
            acc = acc + np.array([c * body_acc[0] - s * body_acc[1],
                                  s * body_acc[0] + c * body_acc[1], body_acc[2]])
        R, thrust = attitude_from_acceleration(acc, yaw, self.hover_thrust)
        R = R @ rotation_vector_to_matrix(bias)
        return matrix_to_quaternion(R), thrust, bias, body_acc
