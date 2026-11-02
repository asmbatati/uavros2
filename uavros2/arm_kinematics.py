"""Forward kinematics of the arm descriptions, from URDF and from Gazebo SDF.

Each arm exists twice: the xacro/URDF that robot_state_publisher and MoveIt
read, and the composed ``models/x500_with_<arm>/model.sdf`` that Gazebo
simulates. Both must describe the same chain, or RViz and MoveIt plan for
an arm that is not where the physics has it. This module computes the
end-effector pose relative to the drone's ``base_link`` from either file so
the two can be compared (see ``test/test_arm_kinematics.py``).
"""

import math
import xml.etree.ElementTree as ET

import numpy as np

# OpenManipulator-X with the 0.50 m contact tool. Contact pose: rod level, pointing
# forward, its tip 0.21 m past the propeller tips; the arm folded so the rod's yaw
# pivot sits on the body's yaw axis and the arm and rod balance about it (almost no
# static moment, no yaw lever for the contact force). Stow pose: rod backwards under
# the body.
OMX_TOOL_LENGTH = 0.50
OMX_TOOL_CONTACT_POSE = [0.0, -1.4, -0.2, 1.6]
OMX_TOOL_STOW_POSE = [0.0, 0.9, 1.1, 1.3]
OMX_TOOL_TIP_IN_BASE = (0.512, 0.0, -0.279)

# OpenManipulator-X gripper (gripper_left_joint; the right finger mimics it): the pads are
# 0.040 + 2 q apart, and end_effector_link sits between their tips.
OMX_GRIPPER_OPEN = 0.019
OMX_GRIPPER_CLOSED = -0.010
OMX_PAD_LENGTH = 0.0443
# The OMX chain in its own frame (openmanip_x_link1, hung from base_link flipped):
# joint 1 along x, joint 2 above it, the upper arm, forearm and hand to end_effector_link.
_OMX_MOUNT_Z = -0.05
_OMX_JOINT1_X = 0.012
_OMX_JOINT2_Z = 0.0595
_OMX_UPPER = complex(0.024, 0.128)
_OMX_FOREARM = 0.124
_OMX_HAND = 0.126
_OMX_LIMITS = ((-3.14159, 3.14159), (-1.5, 1.5), (-1.5, 1.4), (-1.7, 1.97))


def omx_gripper_down_ik(point):
    """OMX joints that put end_effector_link at ``point`` (base_link), the gripper straight down.

    Fingers close along the arm's y axis: base_link y with joint 1 at 0. Joint 1 also
    turns the fingers about the vertical, so a point off the body's x axis turns them;
    points under the body need joint 1 near its singularity (the axis of joint 1 itself),
    where it stays within +-90 degrees and the arm reaches behind the axis instead. Elbow
    towards the front, the branch that keeps the arm's centre of mass within 3 cm of the
    body's axis. Raises ValueError out of reach or outside the joint limits.
    """
    # In openmanip_x_link1: 0.05 m below base_link, rolled by pi.
    ax, ay, az = point[0], -point[1], -(point[2] - _OMX_MOUNT_Z)
    j1 = math.atan2(ay, ax - _OMX_JOINT1_X)
    if abs(j1) > math.pi / 2.0:
        j1 -= math.copysign(math.pi, j1)
    reach = (ax - _OMX_JOINT1_X) * math.cos(j1) + ay * math.sin(j1)
    # Rotating a vector (x, z) of the pitch plane by a joint angle t multiplies x + iz
    # by exp(-it). Straight down is the plane's +z, so the wrist sits a hand above the point.
    d = complex(reach, az - _OMX_HAND - _OMX_JOINT2_Z)
    upper = abs(_OMX_UPPER)
    c = (abs(d) ** 2 - upper ** 2 - _OMX_FOREARM ** 2) / (2.0 * upper * _OMX_FOREARM)
    if abs(c) > 1.0:
        raise ValueError(f"{point} is out of the gripper's reach")
    j3 = math.acos(c) - math.atan2(_OMX_UPPER.imag, _OMX_UPPER.real)
    k = _OMX_UPPER + _OMX_FOREARM * complex(math.cos(j3), -math.sin(j3))
    j2 = math.atan2(k.imag, k.real) - math.atan2(d.imag, d.real)
    j4 = -math.pi / 2.0 - j2 - j3
    q = [j1, j2, j3, j4]
    for v, (lo, hi), name in zip(q, _OMX_LIMITS, ("joint1", "joint2", "joint3", "joint4")):
        if not lo <= v <= hi:
            raise ValueError(f"{point} needs {name} = {v:.2f}, outside [{lo}, {hi}]")
    return q


def _rot(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n == 0.0 or angle == 0.0:
        return np.eye(3)
    x, y, z = axis / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _rpy(r, p, y):
    return _rot((0, 0, 1), y) @ _rot((0, 1, 0), p) @ _rot((1, 0, 0), r)


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    """4x4 homogeneous transform from a translation and roll-pitch-yaw."""
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


def _joint_motion(jtype, axis, q):
    T = np.eye(4)
    if jtype in ("revolute", "continuous"):
        T[:3, :3] = _rot(axis, q)
    elif jtype == "prismatic":
        T[:3, 3] = np.asarray(axis, dtype=float) * q
    return T


def _floats(text, n, default=0.0):
    vals = [float(v) for v in (text or "").split()]
    return tuple(vals + [default] * (n - len(vals)))


def _chain_fk(joints, root, tip, q):
    """
    Walk parent->child joints from root to tip.

    ``joints`` maps child -> (parent, X_parent_joint, type, axis, name).
    """
    path = []
    link = tip
    while link != root:
        if link not in joints:
            raise KeyError(f"{tip!r} is not below {root!r}")
        path.append(joints[link])
        link = joints[link][0]
    T = np.eye(4)
    for parent, origin, jtype, axis, name in reversed(path):
        T = T @ origin @ _joint_motion(jtype, axis, q.get(name, 0.0))
    return T


def urdf_fk(urdf_xml, root, tip, q):
    """Pose of ``tip`` in ``root`` for joint values ``q`` (name -> value)."""
    robot = ET.fromstring(urdf_xml)
    joints = {}
    for j in robot.findall("joint"):
        o = j.find("origin")
        origin = transform(_floats(o.get("xyz") if o is not None else None, 3),
                           _floats(o.get("rpy") if o is not None else None, 3))
        a = j.find("axis")
        axis = _floats(a.get("xyz") if a is not None else "1 0 0", 3)
        joints[j.find("child").get("link")] = (
            j.find("parent").get("link"), origin, j.get("type"), axis, j.get("name"))
    return _chain_fk(joints, root, tip, q)


def sdf_fk(sdf_xml, root, tip, q):
    """Pose of link ``tip`` in link ``root`` of a composed model SDF.

    Supports what the composed arm models use: link poses given
    ``relative_to`` another link (or the model), joints whose frame is the
    child link frame (the SDF default), and axes expressed in that frame.
    """
    model = ET.fromstring(sdf_xml).find(".//model") if not sdf_xml.lstrip().startswith(
        "<model") else ET.fromstring(sdf_xml)
    rel = {}
    for link in model.findall("link"):
        p = link.find("pose")
        rel[link.get("name")] = (
            p.get("relative_to") if p is not None else None,
            transform(*np.split(np.array(_floats(p.text if p is not None else None, 6)), 2)),
        )
    joints = {}
    for j in model.findall("joint"):
        child, parent = j.find("child").text.strip(), j.find("parent").text.strip()
        frame, X = rel.get(child, (None, np.eye(4)))
        if frame not in (None, parent):
            raise ValueError(f"link {child!r} pose is relative_to {frame!r}, "
                             f"expected its parent {parent!r}")
        a = j.find("axis/xyz")
        axis = _floats(a.text if a is not None else "0 0 1", 3)
        joints[child] = (parent, X, j.get("type"), axis, j.get("name"))
    return _chain_fk(joints, root, tip, q)
