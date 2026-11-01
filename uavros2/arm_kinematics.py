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

# OpenManipulator-X with the 0.40 m contact tool. Contact pose: rod level,
# pointing forward, its tip 0.20 m past the propeller tips, arm folded close in:
# 0.022 kg m of forward moment about base_link. (A 0.20 m rod needs the arm at
# full reach, 0.122 kg m: the front rotors then sit at 99 % in free hover and
# the drone falls on first contact.) Stow pose: rod backwards under the body,
# no static moment.
OMX_TOOL_LENGTH = 0.40
OMX_TOOL_CONTACT_POSE = [0.0, -1.4, 0.6, 0.8]
OMX_TOOL_STOW_POSE = [0.0, 0.9, 1.1, 1.3]
OMX_TOOL_TIP_IN_BASE = (0.502, 0.0, -0.244)


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
    """Walk parent->child joints from root to tip. ``joints`` maps child ->
    (parent, X_parent_joint, type, axis, name)."""
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
