"""URDF and Gazebo SDF of every arm must describe the same kinematic chain.

robot_state_publisher / RViz / MoveIt read the xacro URDF; Gazebo simulates
models/x500_with_<arm>/model.sdf. If they disagree, TF and planning put the
end effector somewhere the physics does not (this caught a missing mount
transform: the URDF arms pointed up while the simulated arms hung down).
"""

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from uavros2.arm_kinematics import (
    OMX_TOOL_CONTACT_POSE, OMX_TOOL_LENGTH, OMX_TOOL_STOW_POSE, OMX_TOOL_TIP_IN_BASE,
    sdf_fk, urdf_fk,
)

REPO = Path(__file__).resolve().parent.parent

ARMS = {
    "three_dof": ("x500_with_three_dof_arm", "ee_link",
                  ["j_shoulder", "j_elbow", "j_wrist"]),
    "openmanip_x": ("x500_with_openmanip_x", "end_effector_link",
                    ["joint1", "joint2", "joint3", "joint4"]),
    "panda": ("x500_with_panda", "panda_hand",
              [f"panda_joint{i}" for i in range(1, 8)]),
    "ur5": ("x500_with_ur5", "tool0",
            ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]),
}


def _urdf(arm, *mappings):
    xacro = shutil.which("xacro")
    if xacro is None:
        pytest.skip("xacro not on PATH (source /opt/ros/jazzy/setup.bash)")
    return subprocess.check_output(
        [xacro, str(REPO / "arms" / arm / "urdf" / f"{arm}.urdf.xacro"), "sim_mode:=mock",
         *mappings],
        stderr=subprocess.DEVNULL).decode()


@pytest.mark.parametrize("arm", sorted(ARMS))
def test_urdf_matches_sdf(arm):
    model, ee, joints = ARMS[arm]
    urdf = _urdf(arm)
    sdf = (REPO / "models" / model / "model.sdf").read_text()
    rng = np.random.default_rng(0)
    configs = [np.zeros(len(joints))] + [rng.uniform(-2.5, 2.5, len(joints))
                                         for _ in range(20)]
    for qv in configs:
        q = dict(zip(joints, qv))
        T_u = urdf_fk(urdf, "base_link", ee, q)
        T_s = sdf_fk(sdf, "base_link", ee, q)
        np.testing.assert_allclose(T_u[:3, 3], T_s[:3, 3], atol=1e-4,
                                   err_msg=f"{arm} EE position at q={qv}")
        np.testing.assert_allclose(T_u[:3, :3], T_s[:3, :3], atol=1e-4,
                                   err_msg=f"{arm} EE orientation at q={qv}")


def _initial_pose(urdf_xml, joints):
    """Joint values the arm spawns with: ros2_control initial_value, else 0."""
    q = dict.fromkeys(joints, 0.0)
    for j in ET.fromstring(urdf_xml).findall("ros2_control/joint"):
        for p in j.findall("state_interface/param"):
            if p.get("name") == "initial_value" and j.get("name") in q:
                q[j.get("name")] = float(p.text)
    return q


@pytest.mark.parametrize("arm", sorted(ARMS))
def test_arm_spawns_above_landing_gear(arm):
    model, ee, joints = ARMS[arm]
    urdf = _urdf(arm)
    q = _initial_pose(urdf, joints)
    links = [j.find("child").get("link") for j in ET.fromstring(urdf).findall("joint")]
    # Every link frame of the spawn pose hangs below base_link but above the
    # x500 gear feet (-0.2195 m), so the drone rests on its gear.
    for link in links:
        if link == "base_link":
            continue
        z = urdf_fk(urdf, "base_link", link, q)[2, 3]
        assert -0.2195 < z < 0.0, f"{arm}: {link} at z={z:.3f} in the spawn pose"


def test_contact_tool_model_matches_urdf():
    """The tool model is the OMX plus the rod; its tip is where the demo assumes."""
    urdf = _urdf("openmanip_x", "tool:=rod")
    sdf = (REPO / "models/x500_with_openmanip_x_tool/model.sdf").read_text()
    q = dict(zip(["joint1", "joint2", "joint3", "joint4"], OMX_TOOL_CONTACT_POSE))
    T_u = urdf_fk(urdf, "base_link", "tool_tip", q)
    T_s = sdf_fk(sdf, "base_link", "tool_link", q)
    tip_s = T_s[:3, 3] + OMX_TOOL_LENGTH * T_s[:3, 0]
    np.testing.assert_allclose(T_u[:3, 3], tip_s, atol=1e-4)
    np.testing.assert_allclose(T_u[:3, 3], OMX_TOOL_TIP_IN_BASE, atol=1e-3)
    assert abs(T_u[0, 0] - 1.0) < 1e-3, "rod must be level in the contact pose"


def test_contact_tool_stow_clears_the_gear():
    urdf = _urdf("openmanip_x", "tool:=rod")
    q = dict(zip(["joint1", "joint2", "joint3", "joint4"], OMX_TOOL_STOW_POSE))
    for link in ("tool_link", "tool_tip"):
        assert -0.2195 < urdf_fk(urdf, "base_link", link, q)[2, 3] < 0.0
