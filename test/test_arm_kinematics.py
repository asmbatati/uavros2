"""URDF and Gazebo SDF of every arm must describe the same kinematic chain.

robot_state_publisher / RViz / MoveIt read the xacro URDF; Gazebo simulates
models/x500_with_<arm>/model.sdf. If they disagree, TF and planning put the
end effector somewhere the physics does not (this caught a missing mount
transform: the URDF arms pointed up while the simulated arms hung down).
"""

import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from uavros2.arm_kinematics import sdf_fk, urdf_fk

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


def _urdf(arm):
    xacro = shutil.which("xacro")
    if xacro is None:
        pytest.skip("xacro not on PATH (source /opt/ros/jazzy/setup.bash)")
    return subprocess.check_output(
        [xacro, str(REPO / "arms" / arm / "urdf" / f"{arm}.urdf.xacro"), "sim_mode:=mock"],
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


@pytest.mark.parametrize("arm", sorted(ARMS))
def test_arm_hangs_below_base_link(arm):
    model, ee, joints = ARMS[arm]
    T = urdf_fk(_urdf(arm), "base_link", ee, dict.fromkeys(joints, 0.0))
    # Zero configuration hangs straight down, above the gear feet (-0.2195 m).
    assert -0.2195 < T[2, 3] < -0.1
