"""Arm primitive — references the existing arms/<name>/ tree.

Mirrors the pre-asset-spec arms/<arm>/asset.yaml content (DOF, joints,
controllers.yaml, mount frame). Generators pick this up when composing
``x500_with_<arm>`` UAVs.
"""

from __future__ import annotations

from typing import List, Literal, Optional

from pydantic import Field

from .common import _BaseModel


class ArmDescriptor(_BaseModel):
    version: int = 1
    kind: Literal["arm"]
    name: str
    description: Optional[str] = None
    dof: int
    mass_kg: float
    base_link: str
    ee_link: str
    joints: List[str]
    default_controller: str = "joint_trajectory_controller"
    moveit: bool = False
    # Path to controllers.yaml, relative to the package share dir.
    # The Gazebo generator emits a gz_ros2_control plugin pointing at it.
    controllers_yaml: Optional[str] = None
    # Path to URDF/Xacro for ros2_control hardware interface.
    urdf_xacro: Optional[str] = None
    # PX4 airframe ID assigned when composed onto a UAV via
    # "x500_with_<arm>"; used by the composer to avoid collisions.
    px4_airframe_id_default: Optional[int] = None
