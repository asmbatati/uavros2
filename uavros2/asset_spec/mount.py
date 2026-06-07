"""Mount primitive — fixed, gimbal-1axis, gimbal-2axis, gimbal-3axis.

For v1 we model the most common case: fixed and gimbal. Articulated
mounts (serial chains) are part of the arm descriptor.
"""

from __future__ import annotations

from typing import List, Literal, Optional

from pydantic import Field

from .common import MeshRef, _BaseModel


class GimbalAxis(_BaseModel):
    name: str                                       # e.g. "yaw", "pitch", "roll"
    axis: Literal["x", "y", "z"]
    lower_deg: float
    upper_deg: float
    max_velocity_deg_s: float = 60.0
    damping: float = 0.5
    friction: float = 0.01


class MountDescriptor(_BaseModel):
    version: int = 1
    kind: Literal["mount"]
    name: str
    description: Optional[str] = None
    type: Literal["fixed", "gimbal_1axis", "gimbal_2axis", "gimbal_3axis"]
    axes: List[GimbalAxis] = Field(default_factory=list)
    # Pose of the payload attach point relative to the mount base.
    attach_pose: List[float] = Field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    mesh: Optional[MeshRef] = None
