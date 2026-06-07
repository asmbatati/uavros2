"""Common types shared by every descriptor.

Conventions
-----------
* All distances are metres, masses kg, forces Newtons (field-name suffix).
* Angles default to radians; suffix `_deg` accepts degrees and is
  converted at load time.
* Frame is REP-103 (FLU body / ENU world). Generators translate to
  Gazebo / PX4 FRD as needed.
"""

from __future__ import annotations

import math
from typing import List, Literal, Optional, Union

from pydantic import BaseModel, ConfigDict, Field, field_validator


class _BaseModel(BaseModel):
    """Forbid unknown fields so typos surface at load time."""
    model_config = ConfigDict(extra="forbid")


class Vector3(_BaseModel):
    """3D vector, metres."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __iter__(self):
        return iter(self.as_tuple())


class Pose(_BaseModel):
    """6-DOF pose. Either a flat 6-list or named fields.

    Accepts:
        - ``[x, y, z, roll, pitch, yaw]``  — flat list, radians
        - ``{xyz: [...], rpy: [...]}``     — split form, radians
        - ``{xyz: [...], rpy_deg: [...]}`` — split form, degrees

    Always stored internally as radians.
    """
    xyz: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])

    @classmethod
    def from_any(cls, raw) -> "Pose":
        if raw is None:
            return cls()
        if isinstance(raw, Pose):
            return raw
        if isinstance(raw, list) and len(raw) == 6:
            return cls(xyz=list(raw[:3]), rpy=list(raw[3:]))
        if isinstance(raw, dict):
            xyz = list(raw.get("xyz", [0.0, 0.0, 0.0]))
            if "rpy_deg" in raw:
                rpy = [math.radians(v) for v in raw["rpy_deg"]]
            else:
                rpy = list(raw.get("rpy", [0.0, 0.0, 0.0]))
            return cls(xyz=xyz, rpy=rpy)
        raise ValueError(f"Cannot parse pose from {raw!r}")

    def to_sdf_str(self, degrees: bool = False) -> str:
        """`<pose>` text body, space-separated."""
        x, y, z = self.xyz
        r, p, y_ = self.rpy
        if degrees:
            r, p, y_ = math.degrees(r), math.degrees(p), math.degrees(y_)
        return f"{x:g} {y:g} {z:g} {r:g} {p:g} {y_:g}"


class Inertia(_BaseModel):
    """Either explicit moment-of-inertia tensor or auto-compute hints.

    ``auto: true`` tells the generator to derive ixx/iyy/izz from the
    body geometry + density. Cross-terms default to 0.
    """
    auto: bool = False
    density_kg_per_m3: Optional[float] = None
    ixx: Optional[float] = None
    iyy: Optional[float] = None
    izz: Optional[float] = None
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0


class Shape(_BaseModel):
    """Primitive geometry. Used for visual/collision fallback when no mesh."""
    type: Literal["box", "cylinder", "sphere"]
    size: Optional[List[float]] = None         # box: [x, y, z]
    radius: Optional[float] = None             # cylinder / sphere
    length: Optional[float] = None             # cylinder

    @field_validator("size")
    @classmethod
    def _check_size(cls, v, info):
        if v is not None and len(v) != 3:
            raise ValueError("box.size must have 3 entries [x, y, z]")
        return v


class MeshRef(_BaseModel):
    """`package://` URI to a mesh file (.dae / .stl / .obj)."""
    uri: str
    scale: List[float] = Field(default_factory=lambda: [1.0, 1.0, 1.0])


class Material(_BaseModel):
    rgba: Optional[List[float]] = None         # [r, g, b, a]
    texture: Optional[str] = None


# ---------------------------------------------------------------------------
# Mass spec used by both chassis and arm primitives
# ---------------------------------------------------------------------------

class Mass(_BaseModel):
    total_kg: float
    inertia: Inertia = Field(default_factory=lambda: Inertia(auto=True))


# ---------------------------------------------------------------------------
# Body spec — either a mesh, a primitive shape, or both (mesh for visual,
# primitive for collision).
# ---------------------------------------------------------------------------

class Body(_BaseModel):
    mesh: Optional[MeshRef] = None
    collision: Optional[Shape] = None
    visual: Optional[Shape] = None
    material: Optional[Material] = None
