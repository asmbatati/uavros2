"""UAV assembly descriptor — the file users mostly touch.

Refers to a chassis (by name) + a list of sensors (each referring to a
sensor primitive) + optional arm + PX4 binding + per-simulator overlays.
"""

from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional

from pydantic import Field, model_validator

from .common import _BaseModel


class UAVChassisRef(_BaseModel):
    ref: str                                    # name of assets/chassis/<name>.yaml
    overrides: Dict[str, Any] = Field(default_factory=dict)


class UAVSensorRef(_BaseModel):
    name: str                                   # per-instance name (canonical-topic prefix)
    ref: str                                    # assets/sensors/<ref>.yaml
    mount: str = "base_link"                    # one of chassis.mount_points[].name
    pose: List[float] = Field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    overrides: Dict[str, Any] = Field(default_factory=dict)


class UAVArmRef(_BaseModel):
    ref: str                                    # assets/arms/<ref>.yaml
    mount: str = "base_link"
    pose: List[float] = Field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    namespace: str = "drone"                    # for gz_ros2_control plugin


class PX4Binding(_BaseModel):
    airframe_id: int                            # must be unique across all UAVs
    spawn_pose_m: List[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.1, 0, 0, 0]
    )
    hover_thrust: float = 0.60
    # Extra `param set-default KEY VAL` lines added to the airframe script.
    extra_params: Dict[str, Any] = Field(default_factory=dict)


# ---- Per-simulator overlays (option (a) from the design discussion) -------

class GazeboOverlay(_BaseModel):
    # Map sensor name -> dict of param overrides applied at SDF emit time.
    sensor_plugins: Dict[str, Dict[str, Any]] = Field(default_factory=dict)
    # Free-form SDF strings spliced into the model body. Use sparingly.
    extra_xml: List[str] = Field(default_factory=list)
    # Escape hatch: when true, the Gazebo generator SKIPS model.sdf
    # emission for this UAV. The committed hand-authored
    # models/<name>/model.sdf is treated as the source of truth.
    # Used for assemblies whose body geometry isn't yet expressible
    # in the descriptor schema (composed arms with inline link chains,
    # legacy assets, etc.).
    import_existing_sdf: bool = False


class PX4Overlay(_BaseModel):
    # Extra `param set-default KEY VAL` lines (additive).
    params: Dict[str, Any] = Field(default_factory=dict)


class MuJoCoOverlay(_BaseModel):
    contact_solver: Optional[str] = None
    timestep_s: Optional[float] = None
    not_supported: bool = False


class WebotsOverlay(_BaseModel):
    not_supported: bool = False


class Overlays(_BaseModel):
    gazebo: GazeboOverlay = Field(default_factory=GazeboOverlay)
    px4: PX4Overlay = Field(default_factory=PX4Overlay)
    mujoco: MuJoCoOverlay = Field(default_factory=MuJoCoOverlay)
    webots: WebotsOverlay = Field(default_factory=WebotsOverlay)


# ---- The assembly itself --------------------------------------------------

class UAVDescriptor(_BaseModel):
    version: int = 1
    kind: Literal["uav"]
    name: str                                   # also the generated dir / model name
    description: Optional[str] = None
    chassis: UAVChassisRef
    sensors: List[UAVSensorRef] = Field(default_factory=list)
    arm: Optional[UAVArmRef] = None
    px4: PX4Binding
    overlays: Overlays = Field(default_factory=Overlays)

    @model_validator(mode="after")
    def _unique_sensor_names(self):
        seen = set()
        for s in self.sensors:
            if s.name in seen:
                raise ValueError(f"Duplicate sensor name {s.name!r} in UAV {self.name!r}")
            seen.add(s.name)
        return self
