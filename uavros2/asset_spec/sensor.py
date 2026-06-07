"""Sensor primitive — references an existing sensor SDF model directory.

A sensor descriptor is mostly *metadata*: where the SDF lives (so the
generator can emit a Gazebo `<include uri="model://...">`), what canonical
topic the sensor publishes, what default parameters it accepts.

For v1 we don't regenerate the sensor SDFs themselves — those stay
hand-authored under ``models/<sensor_name>/``. The asset descriptor just
composes them.
"""

from __future__ import annotations

from typing import Any, Dict, Literal, Optional

from pydantic import Field

from .common import MeshRef, _BaseModel


class GazeboSensorBinding(_BaseModel):
    """How the sensor appears in Gazebo."""
    # Existing model dir under models/. The generator emits
    # <include><uri>model://<gz_model></uri></include>
    gz_model: str
    # If the chassis include exposes a link/joint of this name, the
    # generator will emit a fixed joint binding the sensor link to it.
    # Many sensor model SDFs are self-contained and don't need this.
    body_link: Optional[str] = None


class ROSBinding(_BaseModel):
    """How the sensor surfaces in ROS 2.

    ``topic_template`` is a python format string with the available
    variables: ``{ns}`` (UAV namespace) and ``{sensor_name}`` (the
    per-instance name).
    """
    topic_template: str
    msg_type: str
    default_rate_hz: Optional[float] = None


class SensorDescriptor(_BaseModel):
    version: int = 1
    kind: Literal["sensor"]
    name: str
    description: Optional[str] = None
    # Free-form sensor parameters (resolution, FOV, range, etc.).
    # Generator passes these through to the SDF model if it supports
    # them; per-instance overrides happen in the UAV descriptor.
    parameters: Dict[str, Any] = Field(default_factory=dict)
    gazebo: GazeboSensorBinding
    ros: ROSBinding
    mesh: Optional[MeshRef] = None
