"""Chassis primitives — multirotor, VTOL, fixed-wing.

Wide-scope v1: one discriminated union covers every airframe family.
Generators dispatch on ``type``.
"""

from __future__ import annotations

from typing import Annotated, Dict, List, Literal, Optional, Union

from pydantic import Field, model_validator

from .common import (
    Body, Inertia, Mass, MeshRef, Pose, Shape, Vector3, _BaseModel,
)


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------

class Rotor(_BaseModel):
    """A single propeller/motor. Pose is relative to the body frame."""
    name: str
    pose: List[float]                              # [x,y,z,r,p,y] radians
    spin: Literal["cw", "ccw"]
    max_thrust_N: float
    km_torque_coef: float = 0.05                   # k_motor (drag/thrust ratio)


class LandingFoot(_BaseModel):
    name: str
    pose: List[float]
    shape: Literal["sphere", "cylinder", "box"] = "sphere"
    radius: float = 0.012
    size: Optional[List[float]] = None


class LandingGear(_BaseModel):
    """Landing gear footprint. Type is informational; feet[] is what gets drawn."""
    type: Literal["skids", "tall_skids", "fixed_wheels", "retractable_wheels",
                  "skid_landing", "sphere_feet"] = "skids"
    height_m: Optional[float] = None              # only for tall_skids variants
    feet: List[LandingFoot] = Field(default_factory=list)


class MountPoint(_BaseModel):
    """A named frame on the chassis where sensors/arms attach."""
    name: str
    pose: List[float]                             # relative to body frame


class Wing(_BaseModel):
    """Lifting surface."""
    airfoil: str                                   # ref to assets/airfoils/<name>.yaml
    span_m: float
    chord_m: float
    pose: List[float] = Field(default_factory=lambda: [0, 0, 0, 0, 0, 0])
    cl_alpha: float = 5.0                          # lift slope, per rad
    cd0: float = 0.04                              # zero-lift drag
    incidence_deg: float = 2.0


class ControlSurface(_BaseModel):
    name: str
    hinge_pose: List[float]                        # [x,y,z,r,p,y]
    axis: Literal["x", "y", "z"]
    max_deflection_deg: float = 30.0
    gain: float = 1.0


class VTOLTransition(_BaseModel):
    trigger: Literal["airspeed", "manual", "time"] = "airspeed"
    forward_airspeed_mps: Optional[float] = None
    forward_pitch_deg: float = -10.0
    forward_duration_s: Optional[float] = None
    reverse_duration_s: Optional[float] = None


# ---------------------------------------------------------------------------
# Chassis variants — one Pydantic subclass per airframe family
# ---------------------------------------------------------------------------

MultirotorLayout = Literal[
    "quad_x", "quad_plus", "hex_x", "hex_plus", "oct_x", "y6", "coaxial_quad",
]


class _ChassisBase(_BaseModel):
    """Fields common to every chassis variant."""
    version: int = 1
    kind: Literal["chassis"]
    name: str
    description: Optional[str] = None
    body: Body
    mass: Mass
    landing_gear: LandingGear
    mount_points: List[MountPoint] = Field(default_factory=list)


class MultirotorChassis(_ChassisBase):
    type: Literal["multirotor"]
    layout: MultirotorLayout
    arm_length_m: float
    rotors: Dict[str, Rotor]                       # {"fl": Rotor, ...}

    @model_validator(mode="after")
    def _check_rotor_count(self):
        expected = {
            "quad_x": 4, "quad_plus": 4,
            "hex_x": 6, "hex_plus": 6,
            "oct_x": 8, "y6": 6, "coaxial_quad": 4,
        }[self.layout]
        if len(self.rotors) != expected:
            raise ValueError(
                f"layout={self.layout!r} expects {expected} rotors, "
                f"got {len(self.rotors)} ({list(self.rotors)})"
            )
        return self


class VTOLStandardChassis(_ChassisBase):
    type: Literal["vtol_standard"]
    lift_layout: MultirotorLayout
    lift_arm_length_m: float
    lift_rotors: Dict[str, Rotor]
    cruise_rotors: Dict[str, Rotor]
    wing: Wing
    control_surfaces: List[ControlSurface]
    transition: VTOLTransition = Field(default_factory=VTOLTransition)


class TailsitterChassis(_ChassisBase):
    type: Literal["tailsitter"]
    rotors: Dict[str, Rotor]
    wing: Wing
    control_surfaces: List[ControlSurface]
    transition: VTOLTransition = Field(default_factory=VTOLTransition)


class TiltrotorChassis(_ChassisBase):
    type: Literal["tiltrotor"]
    rotors: Dict[str, Rotor]                       # rotors include a tilt_axis hint
    wing: Wing
    control_surfaces: List[ControlSurface]
    transition: VTOLTransition = Field(default_factory=VTOLTransition)


class FixedWingChassis(_ChassisBase):
    type: Literal["fixed_wing"]
    cruise_rotor: Rotor                            # single tractor/pusher
    wing: Wing
    control_surfaces: List[ControlSurface]


ChassisDescriptor = Annotated[
    Union[
        MultirotorChassis,
        VTOLStandardChassis,
        TailsitterChassis,
        TiltrotorChassis,
        FixedWingChassis,
    ],
    Field(discriminator="type"),
]
