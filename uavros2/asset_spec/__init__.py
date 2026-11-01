"""Asset descriptor schemas (Pydantic v2).

Each primitive is a top-level YAML file under ``assets/``:

- ``assets/chassis/<name>.yaml``  — :class:`ChassisDescriptor`
- ``assets/sensors/<name>.yaml``  — :class:`SensorDescriptor`
- ``assets/mounts/<name>.yaml``   — :class:`MountDescriptor`
- ``assets/arms/<name>.yaml``     — :class:`ArmDescriptor`
- ``assets/airfoils/<name>.yaml`` — :class:`AirfoilDescriptor`
- ``assets/uavs/<name>.yaml``     — :class:`UAVDescriptor` (the assembly)

Each YAML carries a top-level ``kind:`` discriminator. :func:`load_any`
dispatches on it.
"""

from .airfoil import AirfoilDescriptor
from .arm import ArmDescriptor
from .chassis import (
    ChassisDescriptor,
    ControlSurface,
    FixedWingChassis,
    LandingGear,
    MountPoint,
    MultirotorChassis,
    Rotor,
    TailsitterChassis,
    TiltrotorChassis,
    VTOLStandardChassis,
    Wing,
)
from .common import Inertia, Material, MeshRef, Pose, Shape, Vector3
from .loader import Catalog, load_any, load_yaml_file
from .mount import MountDescriptor
from .sensor import SensorDescriptor
from .uav import Overlays, PX4Binding, UAVArmRef, UAVChassisRef, UAVDescriptor, UAVSensorRef
from .validator import validate_catalog

__all__ = [
    "Pose", "Vector3", "Inertia", "Material", "Shape", "MeshRef",
    "ChassisDescriptor", "MultirotorChassis", "VTOLStandardChassis",
    "TailsitterChassis", "TiltrotorChassis", "FixedWingChassis",
    "Rotor", "LandingGear", "ControlSurface", "Wing", "MountPoint",
    "SensorDescriptor",
    "MountDescriptor",
    "ArmDescriptor",
    "AirfoilDescriptor",
    "UAVDescriptor", "UAVChassisRef", "UAVSensorRef", "UAVArmRef",
    "PX4Binding", "Overlays",
    "Catalog", "load_any", "load_yaml_file",
    "validate_catalog",
]
