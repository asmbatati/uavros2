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

from .common import Pose, Vector3, Inertia, Material, Shape, MeshRef
from .chassis import (
    ChassisDescriptor, MultirotorChassis, VTOLStandardChassis,
    TailsitterChassis, TiltrotorChassis, FixedWingChassis,
    Rotor, LandingGear, ControlSurface, Wing, MountPoint,
)
from .sensor import SensorDescriptor
from .mount import MountDescriptor
from .arm import ArmDescriptor
from .airfoil import AirfoilDescriptor
from .uav import UAVDescriptor, UAVChassisRef, UAVSensorRef, UAVArmRef, PX4Binding, Overlays
from .loader import Catalog, load_any, load_yaml_file
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
