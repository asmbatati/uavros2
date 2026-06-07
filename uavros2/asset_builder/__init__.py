"""Asset descriptor → simulator artefact generators.

Each generator takes a fully-resolved :class:`uavros2.asset_spec.UAVDescriptor`
plus its :class:`Catalog` context and writes one or more output files.
"""

from .gazebo import build_gazebo
from .px4 import build_px4

__all__ = ["build_gazebo", "build_px4"]
