"""YAML loader and catalog index.

The loader maps a directory tree under ``assets/`` into a :class:`Catalog`
that can resolve ``ref:`` strings to descriptor instances.
"""

from __future__ import annotations

import os
import pathlib
from typing import Any, Dict, Optional, Union

import yaml
from pydantic import TypeAdapter

from .airfoil import AirfoilDescriptor
from .arm import ArmDescriptor
from .chassis import ChassisDescriptor
from .mount import MountDescriptor
from .sensor import SensorDescriptor
from .uav import UAVDescriptor


# Single TypeAdapter that dispatches on `kind:`.
_AnyDescriptor = Union[
    ChassisDescriptor,
    SensorDescriptor,
    MountDescriptor,
    ArmDescriptor,
    AirfoilDescriptor,
    UAVDescriptor,
]
_ADAPTERS = {
    "chassis": TypeAdapter(ChassisDescriptor),
    "sensor": TypeAdapter(SensorDescriptor),
    "mount": TypeAdapter(MountDescriptor),
    "arm": TypeAdapter(ArmDescriptor),
    "airfoil": TypeAdapter(AirfoilDescriptor),
    "uav": TypeAdapter(UAVDescriptor),
}


def load_yaml_file(path: Union[str, pathlib.Path]) -> Dict[str, Any]:
    """Read a YAML file into a plain dict."""
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: YAML root must be a mapping, got {type(data)}")
    return data


def load_any(path: Union[str, pathlib.Path]):
    """Load any descriptor by dispatching on its ``kind:`` field."""
    data = load_yaml_file(path)
    kind = data.get("kind")
    if kind not in _ADAPTERS:
        raise ValueError(
            f"{path}: 'kind' is required and must be one of {list(_ADAPTERS)}, "
            f"got {kind!r}"
        )
    try:
        return _ADAPTERS[kind].validate_python(data)
    except Exception as exc:
        raise ValueError(f"{path}: {exc}") from exc


class Catalog:
    """A loaded set of descriptors, indexed by (kind, name)."""

    SUBDIRS = ("chassis", "sensors", "mounts", "arms", "airfoils", "uavs")

    def __init__(self, root: Union[str, pathlib.Path]):
        self.root = pathlib.Path(root).resolve()
        self.chassis: Dict[str, ChassisDescriptor] = {}
        self.sensors: Dict[str, SensorDescriptor] = {}
        self.mounts: Dict[str, MountDescriptor] = {}
        self.arms: Dict[str, ArmDescriptor] = {}
        self.airfoils: Dict[str, AirfoilDescriptor] = {}
        self.uavs: Dict[str, UAVDescriptor] = {}
        self._buckets = {
            "chassis": self.chassis,
            "sensor":  self.sensors,
            "mount":   self.mounts,
            "arm":     self.arms,
            "airfoil": self.airfoils,
            "uav":     self.uavs,
        }
        if self.root.is_dir():
            self._load_all()

    # ------------------------------------------------------------------

    def _load_all(self) -> None:
        for sub in self.SUBDIRS:
            subdir = self.root / sub
            if not subdir.is_dir():
                continue
            for f in sorted(subdir.glob("*.yaml")):
                desc = load_any(f)
                self._buckets[desc.kind][desc.name] = desc

    # ------------------------------------------------------------------

    def resolve_chassis(self, ref: str) -> ChassisDescriptor:
        if ref not in self.chassis:
            raise KeyError(f"No chassis named {ref!r}; have {sorted(self.chassis)}")
        return self.chassis[ref]

    def resolve_sensor(self, ref: str) -> SensorDescriptor:
        if ref not in self.sensors:
            raise KeyError(f"No sensor named {ref!r}; have {sorted(self.sensors)}")
        return self.sensors[ref]

    def resolve_arm(self, ref: str) -> Optional[ArmDescriptor]:
        if ref not in self.arms:
            raise KeyError(f"No arm named {ref!r}; have {sorted(self.arms)}")
        return self.arms[ref]

    # ------------------------------------------------------------------

    def summary(self) -> Dict[str, list]:
        return {
            "chassis": sorted(self.chassis),
            "sensors": sorted(self.sensors),
            "mounts":  sorted(self.mounts),
            "arms":    sorted(self.arms),
            "airfoils": sorted(self.airfoils),
            "uavs":    sorted(self.uavs),
        }
