"""Reader for worlds/manifest.yaml.

Used by the launch files to resolve per-world settings: the SDF filename,
the spawn pose (passed to PX4 via ``PX4_GZ_MODEL_POSE``), and the
per-simulator availability flags.
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory


_DEFAULT_SPAWN: Tuple[float, ...] = (0.0, 0.0, 0.1, 0.0, 0.0, 0.0)


def _manifest_path() -> str:
    return os.path.join(
        get_package_share_directory("uavros2"), "worlds", "manifest.yaml")


def load_manifest() -> Dict[str, Any]:
    """Return the parsed worlds/manifest.yaml as a dict."""
    path = _manifest_path()
    if not os.path.isfile(path):
        return {"worlds": {}}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    if "worlds" not in data:
        return {"worlds": {}}
    return data


def world_entry(world: str) -> Dict[str, Any]:
    """Return the manifest entry for ``world``. Empty dict if unknown."""
    return load_manifest().get("worlds", {}).get(world, {}) or {}


def spawn_pose(world: str) -> Tuple[float, float, float, float, float, float]:
    """Return [x, y, z, roll, pitch, yaw] spawn pose for ``world``.

    Falls back to (0, 0, 0.1, 0, 0, 0) for unknown worlds, which is the
    safe default for the empty / warehouse case.
    """
    entry = world_entry(world)
    p = entry.get("spawn_pose") or list(_DEFAULT_SPAWN)
    if len(p) < 6:
        p = list(p) + [0.0] * (6 - len(p))
    return tuple(float(v) for v in p[:6])  # type: ignore[return-value]
