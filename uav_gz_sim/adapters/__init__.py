"""Per-simulator adapter backends for sim_control_bridge.

Each adapter implements the :class:`base.SimAdapter` interface:
spawn(), step(), shutdown(), and topic_map(). The bridge picks the right
adapter based on the ``simulator`` parameter at launch time.
"""

from .base import SimAdapter, ADAPTER_REGISTRY

__all__ = ["SimAdapter", "ADAPTER_REGISTRY", "load"]


def load(name: str) -> SimAdapter:
    """Return an adapter instance by name, importing lazily."""
    if name not in ADAPTER_REGISTRY:
        # Import on demand so optional sim deps don't crash unrelated launches.
        if name == "gazebo":
            from . import gazebo  # noqa: F401
        elif name == "webots":
            from . import webots  # noqa: F401
        elif name == "mujoco":
            from . import mujoco  # noqa: F401
        elif name == "isaac":
            from . import isaac  # noqa: F401
        elif name == "pybullet":
            from . import pybullet  # noqa: F401
        elif name == "genesis":
            from . import genesis  # noqa: F401
        else:
            raise ValueError(f"Unknown simulator adapter: {name!r}")

    return ADAPTER_REGISTRY[name]()
