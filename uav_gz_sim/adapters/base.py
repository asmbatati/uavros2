"""Base class and registry for sim_control_bridge adapters."""

from abc import ABC, abstractmethod
from typing import Dict


# Populated by each adapter module's import-time `register(...)` call.
ADAPTER_REGISTRY: Dict[str, type] = {}


def register(name: str):
    """Class decorator that registers a SimAdapter subclass under ``name``."""
    def _wrap(cls):
        ADAPTER_REGISTRY[name] = cls
        return cls
    return _wrap


class SimAdapter(ABC):
    """Pluggable physics-engine backend for sim_control_bridge.

    For simulators with native PX4 integration (gazebo, webots, isaac), the
    adapter is inert: ``step()`` does nothing and the bridge does not own
    state publication — MAVROS / the native bridge does. The adapter exists
    only to declare which canonical topics are owned externally.

    For simulators without PX4 (mujoco, pybullet, genesis), the adapter
    drives the physics step from setpoints, runs a rotor mixer + PID, and
    synthesizes MAVROS-shaped state topics.
    """

    @property
    def is_inert(self) -> bool:
        """If True, the bridge does not publish state — MAVROS does.

        Override in active backends (mujoco/pybullet/genesis) to return False.
        """
        return True

    @abstractmethod
    def spawn(self, uav: str, arm: str, world: str) -> None:
        """Load the UAV (+ arm) into the simulator. Called once at startup."""

    def step(self, dt: float, setpoint: dict) -> dict:
        """Advance physics by ``dt`` seconds using ``setpoint``.

        Returns a dict with synthesized state:
            {'pose': (x, y, z, qx, qy, qz, qw),
             'vel':  (vx, vy, vz),
             'imu':  (ax, ay, az, gx, gy, gz),
             'joint_states': {name: (pos, vel, eff)}}

        Inert backends return {} and the bridge skips publication.
        """
        return {}

    @abstractmethod
    def shutdown(self) -> None:
        """Clean up the simulator process / API handles."""

    def topic_map(self) -> Dict[str, str]:
        """Map of canonical topic name -> native simulator topic name.

        Returned dict is informational only (used by sensor_relay when no
        native bridge can be configured). Override per simulator.
        """
        return {}
