"""PyBullet adapter — active, placeholder only.

Loads a URDF via pybullet's native loader and steps the engine. The
controller behind it (in sim_control_bridge.py) is a placeholder PID
sufficient for headless CI smoke tests; not for real flight dynamics.
"""

import os
from typing import Optional

from .base import SimAdapter, register


@register("pybullet")
class PyBulletAdapter(SimAdapter):

    def __init__(self):
        self._p = None
        self._uid: Optional[int] = None

    @property
    def is_inert(self) -> bool:
        return False

    def _load_pybullet(self):
        if self._p is None:
            try:
                import pybullet as p
                import pybullet_data
            except ImportError as exc:
                raise RuntimeError(
                    "pybullet not installed. pip install pybullet."
                ) from exc
            p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            self._p = p

    def _resolve_urdf(self, uav: str, arm: str) -> str:
        try:
            from ament_index_python import get_package_share_directory
            share = get_package_share_directory("uavros2")
        except Exception:
            share = os.path.join(os.path.dirname(__file__), "..", "..")

        if arm and arm != "none":
            composed = os.path.join(share, "models", f"x500_with_{arm}", "urdf", f"x500_with_{arm}.urdf")
            if os.path.isfile(composed):
                return composed
        bare = os.path.join(share, "models", uav, "urdf", f"{uav}.urdf")
        if os.path.isfile(bare):
            return bare
        raise FileNotFoundError(
            f"No URDF asset found for uav={uav!r}, arm={arm!r}."
        )

    def spawn(self, uav: str, arm: str, world: str) -> None:
        self._load_pybullet()
        urdf = self._resolve_urdf(uav, arm)
        self._uid = self._p.loadURDF(urdf, [0, 0, 0.1])

    def step(self, dt: float, setpoint: dict) -> dict:
        if self._uid is None:
            raise RuntimeError("PyBulletAdapter.spawn() must be called before step()")
        self._p.stepSimulation()
        pos, ori = self._p.getBasePositionAndOrientation(self._uid)
        vlin, _ = self._p.getBaseVelocity(self._uid)
        return {
            "pose": (pos[0], pos[1], pos[2], ori[0], ori[1], ori[2], ori[3]),
            "vel": tuple(vlin),
            "imu": (0.0, 0.0, 9.81, 0.0, 0.0, 0.0),
        }

    def shutdown(self) -> None:
        if self._p is not None:
            try:
                self._p.disconnect()
            except Exception:
                pass
        self._p = None
        self._uid = None
