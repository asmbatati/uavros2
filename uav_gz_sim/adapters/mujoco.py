"""MuJoCo adapter — active.

Steps a MuJoCo physics model and produces MAVROS-shaped state topics.
Loads the MJCF for the requested UAV (and optionally arm) from
``arms/<arm>/mjcf/<arm>.xml`` and ``models/<uav>/mjcf/<uav>.xml``.
"""

import os
from typing import Dict, Optional

from .base import SimAdapter, register


@register("mujoco")
class MujocoAdapter(SimAdapter):
    """Loads a MuJoCo model, steps it, exposes state."""

    def __init__(self):
        self._model = None
        self._data = None
        self._mujoco = None  # late-imported

    @property
    def is_inert(self) -> bool:
        return False

    def _load_mujoco(self):
        if self._mujoco is None:
            try:
                import mujoco
            except ImportError as exc:
                raise RuntimeError(
                    "mujoco package not installed. pip install mujoco."
                ) from exc
            self._mujoco = mujoco

    def _resolve_asset(self, uav: str, arm: str) -> str:
        """Return path to the MJCF file to load.

        Prefers a composed ``models/x500_with_<arm>/mjcf/...`` if arm != none,
        else the bare UAV MJCF.
        """
        try:
            from ament_index_python import get_package_share_directory
            share = get_package_share_directory("uav_gz_sim")
        except Exception:
            share = os.path.join(os.path.dirname(__file__), "..", "..")

        if arm and arm != "none":
            composed = os.path.join(share, "models", f"x500_with_{arm}", "mjcf", f"x500_with_{arm}.xml")
            if os.path.isfile(composed):
                return composed
        bare = os.path.join(share, "models", uav, "mjcf", f"{uav}.xml")
        if os.path.isfile(bare):
            return bare

        raise FileNotFoundError(
            f"No MJCF asset found for uav={uav!r}, arm={arm!r}. "
            "Generate one with scripts/convert/urdf_to_mjcf.py."
        )

    def spawn(self, uav: str, arm: str, world: str) -> None:
        self._load_mujoco()
        path = self._resolve_asset(uav, arm)
        self._model = self._mujoco.MjModel.from_xml_path(path)
        self._data = self._mujoco.MjData(self._model)

    def step(self, dt: float, setpoint: dict) -> dict:
        if self._model is None:
            raise RuntimeError("MujocoAdapter.spawn() must be called before step()")
        # Placeholder mixer: pass setpoint accelerations as ctrl signals if
        # the model has actuators; real rotor mixing + attitude PID belongs
        # in sim_control_bridge.py. setpoint["actuators"] is None until a
        # message arrives, so we have to guard explicitly.
        acts = setpoint.get("actuators")
        if acts is not None and len(acts) == self._model.nu:
            self._data.ctrl[:] = acts
        else:
            # Default to hover-ish thrust so the model doesn't just freefall
            # before the first setpoint arrives - tuned for the x500 MJCF
            # (1.5 kg, 4 rotors at gear=8 -> 0.46 gives mg = 14.7 N).
            self._data.ctrl[:] = 0.46
        self._mujoco.mj_step(self._model, self._data)

        q = self._data.qpos
        v = self._data.qvel
        # Floating base assumed: first 7 of qpos are (x,y,z,qw,qx,qy,qz)
        pose = (float(q[0]), float(q[1]), float(q[2]),
                float(q[4]), float(q[5]), float(q[6]), float(q[3]))
        vel = (float(v[0]), float(v[1]), float(v[2]))
        return {"pose": pose, "vel": vel, "imu": (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)}

    def shutdown(self) -> None:
        self._model = None
        self._data = None
