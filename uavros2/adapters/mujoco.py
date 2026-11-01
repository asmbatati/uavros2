"""MuJoCo adapter — active.

Steps a MuJoCo physics model and produces MAVROS-shaped state topics.
Loads the MJCF for the requested UAV (and optionally arm) from
``arms/<arm>/mjcf/<arm>.xml`` and ``models/<uav>/mjcf/<uav>.xml``.

There is no PX4 here: a small position controller flies the model to
``setpoint_position/pose`` (holds the spawn point until the first one), unless
``actuators`` messages drive the rotors directly.
"""

import os

import numpy as np

from ..control_math import allocation_matrix, position_control
from .base import register, SimAdapter

# Real-time stepping catches up at most this many physics steps per call.
MAX_SUBSTEPS = 50


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
            share = get_package_share_directory("uavros2")
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
        self._mujoco.mj_forward(self._model, self._data)

        m = self._model
        self._base = self._mujoco.mj_name2id(m, self._mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self._mass = float(m.body_subtreemass[self._base])
        self._inertia = m.body_inertia[self._base].copy()
        self._hold = (self._data.qpos[:3].copy(), 0.0)
        sites = m.actuator_trnid[:, 0]
        self._alloc_inv = np.linalg.pinv(allocation_matrix(
            m.site_pos[sites][:, :2], m.actuator_gear[:, 2], m.actuator_gear[:, 5]))

    def _control(self, setpoint: dict) -> np.ndarray:
        """Rotor commands from the latest position setpoint."""
        msg = setpoint.get("position")
        if msg is not None:
            p, o = msg.pose.position, msg.pose.orientation
            yaw = np.arctan2(2.0 * (o.w * o.z + o.x * o.y),
                             1.0 - 2.0 * (o.y * o.y + o.z * o.z))
            self._hold = (np.array([p.x, p.y, p.z]), float(yaw))
        q, v = self._data.qpos, self._data.qvel
        rot = self._data.xmat[self._base].reshape(3, 3)
        thrust, torque = position_control(
            self._hold[0], self._hold[1], q[:3], v[:3], rot, v[3:6],
            self._mass, self._inertia, gravity=-self._model.opt.gravity[2])
        return np.clip(self._alloc_inv @ np.r_[thrust, torque], 0.0, 1.0)

    def step(self, dt: float, setpoint: dict) -> dict:
        if self._model is None:
            raise RuntimeError("MujocoAdapter.spawn() must be called before step()")
        acts = setpoint.get("actuators")
        direct = acts is not None and len(acts) == self._model.nu
        substeps = max(1, min(MAX_SUBSTEPS, round(dt / self._model.opt.timestep)))
        for _ in range(substeps):
            self._data.ctrl[:] = acts if direct else self._control(setpoint)
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
