"""Gazebo adapter — inert.

PX4 SITL + MAVROS / uXRCE-DDS publishes state. ros_gz_bridge owns sensor
topic remapping (configured in launch/simulators/gazebo.launch.py). This
adapter exists so sim_control_bridge can be loaded uniformly across sims.
"""

from .base import SimAdapter, register


@register("gazebo")
class GazeboAdapter(SimAdapter):
    @property
    def is_inert(self) -> bool:
        return True

    def spawn(self, uav: str, arm: str, world: str) -> None:
        # No-op: gz_sim.launch.py + MAVROS owns spawning.
        return

    def shutdown(self) -> None:
        return
