"""Webots adapter — inert.

PX4 SITL (webots target) + webots_ros2_driver publish state and sensors.
"""

from .base import SimAdapter, register


@register("webots")
class WebotsAdapter(SimAdapter):
    @property
    def is_inert(self) -> bool:
        return True

    def spawn(self, uav: str, arm: str, world: str) -> None:
        return

    def shutdown(self) -> None:
        return
