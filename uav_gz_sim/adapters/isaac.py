"""Isaac Sim adapter — inert (scaffolded).

PegasusSimulator's PX4 backend would publish state and sensors via the
omni.isaac.ros2_bridge graph. Wiring is a follow-up; see docs/SIMULATORS.md.
"""

from .base import SimAdapter, register


@register("isaac")
class IsaacAdapter(SimAdapter):
    @property
    def is_inert(self) -> bool:
        return True

    def spawn(self, uav: str, arm: str, world: str) -> None:
        raise NotImplementedError(
            "Isaac backend is scaffolded only. See docs/SIMULATORS.md."
        )

    def shutdown(self) -> None:
        return
