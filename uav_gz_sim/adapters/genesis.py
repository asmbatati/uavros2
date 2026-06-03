"""Genesis adapter — stub.

Genesis API is still moving; revisit when stable.
"""

from .base import SimAdapter, register


@register("genesis")
class GenesisAdapter(SimAdapter):
    @property
    def is_inert(self) -> bool:
        return False

    def spawn(self, uav: str, arm: str, world: str) -> None:
        raise NotImplementedError(
            "Genesis backend is not yet implemented. See docs/SIMULATORS.md."
        )

    def shutdown(self) -> None:
        return
