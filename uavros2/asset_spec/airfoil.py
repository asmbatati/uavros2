"""Airfoil primitive — aero coefficients for fixed-wing / VTOL chassis."""

from __future__ import annotations

from typing import Literal, Optional

from .common import _BaseModel


class AirfoilDescriptor(_BaseModel):
    version: int = 1
    kind: Literal["airfoil"]
    name: str
    description: Optional[str] = None
    cl_alpha: float = 5.0          # lift slope per rad
    cl_0: float = 0.0              # lift at alpha = 0
    cd_0: float = 0.04             # zero-lift drag
    cd_alpha: float = 0.0          # drag slope per rad
    alpha_stall_deg: float = 15.0
    cm_0: float = 0.0              # zero-lift moment
