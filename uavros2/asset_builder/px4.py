"""PX4 airframe-file generator.

Emits ``config/px4/<id>_gz_<uav>`` from the UAV descriptor's chassis
rotor layout and PX4 binding.

For multirotor: rotors are emitted in PX4 convention order so the
ESC FUNC mapping matches stock PX4 conventions.
"""

from __future__ import annotations

import pathlib
from typing import List

from ..asset_spec import Catalog, MultirotorChassis, UAVDescriptor
from .gazebo import _TEMPLATES


# PX4 stock conventions for rotor index → physical position name.
# (Index, name) — order matters: ESC FUNC 1..N are assigned in this order.
_PX4_QUAD_X_ORDER = ["fl", "rr", "fr", "rl"]
_PX4_HEX_X_ORDER  = ["fl", "rr", "fr", "rl", "f",  "r"]
_PX4_OCT_X_ORDER  = ["fl", "rr", "fr", "rl", "fl2", "rr2", "fr2", "rl2"]
_PX4_ROTOR_ORDER = {
    "quad_x": _PX4_QUAD_X_ORDER,
    "quad_plus": _PX4_QUAD_X_ORDER,
    "hex_x":  _PX4_HEX_X_ORDER,
    "hex_plus": _PX4_HEX_X_ORDER,
    "oct_x":  _PX4_OCT_X_ORDER,
    "y6": _PX4_HEX_X_ORDER,
    "coaxial_quad": _PX4_QUAD_X_ORDER,
}


def _ordered_rotors(chassis: MultirotorChassis):
    order = _PX4_ROTOR_ORDER.get(chassis.layout, sorted(chassis.rotors))
    out = []
    for k in order:
        if k not in chassis.rotors:
            # Fall back to insertion order for rotors the convention
            # doesn't name (lets uncommon layouts still build).
            continue
        r = chassis.rotors[k]
        # CCW rotors get +k_m (positive yaw torque), CW get -k_m.
        sign = +1 if r.spin == "ccw" else -1
        out.append({
            "name": r.name,
            "pose": r.pose,
            "spin": r.spin,
            "km_signed": sign * r.km_torque_coef,
        })
    # Append any rotors that weren't in the canonical order.
    for k, r in chassis.rotors.items():
        if k not in order:
            sign = +1 if r.spin == "ccw" else -1
            out.append({
                "name": r.name,
                "pose": r.pose,
                "spin": r.spin,
                "km_signed": sign * r.km_torque_coef,
            })
    return out


def build_px4(
    uav: UAVDescriptor,
    catalog: Catalog,
    out_dir: pathlib.Path,
) -> List[pathlib.Path]:
    """Write ``config/px4/<id>_gz_<uav>`` and return the path."""
    chassis = catalog.resolve_chassis(uav.chassis.ref)
    if not isinstance(chassis, MultirotorChassis):
        raise NotImplementedError(
            f"PX4 airframe generation for chassis type "
            f"{chassis.__class__.__name__} is not implemented in v1"
        )

    rotors = _ordered_rotors(chassis)
    rotor_count = len(rotors)

    short_name = f"Gazebo {uav.name}"
    px4_type = {
        "quad_x": "Quadrotor", "quad_plus": "Quadrotor",
        "hex_x": "Hexarotor",  "hex_plus": "Hexarotor",
        "oct_x": "Octorotor",  "y6": "Hexarotor coaxial",
        "coaxial_quad": "Quadrotor coaxial",
    }.get(chassis.layout, "Quadrotor")

    # Merge PX4 binding `extra_params` with `overlays.px4.params`.
    extra_params = dict(uav.px4.extra_params)
    extra_params.update(uav.overlays.px4.params)

    text = _TEMPLATES.get_template("px4/airframe.j2").render(
        uav=uav,
        rotors=rotors,
        rotor_count=rotor_count,
        hover_thrust=uav.px4.hover_thrust,
        short_name=short_name,
        px4_type=px4_type,
        extra_params=extra_params,
    )

    af_dir = out_dir / "config" / "px4"
    af_dir.mkdir(parents=True, exist_ok=True)
    af_path = af_dir / f"{uav.px4.airframe_id}_gz_{uav.name}"
    af_path.write_text(text)
    return [af_path]
