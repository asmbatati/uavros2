"""PX4 airframe-file generator.

Emits ``config/px4/<id>_gz_<uav>`` from the UAV descriptor's chassis
type and rotor / control-surface layout.

Dispatches on chassis ``type``:
  - multirotor       → templates/px4/airframe.j2          (CA_AIRFRAME=0)
  - vtol_standard    → templates/px4/vtol_standard_airframe.j2 (CA_AIRFRAME=2)
  - fixed_wing       → templates/px4/fixed_wing_airframe.j2    (CA_AIRFRAME=1)

For multirotor rotors are emitted in PX4 convention order so the
ESC FUNC mapping matches stock PX4 conventions.
"""

from __future__ import annotations

import pathlib
from typing import Any, Dict, List

from ..asset_spec import (
    Catalog, FixedWingChassis, MultirotorChassis, UAVDescriptor,
    VTOLStandardChassis,
)
from .gazebo import _TEMPLATES


# PX4 stock conventions for rotor index → physical position name.
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

# Map a control-surface name to PX4's CA_SV_CS<N>_TYPE enum + torque effect.
# Values mirror PX4's MAVLink / param documentation:
#   1 = aileron, 2 = elevator, 3 = rudder, 4 = flap, 5 = elevon, 6 = ruddervator
_CS_TYPE_MAP = {
    "aileron_left":  {"type_id": 1, "trq_roll": +1.0, "trq_pitch":  0.0, "trq_yaw":  0.0},
    "aileron_right": {"type_id": 1, "trq_roll": -1.0, "trq_pitch":  0.0, "trq_yaw":  0.0},
    "elevator":      {"type_id": 2, "trq_roll":  0.0, "trq_pitch": +1.0, "trq_yaw":  0.0},
    "rudder":        {"type_id": 3, "trq_roll":  0.0, "trq_pitch":  0.0, "trq_yaw": +1.0},
    "flap":          {"type_id": 4, "trq_roll":  0.0, "trq_pitch":  0.0, "trq_yaw":  0.0},
    "elevon_left":   {"type_id": 5, "trq_roll": +1.0, "trq_pitch": +1.0, "trq_yaw":  0.0},
    "elevon_right":  {"type_id": 5, "trq_roll": -1.0, "trq_pitch": +1.0, "trq_yaw":  0.0},
}


def _signed_km(spin: str, km: float) -> float:
    return (+1 if spin == "ccw" else -1) * km


def _ordered_multirotor_rotors(rotors: Dict[str, Any], layout: str) -> List[dict]:
    order = _PX4_ROTOR_ORDER.get(layout, sorted(rotors))
    out: List[dict] = []
    seen = set()
    for k in order:
        if k not in rotors:
            continue
        seen.add(k)
        r = rotors[k]
        out.append({
            "name": r.name, "pose": r.pose, "spin": r.spin,
            "km_signed": _signed_km(r.spin, r.km_torque_coef),
        })
    for k, r in rotors.items():
        if k in seen:
            continue
        out.append({
            "name": r.name, "pose": r.pose, "spin": r.spin,
            "km_signed": _signed_km(r.spin, r.km_torque_coef),
        })
    return out


def _resolve_control_surfaces(surfaces) -> List[dict]:
    """Translate user-named control surfaces to PX4 CA_SV_CS<N>_* entries."""
    out = []
    for cs in surfaces:
        meta = _CS_TYPE_MAP.get(cs.name, {
            "type_id": 0, "trq_roll": 0.0, "trq_pitch": 0.0, "trq_yaw": 0.0,
        })
        out.append({
            "name": cs.name,
            "type_id": meta["type_id"],
            "trq_roll": meta["trq_roll"],
            "trq_pitch": meta["trq_pitch"],
            "trq_yaw": meta["trq_yaw"],
            "max_deflection_deg": cs.max_deflection_deg,
        })
    return out


# ---------------------------------------------------------------------------
# Public entry
# ---------------------------------------------------------------------------

def build_px4(
    uav: UAVDescriptor,
    catalog: Catalog,
    out_dir: pathlib.Path,
) -> List[pathlib.Path]:
    """Write ``config/px4/<id>_gz_<uav>`` and return the path."""
    chassis = catalog.resolve_chassis(uav.chassis.ref)

    extra_params = dict(uav.px4.extra_params)
    extra_params.update(uav.overlays.px4.params)
    short_name = f"Gazebo {uav.name}"

    if isinstance(chassis, MultirotorChassis):
        rotors = _ordered_multirotor_rotors(chassis.rotors, chassis.layout)
        text = _TEMPLATES.get_template("px4/airframe.j2").render(
            uav=uav,
            rotors=rotors,
            rotor_count=len(rotors),
            hover_thrust=uav.px4.hover_thrust,
            short_name=short_name,
            px4_type={
                "quad_x": "Quadrotor", "quad_plus": "Quadrotor",
                "hex_x": "Hexarotor",  "hex_plus": "Hexarotor",
                "oct_x": "Octorotor",  "y6": "Hexarotor coaxial",
                "coaxial_quad": "Quadrotor coaxial",
            }.get(chassis.layout, "Quadrotor"),
            extra_params=extra_params,
        )

    elif isinstance(chassis, VTOLStandardChassis):
        lift = _ordered_multirotor_rotors(chassis.lift_rotors, chassis.lift_layout)
        cruise = [
            {"name": r.name, "pose": r.pose, "spin": r.spin,
             "km_signed": _signed_km(r.spin, r.km_torque_coef)}
            for r in chassis.cruise_rotors.values()
        ]
        all_rotors = lift + cruise
        cs = _resolve_control_surfaces(chassis.control_surfaces)
        text = _TEMPLATES.get_template("px4/vtol_standard_airframe.j2").render(
            uav=uav,
            rotors=all_rotors,
            rotor_count=len(all_rotors),
            control_surfaces=cs,
            control_surface_count=len(cs),
            transition=chassis.transition,
            hover_thrust=uav.px4.hover_thrust,
            short_name=short_name,
            extra_params=extra_params,
        )

    elif isinstance(chassis, FixedWingChassis):
        cruise_rotor = {
            "name": chassis.cruise_rotor.name,
            "pose": chassis.cruise_rotor.pose,
            "spin": chassis.cruise_rotor.spin,
            "km_signed": _signed_km(
                chassis.cruise_rotor.spin, chassis.cruise_rotor.km_torque_coef
            ),
        }
        cs = _resolve_control_surfaces(chassis.control_surfaces)
        text = _TEMPLATES.get_template("px4/fixed_wing_airframe.j2").render(
            uav=uav,
            cruise_rotor=cruise_rotor,
            control_surfaces=cs,
            control_surface_count=len(cs),
            short_name=short_name,
            extra_params=extra_params,
        )

    else:
        raise NotImplementedError(
            f"PX4 airframe generation for chassis type "
            f"{chassis.__class__.__name__} is not implemented in v1"
        )

    af_dir = out_dir / "config" / "px4"
    af_dir.mkdir(parents=True, exist_ok=True)
    af_path = af_dir / f"{uav.px4.airframe_id}_gz_{uav.name}"
    af_path.write_text(text)
    return [af_path]
