"""Gazebo SDF + model.config generator.

Produces ``models/<uav>/model.sdf`` and ``models/<uav>/model.config`` from
a :class:`UAVDescriptor` resolved against a :class:`Catalog`.
"""

from __future__ import annotations

import math
import pathlib
from typing import Dict, List, Optional

import jinja2

from ..asset_spec import Catalog, UAVDescriptor


_TEMPLATES = jinja2.Environment(
    loader=jinja2.FileSystemLoader(
        str(pathlib.Path(__file__).parent / "templates")
    ),
    autoescape=False,
    keep_trailing_newline=True,
    trim_blocks=True,
    lstrip_blocks=True,
)


def _pose_to_sdf_str(pose: List[float], degrees: bool = True) -> str:
    """Render a 6-list pose as `<pose>` body text."""
    x, y, z, r, p, yw = pose
    if degrees:
        r, p, yw = math.degrees(r), math.degrees(p), math.degrees(yw)
    return f"{x:g} {y:g} {z:g} {r:g} {p:g} {yw:g}"


def build_gazebo(
    uav: UAVDescriptor,
    catalog: Catalog,
    out_dir: pathlib.Path,
    *,
    author_name: str = "Abdulrahman S. Al-Batati",
    author_email: str = "asmalbatati@hotmail.com",
) -> List[pathlib.Path]:
    """Write ``models/<uav.name>/{model.sdf,model.config}`` and return the paths.

    When ``uav.overlays.gazebo.import_existing_sdf`` is true, the SDF
    emission is skipped (the committed hand-authored model.sdf is
    treated as the source of truth) and only model.config is rewritten.
    """
    if uav.overlays.gazebo.import_existing_sdf:
        # Only regenerate model.config (cheap metadata) - leave model.sdf alone.
        chassis = catalog.resolve_chassis(uav.chassis.ref)
        cfg_text = _TEMPLATES.get_template("gazebo/model.config.j2").render(
            uav=uav,
            author_name=author_name,
            author_email=author_email,
            description=uav.description or chassis.description
                or f"Imported / hand-authored UAV {uav.name}",
        )
        model_dir = out_dir / "models" / uav.name
        model_dir.mkdir(parents=True, exist_ok=True)
        cfg_path = model_dir / "model.config"
        cfg_path.write_text(cfg_text)
        return [cfg_path]

    chassis = catalog.resolve_chassis(uav.chassis.ref)

    # Build the per-sensor render context.
    sensors_ctx = []
    for s in uav.sensors:
        sd = catalog.resolve_sensor(s.ref)
        sensors_ctx.append({
            "name":      s.name,
            "ref":       s.ref,
            "mount":     s.mount,
            "pose_sdf":  _pose_to_sdf_str(s.pose, degrees=True),
            "gz_model":  sd.gazebo.gz_model,
            "body_link": sd.gazebo.body_link,
        })

    arm_ctx = None
    if uav.arm is not None:
        arm_ctx = {
            "ref":      uav.arm.ref,
            "mount":    uav.arm.mount,
            "pose_sdf": _pose_to_sdf_str(uav.arm.pose, degrees=True),
        }

    extra_xml = list(uav.overlays.gazebo.extra_xml)

    sdf_text = _TEMPLATES.get_template("gazebo/model.sdf.j2").render(
        uav=uav,
        chassis=chassis,
        sensors=sensors_ctx,
        arm=arm_ctx,
        extra_xml=extra_xml,
    )
    cfg_text = _TEMPLATES.get_template("gazebo/model.config.j2").render(
        uav=uav,
        author_name=author_name,
        author_email=author_email,
        description=uav.description or chassis.description
            or f"Auto-generated UAV {uav.name}",
    )

    model_dir = out_dir / "models" / uav.name
    model_dir.mkdir(parents=True, exist_ok=True)
    sdf_path = model_dir / "model.sdf"
    cfg_path = model_dir / "model.config"
    sdf_path.write_text(sdf_text)
    cfg_path.write_text(cfg_text)
    return [sdf_path, cfg_path]
