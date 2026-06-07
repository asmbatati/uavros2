"""Cross-catalog validation.

Pydantic catches per-descriptor errors at load time. This module catches
*relational* errors: airframe-ID collisions, dangling refs, sensor mount
points that don't exist on the chassis, etc.
"""

from __future__ import annotations

from collections import defaultdict
from typing import List, Tuple

from .chassis import MultirotorChassis
from .loader import Catalog


def validate_catalog(catalog: Catalog) -> List[str]:
    """Return a list of human-readable problems. Empty list = catalog OK."""
    problems: List[str] = []

    # 1. Airframe ID uniqueness across UAVs.
    seen: dict[int, str] = {}
    for uav in catalog.uavs.values():
        prior = seen.get(uav.px4.airframe_id)
        if prior is not None:
            problems.append(
                f"airframe_id collision: {uav.name!r} and {prior!r} both "
                f"claim PX4 airframe ID {uav.px4.airframe_id}"
            )
        else:
            seen[uav.px4.airframe_id] = uav.name

    # 2. Sensor names unique within each UAV (already checked by Pydantic
    #    model_validator, but double-check for the cross-catalog walk).
    for uav in catalog.uavs.values():
        names = [s.name for s in uav.sensors]
        if len(names) != len(set(names)):
            problems.append(
                f"UAV {uav.name!r}: duplicate sensor names {names!r}"
            )

    # 3. Every UAV's chassis ref resolves.
    for uav in catalog.uavs.values():
        if uav.chassis.ref not in catalog.chassis:
            problems.append(
                f"UAV {uav.name!r}: unknown chassis ref {uav.chassis.ref!r} "
                f"(have {sorted(catalog.chassis)})"
            )

    # 4. Every UAV's sensor refs resolve, and their mount frames exist on
    #    the chassis.
    for uav in catalog.uavs.values():
        chassis = catalog.chassis.get(uav.chassis.ref)
        chassis_frames = (
            {mp.name for mp in chassis.mount_points} if chassis else set()
        )
        for s in uav.sensors:
            if s.ref not in catalog.sensors:
                problems.append(
                    f"UAV {uav.name!r}: unknown sensor ref {s.ref!r} for "
                    f"instance {s.name!r}"
                )
            if chassis and s.mount not in chassis_frames:
                problems.append(
                    f"UAV {uav.name!r}: sensor {s.name!r} mounts on "
                    f"{s.mount!r}, but chassis {chassis.name!r} has no such "
                    f"mount_point (have {sorted(chassis_frames)})"
                )

    # 5. Arm refs resolve.
    for uav in catalog.uavs.values():
        if uav.arm is None:
            continue
        if uav.arm.ref not in catalog.arms:
            problems.append(
                f"UAV {uav.name!r}: unknown arm ref {uav.arm.ref!r} "
                f"(have {sorted(catalog.arms)})"
            )

    # 6. Multirotor rotor positions sanity-check (in CG-relative XY plane,
    #    rotor radius from origin shouldn't be wildly larger than arm_length).
    for chassis in catalog.chassis.values():
        if not isinstance(chassis, MultirotorChassis):
            continue
        for rname, rotor in chassis.rotors.items():
            x, y, *_ = rotor.pose
            r = (x * x + y * y) ** 0.5
            if r > 3 * chassis.arm_length_m:
                problems.append(
                    f"chassis {chassis.name!r}: rotor {rname!r} at r={r:.2f}m "
                    f"is >3x arm_length ({chassis.arm_length_m}m); typo?"
                )

    return problems
