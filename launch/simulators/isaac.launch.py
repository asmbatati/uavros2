#!/usr/bin/env python3
"""Isaac Sim backend for uavros2 — SCAFFOLDED.

Not wired end-to-end. Documents the install path and exits with guidance.
See docs/SIMULATORS.md for the PegasusSimulator-based PX4 integration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown


def _setup(_context, *_args, **_kwargs):
    return [
        LogInfo(msg=(
            "[isaac] Isaac Sim backend is scaffolded only — not wired in this pass.\n"
            "        To use Isaac:\n"
            "          1. Install Isaac Sim 4.x via NVIDIA Omniverse Launcher.\n"
            "          2. Clone PegasusSimulator: https://github.com/PegasusSimulator/PegasusSimulator\n"
            "          3. Pre-generate USD assets with scripts/convert/urdf_to_usd.py\n"
            "             (must be run inside Isaac's Python interpreter).\n"
            "          4. Wire this launch file: spawn UAV via PegasusSimulator's PX4 backend.\n"
            "        See docs/SIMULATORS.md for the full roadmap."
        )),
        Shutdown(reason="Isaac backend not yet implemented"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("uav", default_value="x500"),
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("namespace", default_value="drone"),
        DeclareLaunchArgument("arm", default_value="none"),
        OpaqueFunction(function=_setup),
    ])
