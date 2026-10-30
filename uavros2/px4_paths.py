"""Locate the PX4-Autopilot tree that belongs to the active workspace."""

import os

from ament_index_python import get_package_share_directory


def find_px4_dir():
    """Return the PX4-Autopilot sibling of the workspace holding ``ros2_ws``.

    Derived from the installed package share path
    (``<DEV_DIR>/ros2_ws/install/uavros2/share/uavros2``), so the launch
    always uses the PX4 tree next to the active ``ros2_ws``, not a stray
    copy named by ``$PX4_DIR``. Falls back to ``$DEV_DIR/PX4-Autopilot``
    (default ``~/drone_arm_ws``).
    """
    cur = get_package_share_directory("uavros2")
    for _ in range(8):
        cur = os.path.dirname(cur)
        if not cur or cur == "/":
            break
        candidate = os.path.join(cur, "PX4-Autopilot")
        if (os.path.isdir(candidate)
                and os.path.isdir(os.path.join(cur, "ros2_ws"))):
            return candidate
    dev_dir = os.getenv("DEV_DIR",
                        os.path.join(os.path.expanduser("~"), "drone_arm_ws"))
    return os.path.join(dev_dir, "PX4-Autopilot")
