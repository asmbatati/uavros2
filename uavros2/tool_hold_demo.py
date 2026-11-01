#!/usr/bin/env python3
"""Hold the tool at one point in the air: the arm cancels the drone's error (B16).

Flies ``x500_with_openmanip_x_tool`` to a hover point, moves the arm to the contact
pose, and logs the tool for ``tool_hold_s`` with the arm still (``hold_off``), then for
``tool_hold_s`` with the arm correcting (``hold_on``). The correction uses the drone's
estimated pose, what a real system has; ``true_tip_*`` is Gazebo's truth, used
only to measure. Writes ``<arm>_hold.json``.

    UAV=x500_with_openmanip_x_tool WORLD=wall DEMO=tool_hold_demo
    aerial_arm_experiment.sh openmanip_x 4 results/hold -p controller:=px4_ff
"""

import json
import math
import os
import statistics
import time

import numpy as np

from .aerial_arm_demo import main as demo_main
from .arm_aware_control import tool_correction
from .arm_kinematics import OMX_TOOL_CONTACT_POSE as CONTACT_POSE
from .wall_contact_demo import WallContactDemo


# From the arm's URDF.
JOINT_LIMITS = {"joint1": (-3.14, 3.14), "joint2": (-1.5, 1.5),
                "joint3": (-1.5, 1.4), "joint4": (-1.7, 1.97)}


def hold_metrics(rows, phase, settle_s=3.0):
    """Tool scatter about its mean and about the target, truth when logged."""
    rows = [r for r in rows if r["phase"] == phase and r.get("true_tip_x") is not None]
    if not rows:
        return None
    t0 = rows[0]["t"]
    rows = [r for r in rows if r["t"] - t0 >= settle_s]
    xyz = [[r[f"true_tip_{a}"] for r in rows] for a in "xyz"]
    centre = [statistics.fmean(v) for v in xyz]
    dist = sorted(math.dist(p, centre) for p in zip(*xyz))
    return {
        "n": len(rows),
        "rms_m": math.sqrt(statistics.fmean(d * d for d in dist)),
        "p95_m": dist[int(0.95 * len(dist))],
        "max_m": dist[-1],
        "within_2cm_fraction": sum(d < 0.02 for d in dist) / len(dist),
        "sd_xyz_m": [statistics.pstdev(v) for v in xyz],
    }


class ToolHoldDemo(WallContactDemo):

    def __init__(self):
        super().__init__()
        self.declare_parameter("tool_hold_s", 30.0)
        self.declare_parameter("hold_x", 1.0)
        self.hold_target = None
        self.correcting = False
        self.model = None
        self.create_timer(0.02, self._correct_tick)

    def _correct_tick(self):
        if not self.correcting or self.hold_target is None:
            return
        o = self.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        # Target in the body frame: inverse rotation of (target - position).
        err = np.array(self.hold_target) - self._control_position()
        x, y, z, w = q
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        nominal = dict(zip(self.joints, CONTACT_POSE))
        dq = tool_correction(self.model, nominal, self.joints, "tool_tip", rot.T @ err)
        self._command_joints([float(np.clip(nominal[j] + d, *JOINT_LIMITS[j]))
                              for j, d in zip(self.joints, dq)])

    def run(self):
        hold_s = float(self.get_parameter("tool_hold_s").value)
        settle_s = float(self.get_parameter("hover_settle_s").value)
        self._wait(lambda: self.state is not None and self.state.connected, 60.0, "FCU")
        self._wait(lambda: self.pose is not None, 180.0, "local position")
        self._wait(lambda: self.ctrl is not None, 60.0, "controller_state")
        self.joints = list(self.ctrl.joint_names)
        self.model = self._make_feed_forward().model

        home = self.pose.pose.position
        z = home.z + self.hover_z
        self.setpoint = (home.x, home.y, z)
        time.sleep(2.0)
        self.phase = "takeoff"
        self._offboard_and_arm()
        self._wait(lambda: abs(self.pose.pose.position.z - z) < 0.15, 120.0, "hover altitude")
        self._start_outer_loop()
        self.phase = "hover"
        time.sleep(settle_s)

        codes = {"arm_out": self._move_arm(self.joints, CONTACT_POSE, 5.0)}
        self.phase = "to_point"
        self._go((home.x + float(self.get_parameter("hold_x").value), home.y, z), 0.3)
        time.sleep(settle_s)

        self.phase = "hold_off"
        time.sleep(hold_s)

        # The tool's mean place during the still hold, in the frame the loop flies on.
        tips = [r for r in self.rows[-200:] if r.get("true_tip_x") is not None]
        keys = "tip_{}" if self.truth_offset is None else "true_tip_{}"
        offset = self.truth_offset if self.truth_offset is not None else np.zeros(3)
        self.hold_target = tuple(
            statistics.fmean(r[keys.format(a)] for r in tips) + offset[i]
            for i, a in enumerate("xyz"))
        self.correcting = True
        self.phase = "hold_on"
        time.sleep(hold_s)
        self.correcting = False

        self.phase = "arm_stow"
        codes["arm_stow"] = self._move_arm(self.joints, [0.0, 0.9, 1.1, 1.3], 5.0)
        self.phase = "return"
        self._go((home.x, home.y, z), 0.5)
        time.sleep(2.0)
        self.phase = "land"
        self._land()
        self.phase = "done"
        return self.joints, codes

    def write_results(self, joints, codes, error=None):
        summary = super().write_results(joints, codes, error)
        with self.lock:
            rows = list(self.rows)
        result = {"controller": self.controller, "error": error,
                  "position_source": self.get_parameter("position_source").value,
                  "off": hold_metrics(rows, "hold_off"), "on": hold_metrics(rows, "hold_on")}
        with open(os.path.join(self.output_dir, f"{self.arm}_hold.json"), "w") as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f"hold: {result}")
        return summary


def main(args=None):
    return demo_main(args, node_class=ToolHoldDemo)


if __name__ == "__main__":
    raise SystemExit(main())
