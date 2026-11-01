#!/usr/bin/env python3
"""Contact experiment: push a tool against a wall from hover, position control only.

The baseline for the aerial arm's contact jobs (painting, cleaning, maintenance).
Flies ``x500_with_openmanip_x_tool`` in the ``wall`` world under PX4 OFFBOARD
position control:

1. take off, hover, move the arm to the forward pose (rod level, pointing +x);
2. fly to ``standoff`` from the wall and hold; the hold tares the force sensor;
3. creep towards the wall until the sensor sees contact;
4. move the position setpoint ``press_depth`` past the contact point and hold;
5. retract, stow the arm, fly back and land.

By default nothing regulates force: it measures what plain position control does
against a surface. ``force_target`` > 0 holds that force instead: with the outer
loop, directly in attitude and thrust (hybrid: force along the wall normal,
position along the wall); with PX4 position control, by moving the setpoint
(admittance). ``controller`` picks the flight controller as in
``aerial_arm_demo``; the feed-forward ones also cancel the measured contact
force's moment. Logs drone, tool tip and force columns to the
same CSV as ``aerial_arm_demo`` and writes ``<arm>_contact.json``.

    UAV=x500_with_openmanip_x_tool WORLD=wall DEMO=wall_contact_demo \\
        aerial_arm_experiment.sh openmanip_x 4 results/contact -p press_depth:=0.05
"""

import json
import math
import os
import statistics
import time

from geometry_msgs.msg import Wrench
import numpy as np
from rclpy.qos import qos_profile_sensor_data

from .aerial_arm_demo import AerialArmDemo, _quat_rotate, main as demo_main
from .arm_aware_control import WallForceControl
from .arm_kinematics import OMX_TOOL_CONTACT_POSE as CONTACT_POSE
from .arm_kinematics import OMX_TOOL_STOW_POSE as STOW_POSE
from .arm_kinematics import OMX_TOOL_TIP_IN_BASE as TIP_IN_BASE


TIP_RADIUS = 0.015
CONTACT_FORCE_N = 0.5
CONTACT_LOST_N = 0.2
FALL_M = 0.4
ADMITTANCE_M_PER_S_PER_N = 0.03
ADMITTANCE_MAX_M_PER_S = 0.10


def contact_torque(force):
    """Torque on the body, FLU, when the wall pushes the tip back: r x (-F, 0, 0)."""
    return np.array([0.0, -TIP_IN_BASE[2] * force, TIP_IN_BASE[1] * force])


def contact_metrics(rows, tare, press_settle_s=3.0):
    """Force and tool-position statistics of the ``press`` phase."""
    press = [r for r in rows if r["phase"] == "press"]
    if not press:
        return {"contact_reached": False}
    t0 = press[0]["t"]
    steady = [r for r in press if r["t"] - t0 >= press_settle_s] or press
    force = [abs(r["fx"] - tare) for r in steady if r.get("fx") is not None]
    tip_y = [r["tip_y"] for r in steady]
    tip_z = [r["tip_z"] for r in steady]
    half = len(force) // 2
    return {
        "contact_reached": True,
        "force_mean_n": statistics.fmean(force),
        "force_sd_n": statistics.pstdev(force),
        "force_min_n": min(force),
        "force_max_n": max(force),
        "force_first_half_n": statistics.fmean(force[:half]),
        "force_second_half_n": statistics.fmean(force[half:]),
        "contact_lost_fraction": sum(f < CONTACT_LOST_N for f in force) / len(force),
        "tip_y_sd_m": statistics.pstdev(tip_y),
        "tip_z_sd_m": statistics.pstdev(tip_z),
        "tip_y_range_m": max(tip_y) - min(tip_y),
        "tip_z_range_m": max(tip_z) - min(tip_z),
        "drone_x_error_mean_m": statistics.fmean(r["sp_x"] - r["x"] for r in steady),
        "n": len(force),
    }


class WallContactDemo(AerialArmDemo):

    def __init__(self):
        super().__init__()
        self.declare_parameter("wall_x", 3.0)
        self.declare_parameter("standoff", 0.20)
        self.declare_parameter("approach_speed", 0.05)
        self.declare_parameter("press_depth", 0.05)
        self.declare_parameter("press_s", 20.0)
        self.declare_parameter("arm_move_s", 5.0)
        self.declare_parameter("force_target", 0.0)
        ns = self.get_parameter("namespace").value.strip("/")
        self.wrench = None
        self.tare = 0.0
        self.contact_setpoint_x = None
        self.fell_after_s = None
        self.create_subscription(Wrench, f"/{ns}/tool/wrench", self._on_wrench,
                                 qos_profile_sensor_data)

    def _urdf_mappings(self):
        return {"sim_mode": "mock", "tool": "rod"}

    def _arm_mass(self):
        feed_forward = self._feed_forward()
        return feed_forward.model.mass if feed_forward is not None else 0.714

    def _feed_forward(self):
        if self.outer is not None:
            return self.outer.feed_forward
        return getattr(self, "arm_ff", None)

    def _on_wrench(self, msg):
        self.wrench = msg

    def _add_fields(self, row):
        o = self.pose.pose.orientation
        rx, ry, rz = _quat_rotate((o.x, o.y, o.z, o.w), TIP_IN_BASE)
        row["tip_x"], row["tip_y"], row["tip_z"] = row["x"] + rx, row["y"] + ry, row["z"] + rz
        w = self.wrench
        row["fx"], row["fy"], row["fz"] = (
            (w.force.x, w.force.y, w.force.z) if w is not None else (None, None, None))

    def _go(self, target, speed):
        """Move the position setpoint in a straight line at ``speed`` m/s."""
        start = self.setpoint
        dist = math.dist(start, target)
        steps = max(1, int(dist / speed / 0.05))
        for i in range(1, steps + 1):
            self.setpoint = tuple(a + (b - a) * i / steps for a, b in zip(start, target))
            time.sleep(0.05)

    def _recent_fx(self, seconds):
        with self.lock:
            rows = self.rows[-int(seconds * 50):]
        return [r["fx"] for r in rows if r.get("fx") is not None]

    def run(self):
        wall_x = float(self.get_parameter("wall_x").value)
        standoff = float(self.get_parameter("standoff").value)
        speed = float(self.get_parameter("approach_speed").value)
        depth = float(self.get_parameter("press_depth").value)
        press_s = float(self.get_parameter("press_s").value)
        arm_move_s = float(self.get_parameter("arm_move_s").value)
        settle_s = float(self.get_parameter("hover_settle_s").value)

        self._wait(lambda: self.state is not None and self.state.connected, 60.0, "FCU")
        self._wait(lambda: self.pose is not None, 180.0, "local position")
        self._wait(lambda: self.ctrl is not None, 60.0, "controller_state")
        self._wait(lambda: self.wrench is not None, 60.0, "tool force sensor")
        joints = list(self.ctrl.joint_names)

        home = self.pose.pose.position
        self.setpoint = (home.x, home.y, home.z + self.hover_z)
        time.sleep(2.0)
        self.phase = "takeoff"
        self._offboard_and_arm()
        self._wait(lambda: abs(self.pose.pose.position.z - self.setpoint[2]) < 0.15,
                   120.0, "hover altitude")
        self._start_outer_loop()
        self.phase = "hover"
        time.sleep(settle_s)

        codes = {}
        self.phase = "arm_out"
        codes["arm_out"] = self._move_arm(joints, CONTACT_POSE, arm_move_s)
        time.sleep(settle_s)

        self.phase = "to_wall"
        # The drone spawns at the world origin but its local origin is off by
        # up to ~0.2 m (EKF, GPS noise), so measure from where it took off.
        contact_x = home.x + wall_x - TIP_RADIUS - TIP_IN_BASE[0]
        y, z = self.setpoint[1], self.setpoint[2]
        self._go((contact_x - standoff, y, z), 0.3)
        self.phase = "free"
        time.sleep(8.0)
        self.tare = statistics.fmean(self._recent_fx(5.0))
        self.get_logger().info(f"free hold {standoff} m from the wall; tare fx {self.tare:.3f} N")

        self.phase = "approach"
        limit_x = contact_x + 0.35
        while self.setpoint[0] < limit_x:
            self.setpoint = (self.setpoint[0] + speed * 0.05, y, z)
            time.sleep(0.05)
            recent = self._recent_fx(0.2)
            if recent and abs(statistics.fmean(recent) - self.tare) > CONTACT_FORCE_N:
                break
        else:
            raise RuntimeError("no contact: setpoint passed the wall by 0.35 m")
        # The setpoint leads the drone while creeping, so press from where the
        # drone actually is.
        contact_drone_x = self.pose.pose.position.x
        self.contact_setpoint_x = self.setpoint[0]
        self.get_logger().info(f"contact: drone x {contact_drone_x:.3f} m, "
                               f"setpoint x {self.contact_setpoint_x:.3f} m")

        self.setpoint = (contact_drone_x, y, z)
        self.phase = "press_ramp"
        self._go((contact_drone_x + depth, y, z), 0.05)
        self.phase = "press"
        target = float(self.get_parameter("force_target").value)
        if self.outer is not None:
            self.outer.freeze[0] = True
        feed_forward = self._feed_forward()
        force_control = None
        if target > 0.0 and self.outer is not None:
            force_control = WallForceControl(
                float(self.get_parameter("body_mass").value) + self._arm_mass(), target)
            self.outer.force_control = force_control
        force = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < press_s:
            if self.pose.pose.position.z < z - FALL_M:
                self.fell_after_s = time.monotonic() - t0
                raise RuntimeError(f"drone fell {self.fell_after_s:.1f} s into the press")
            recent = self._recent_fx(0.1)
            if recent:
                force = abs(statistics.fmean(recent) - self.tare)
            if feed_forward is not None:
                feed_forward.external_torque = contact_torque(force)
            if force_control is not None:
                force_control.measured = force
            elif target > 0.0:
                rate = ADMITTANCE_M_PER_S_PER_N * (target - force)
                rate = max(-ADMITTANCE_MAX_M_PER_S, min(ADMITTANCE_MAX_M_PER_S, rate))
                x = min(self.setpoint[0] + rate * 0.02, contact_drone_x + 0.20)
                self.setpoint = (x, y, z)
            time.sleep(0.02)
        if feed_forward is not None:
            feed_forward.external_torque = np.zeros(3)
        if self.outer is not None:
            self.outer.force_control = None
            self.outer.integral[0] = 0.0
            self.setpoint = (self.pose.pose.position.x, y, z)
            self.outer.freeze[0] = False

        self.phase = "retract"
        self._go((contact_x - standoff, y, z), 0.15)
        time.sleep(3.0)
        self.phase = "arm_stow"
        codes["arm_stow"] = self._move_arm(joints, STOW_POSE, arm_move_s)
        time.sleep(3.0)
        self.phase = "return"
        self._go((home.x, home.y, z), 0.5)
        time.sleep(3.0)

        self.phase = "land"
        self._land()
        self.phase = "done"
        return joints, codes

    def write_results(self, joints, codes, error=None):
        summary = super().write_results(joints, codes, error)
        with self.lock:
            rows = list(self.rows)
        contact = contact_metrics(rows, self.tare)
        contact.update({
            "controller": self.controller,
            "force_target_n": float(self.get_parameter("force_target").value),
            "press_depth_m": float(self.get_parameter("press_depth").value),
            "tare_n": self.tare,
            "contact_setpoint_x_m": self.contact_setpoint_x,
            "fell_after_s": self.fell_after_s,
            "error": error,
        })
        with open(os.path.join(self.output_dir, f"{self.arm}_contact.json"), "w") as f:
            json.dump(contact, f, indent=2)
        self.get_logger().info(f"contact: {contact}")
        return summary


def main(args=None):
    return demo_main(args, node_class=WallContactDemo)


if __name__ == "__main__":
    raise SystemExit(main())
