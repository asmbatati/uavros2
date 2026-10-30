#!/usr/bin/env python3
"""Aerial-manipulation flight experiment: hover in OFFBOARD and move the arm.

Takes off through MAVROS (OFFBOARD position setpoints), hovers, drives the
arm through a list of joint poses with the joint_trajectory_controller
action, then lands. While flying it logs at 50 Hz:

* the drone position and its setpoint (``mavros/local_position/pose``),
* the arm's reference and measured joints (``controller_state``),
* the end-effector position in the map frame (drone pose composed with
  the ``base_link -> <ee_link>`` transform from robot_state_publisher).

Writes ``<output_dir>/<arm>_flight.csv`` and ``<arm>_summary.json`` with
per-phase hover error and joint tracking statistics.

Run it against a sim started with an arm, e.g.::

    ros2 launch uavros2 sim.launch.py uav:=x500_with_openmanip_x arm:=openmanip_x
    ros2 run uavros2 aerial_arm_demo --ros-args -p arm:=openmanip_x
"""

import csv
import json
import math
import os
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


# Joint poses per arm: (label, positions). three_dof, panda and ur5 are
# planar pitch-chain scaffolds hanging below base_link, so a positive first
# joint swings the arm (and its centre of mass) to one side of the hover
# point.
ARM_POSES = {
    "three_dof": [
        ("reach_a", [1.2, -0.6, 0.3]),
        ("reach_b", [-1.2, 0.6, -0.3]),
        ("fold", [0.4, 1.2, 0.8]),
        ("stow", [0.0, 0.0, 0.0]),
    ],
    # OpenManipulator-X (real kinematics: joint1 yaw, joints 2-4 pitch).
    # reach_a: forward reach, end effector ~0.31 m ahead of base_link;
    # reach_b: the same reach swung 90 deg to the side; stow is the pose the
    # arm spawns and lands in (above the landing gear).
    "openmanip_x": [
        ("reach_a", [0.0, 0.8, -0.3, 0.4]),
        ("reach_b", [1.57, 0.8, -0.3, 0.4]),
        ("fold", [0.0, -1.2, 1.4, 1.0]),
        ("stow", [0.0, -1.5, 1.4, 1.5]),
    ],
    "panda": [
        ("reach_a", [1.0, -0.3, 0.2, 0.2, 0.1, 0.1, 0.0]),
        ("reach_b", [-1.0, 0.3, -0.2, -0.2, -0.1, -0.1, 0.0]),
        ("fold", [0.3, 0.6, 0.6, 0.5, 0.4, 0.3, 0.2]),
        ("stow", [0.0] * 7),
    ],
    "ur5": [
        ("reach_a", [1.0, -0.3, 0.2, 0.2, 0.1, 0.0]),
        ("reach_b", [-1.0, 0.3, -0.2, -0.2, -0.1, 0.0]),
        ("fold", [0.3, 0.7, 0.6, 0.5, 0.4, 0.2]),
        ("stow", [0.0] * 6),
    ],
}

# arm:=none flies the same protocol on a plain x500 (no arm) as a baseline:
# same phases and durations, nothing moves.
ARM_POSES["none"] = [(label, None) for label, _ in ARM_POSES["openmanip_x"]]

EE_LINKS = {
    "three_dof": "ee_link",
    "openmanip_x": "end_effector_link",
    "panda": "panda_hand",
    "ur5": "tool0",
    "none": None,
}


def _quat_rotate(q, v):
    """Rotate vector v by unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v); v' = v + w * t + cross(q.xyz, t)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _stats(values):
    if not values:
        return {"rms": None, "max": None, "n": 0}
    return {
        "rms": math.sqrt(sum(v * v for v in values) / len(values)),
        "max": max(abs(v) for v in values),
        "n": len(values),
    }


def summarize(rows, settle_s=1.0):
    """Per-phase statistics from logged rows (dicts as written to the CSV).

    The first ``settle_s`` seconds of each arm phase are kept (that is
    where the disturbance is), but each phase reports both the whole-phase
    statistics and the steady part after the motion.
    """
    phases = {}
    for r in rows:
        phases.setdefault(r["phase"], []).append(r)
    out = {}
    for phase, rs in phases.items():
        t0 = rs[0]["t"]
        horiz = [math.hypot(r["x"] - r["sp_x"], r["y"] - r["sp_y"]) for r in rs]
        vert = [r["z"] - r["sp_z"] for r in rs]
        joint_err = [r["joint_err_norm"] for r in rs if r["joint_err_norm"] is not None]
        steady = [r for r in rs if r["t"] - t0 >= settle_s]
        out[phase] = {
            "duration_s": rs[-1]["t"] - t0,
            "horizontal_error_m": _stats(horiz),
            "vertical_error_m": _stats(vert),
            "joint_error_rad": _stats(joint_err),
            "steady_horizontal_error_m": _stats(
                [math.hypot(r["x"] - r["sp_x"], r["y"] - r["sp_y"]) for r in steady]),
        }
    return out


class AerialArmDemo(Node):

    def __init__(self):
        super().__init__("aerial_arm_demo")
        self.declare_parameter("namespace", "drone")
        self.declare_parameter("arm", "openmanip_x")
        self.declare_parameter("hover_z", 2.0)
        self.declare_parameter("hover_settle_s", 8.0)
        self.declare_parameter("move_s", 2.0)
        self.declare_parameter("hold_s", 5.0)
        self.declare_parameter("output_dir", os.path.expanduser("~/uavros2_results"))

        ns = self.get_parameter("namespace").value.strip("/")
        self.arm = self.get_parameter("arm").value
        if self.arm not in ARM_POSES:
            raise ValueError(f"no demo poses for arm {self.arm!r}; "
                             f"choose one of {sorted(ARM_POSES)}")
        self.hover_z = float(self.get_parameter("hover_z").value)
        self.output_dir = self.get_parameter("output_dir").value
        self.ee_link = EE_LINKS[self.arm]

        self.state = None
        self.pose = None
        self.ctrl = None
        self.setpoint = None
        self.phase = "preflight"
        self.rows = []
        self.t0 = time.monotonic()
        self.lock = threading.Lock()

        self.create_subscription(State, f"/{ns}/mavros/state", self._on_state, 10)
        self.create_subscription(PoseStamped, f"/{ns}/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(
            JointTrajectoryControllerState,
            f"/{ns}/joint_trajectory_controller/controller_state", self._on_ctrl, 10)
        self.sp_pub = self.create_publisher(
            PoseStamped, f"/{ns}/mavros/setpoint_position/local", 10)
        self.arming = self.create_client(CommandBool, f"/{ns}/mavros/cmd/arming")
        self.set_mode = self.create_client(SetMode, f"/{ns}/mavros/set_mode")
        self.jtc = ActionClient(
            self, FollowJointTrajectory,
            f"/{ns}/joint_trajectory_controller/follow_joint_trajectory")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.05, self._stream_setpoint)   # 20 Hz, PX4 needs > 2 Hz
        self.create_timer(0.02, self._log)               # 50 Hz

    # ---------------------------------------------------------------- inputs
    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg

    def _on_ctrl(self, msg):
        self.ctrl = msg

    def _stream_setpoint(self):
        if self.setpoint is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.setpoint
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)

    def _ee_in_map(self):
        if self.ee_link is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform("base_link", self.ee_link, Time())
        except Exception:  # noqa: BLE001 - TF not ready yet
            return None
        p, q = self.pose.pose.position, self.pose.pose.orientation
        t = tf.transform.translation
        rx, ry, rz = _quat_rotate((q.x, q.y, q.z, q.w), (t.x, t.y, t.z))
        return (p.x + rx, p.y + ry, p.z + rz)

    def _log(self):
        if self.pose is None or self.setpoint is None:
            return
        p = self.pose.pose.position
        row = {
            "t": time.monotonic() - self.t0, "phase": self.phase,
            "x": p.x, "y": p.y, "z": p.z,
            "sp_x": self.setpoint[0], "sp_y": self.setpoint[1], "sp_z": self.setpoint[2],
            "joint_err_norm": None, "ee_x": None, "ee_y": None, "ee_z": None,
        }
        ctrl = self.ctrl
        if ctrl is not None and ctrl.error.positions:
            row["joint_err_norm"] = math.sqrt(sum(e * e for e in ctrl.error.positions))
            for name, ref, fb in zip(ctrl.joint_names, ctrl.reference.positions,
                                     ctrl.feedback.positions):
                row[f"ref_{name}"] = ref
                row[f"q_{name}"] = fb
        ee = self._ee_in_map()
        if ee is not None:
            row["ee_x"], row["ee_y"], row["ee_z"] = ee
        with self.lock:
            self.rows.append(row)

    # -------------------------------------------------------------- mission
    def _wait(self, cond, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cond():
                return
            time.sleep(0.1)
        raise TimeoutError(f"timed out waiting for {what}")

    def _call(self, client, req):
        if not client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError(f"service {client.srv_name} not available")
        fut = client.call_async(req)
        self._wait(fut.done, 10.0, client.srv_name)
        return fut.result()

    def _offboard_and_arm(self):
        for _ in range(30):
            st = self.state
            if st.mode != "OFFBOARD":
                self._call(self.set_mode, SetMode.Request(custom_mode="OFFBOARD"))
            if not st.armed:
                self._call(self.arming, CommandBool.Request(value=True))
            time.sleep(1.0)
            if self.state.mode == "OFFBOARD" and self.state.armed:
                return
        raise RuntimeError(f"could not enter OFFBOARD + arm (state: {self.state})")

    def _move_arm(self, joints, positions, move_s):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        sec = int(move_s)
        pt.time_from_start = Duration(sec=sec, nanosec=int((move_s - sec) * 1e9))
        goal.trajectory.points = [pt]
        if not self.jtc.wait_for_server(timeout_sec=10.0):
            raise TimeoutError("joint_trajectory_controller action server not available")
        send = self.jtc.send_goal_async(goal)
        self._wait(send.done, 10.0, "trajectory goal acceptance")
        handle = send.result()
        if not handle.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = handle.get_result_async()
        self._wait(res.done, move_s + 10.0, "trajectory result")
        return res.result().result.error_code

    def run(self):
        move_s = float(self.get_parameter("move_s").value)
        hold_s = float(self.get_parameter("hold_s").value)
        settle_s = float(self.get_parameter("hover_settle_s").value)

        self.get_logger().info("waiting for MAVROS connection, pose and arm controller")
        self._wait(lambda: self.state is not None and self.state.connected, 60.0, "FCU")
        self._wait(lambda: self.pose is not None, 60.0, "local position")
        joints = []
        if self.arm != "none":
            self._wait(lambda: self.ctrl is not None, 60.0, "controller_state")
            joints = list(self.ctrl.joint_names)

        home = self.pose.pose.position
        self.setpoint = (home.x, home.y, home.z + self.hover_z)
        time.sleep(2.0)  # setpoints must already stream before OFFBOARD
        self.phase = "takeoff"
        self._offboard_and_arm()
        self.get_logger().info(f"OFFBOARD + armed; climbing to {self.hover_z} m")
        self._wait(lambda: abs(self.pose.pose.position.z - self.setpoint[2]) < 0.15,
                   60.0, "hover altitude")

        self.phase = "hover"
        time.sleep(settle_s)

        codes = {}
        for label, positions in ARM_POSES[self.arm]:
            if positions is None:  # baseline: hold the hover for the same time
                self.phase = f"arm_{label}"
                time.sleep(move_s + hold_s)
                continue
            if len(positions) != len(joints):
                raise ValueError(f"pose {label} has {len(positions)} values, "
                                 f"controller has {len(joints)} joints")
            self.phase = f"arm_{label}"
            self.get_logger().info(f"arm -> {label}")
            codes[label] = self._move_arm(joints, positions, move_s)
            time.sleep(hold_s)

        self.phase = "land"
        self._call(self.set_mode, SetMode.Request(custom_mode="AUTO.LAND"))
        self._wait(lambda: not self.state.armed, 90.0, "landing and disarm")
        self.phase = "done"
        return joints, codes

    def write_results(self, joints, codes, error=None):
        os.makedirs(self.output_dir, exist_ok=True)
        with self.lock:
            rows = list(self.rows)
        csv_path = os.path.join(self.output_dir, f"{self.arm}_flight.csv")
        fields = []
        for r in rows:
            fields += [k for k in r if k not in fields]
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
        flying = [r for r in rows if r["phase"] not in ("preflight", "takeoff", "land", "done")]
        summary = {
            "arm": self.arm,
            "joints": joints,
            "hover_z_m": self.hover_z,
            "trajectory_error_codes": codes,
            "error": error,
            "phases": summarize(flying),
        }
        with open(os.path.join(self.output_dir, f"{self.arm}_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(f"wrote {csv_path} ({len(rows)} rows)")
        return summary


def main(args=None):
    rclpy.init(args=args)
    node = AerialArmDemo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    joints, codes, error = [], {}, None
    try:
        joints, codes = node.run()
    except Exception as exc:  # noqa: BLE001 - always write what was logged
        error = f"{type(exc).__name__}: {exc}"
        node.get_logger().error(error)
    summary = node.write_results(joints, codes, error)
    for phase, s in summary["phases"].items():
        h = s["horizontal_error_m"]
        if h["n"]:
            node.get_logger().info(
                f"{phase:>14}: horiz rms {h['rms']:.3f} m max {h['max']:.3f} m, "
                f"vert rms {s['vertical_error_m']['rms']:.3f} m")
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
