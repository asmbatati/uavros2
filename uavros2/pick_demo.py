#!/usr/bin/env python3
r"""Pick and place from hover: carry a part from one post to another (B11).

Flies ``x500_with_openmanip_x`` (the OMX with its gripper) in the ``pick`` world: a
50 g part (4 x 3 x 5 cm) on a post whose top is 1.0 m up, and a second post 1 m away.

1. take off to the height where the arm, pointing straight down, reaches the post tops;
2. open the gripper, turn the arm down and fly over the first post beside the part;
3. touch the post's top with the finger tips and correct the hover height by where it
   was: the estimate's height drifts by a few centimetres between take-off and here,
   more than the grasp allows;
4. move over the part, trim the drone so the gripper is over it, and lower it to just
   above the part; wait for a moment when it is within ``grasp_window`` of the part
   across the fingers, then lower it round the part until the tips touch the post,
   back up 2 cm and close, the arm cancelling the drone's error along and up the arm's
   plane meanwhile; if a pad landed on the part or the fingers closed on nothing, try
   again (``GRASP_TRIES`` in all); lift;
5. fly over the second post, trim, lower the part until it touches the post, open,
   raise and stow the arm;
6. fly back and land.

The drone flies on its own position estimate (run it with ``UAVROS2_RTK=1``), and the
arm corrects with the same estimate. The posts' places are known in the world frame,
as on a surveyed site, and the take-off point ties that frame to the estimate's;
nothing senses the part, and only touch finds the posts' height (the wrist's force,
or the finger tips stopping while the arm still lowers them). Gazebo's truth
(``/<ns>/truth``, ``/<ns>/part/truth``) is only logged, to measure how far off the
gripper was and where the part ended up. Writes ``<arm>_pick.json`` next to
``aerial_arm_demo``'s flight CSV.

    UAVROS2_RTK=1 WORLD=pick DEMO=pick_demo aerial_arm_experiment.sh openmanip_x 4 \\
        results/pick -p controller:=outer_ff
"""

import json
import math
import os
import statistics
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from geometry_msgs.msg import Wrench
import numpy as np
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

from .aerial_arm_demo import _quat_rotate, AerialArmDemo
from .aerial_arm_demo import main as demo_main
from .arm_aware_control import ArmModel
from .arm_kinematics import OMX_GRIPPER_CLOSED, omx_gripper_down_ik, OMX_GRIPPER_OPEN

STOW_POSE = [0.0, -1.1, 1.2, 0.9]
GEAR_REST_Z = 0.227    # base_link above the ground with the x500 standing on its gear
GRIP_DEPTH = 0.41      # end_effector_link (the finger tips) below base_link when gripping
TIPS_ABOVE_POST = 0.02  # gripping, the 4.4 cm pads hold the part's top 3 cm
PREGRASP_UP = 0.06     # before closing in: the tips 3 cm above the part's top
CARRY_UP = 0.08        # flying: the tips 10 cm above the post tops, the part's bottom 8 cm
MIN_DEPTH = 0.33       # the gripper's reach straight down (omx_gripper_down_ik), with
MAX_DEPTH = 0.48       # 1 cm off joint 2's limit at the top
SERVO_X = 0.05         # how far the arm may reach fore and aft to cancel the drone's error,
SERVO_Y = 0.035        # and sideways (turning joint 1; with reach_x > 0 only)
SERVO_TAU = 0.15       # s, smoothing of the arm's correction
TOUCH_DX = 0.055       # touch the post this far ahead of the part: pads on the post, off the part
TOUCH_N = 0.4          # contact: the wrist's force this far from before (median over 0.2 s);
TOUCH_STALL_M = 0.025  # or the tips this much short of how far the arm has lowered them
TOUCH_SPEED = 0.02     # m/s, lowering until contact
TOUCH_SETTLE_S = 0.5   # the arm's start jolts the wrist: no force test this long
TOUCH_LOW = 0.03       # touch the post this much lower than the grasp hover: more reach
GUARD_UP = 0.035       # grasping, lower quickly to this far above where the post should be,
GUARD_BELOW = 0.05     # then slowly until the tips stall, at most this far below it
ON_PART_M = 0.03       # tips stalling this far above the post: a pad landed on the part
GRASP_TRIES = 3
PART_SIZE = (0.04, 0.03, 0.05)  # x, y (across the fingers: 2.4 cm to spare each side), z
LIFTED_M = 0.02        # the part counts as lifted 2 cm off its post
ON_POST_M = 0.06       # and as placed within 6 cm of the second post's centre (top 16 cm)
HOLDING_Q = OMX_GRIPPER_CLOSED + 0.003  # fingers stopped short of closed (the part: -0.005)
FALL_M = 0.4
PATH_POINTS = 10


def part_tilt(rotation):
    """Angle between the part's z axis and the vertical, rad."""
    x, y = rotation.x, rotation.y
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))


def pick_metrics(rows, post_top, place_xy):
    """Grasp offset, whether the part was lifted and carried, and where it ended up."""
    rows = [r for r in rows if r.get("part_x") is not None]
    out = {"truth_logged": bool(rows)}
    if not rows:
        return out
    rest_z = post_top + PART_SIZE[2] / 2.0
    # The last attempt's close: its first row.
    close = [i for i, r in enumerate(rows) if r["phase"] == "close" and r.get("true_ee_x") is not None]
    while len(close) > 1 and close[-2] == close[-1] - 1:
        close.pop()
    if close:
        r = rows[close[-1]]
        out["grasp_offset_xy_m"] = [r["true_ee_x"] - r["part_x"], r["true_ee_y"] - r["part_y"]]
        out["grasp_offset_z_m"] = r["true_ee_z"] - (post_top + TIPS_ABOVE_POST)
    lift = [r for r in rows if r["phase"] == "lift"]
    out["lifted"] = bool(lift) and lift[-1]["part_z"] > rest_z + LIFTED_M
    carry = [r for r in rows if r["phase"] in ("to_place", "over_place")]
    out["carried"] = bool(carry) and min(r["part_z"] for r in carry) > rest_z + LIFTED_M
    last = rows[-1]
    out["final_part_xyz_m"] = [last["part_x"], last["part_y"], last["part_z"]]
    out["place_error_m"] = math.hypot(last["part_x"] - place_xy[0], last["part_y"] - place_xy[1])
    out["final_tilt_deg"] = math.degrees(last["part_tilt"])
    out["placed"] = (out["place_error_m"] < ON_POST_M and abs(last["part_z"] - rest_z) < 0.01
                     and out["final_tilt_deg"] < 10.0)
    return out


def arm_ik(x, depth, y=0.0):
    """OMX joints for the finger tips at (x, y) and ``depth`` below base_link, pointing down."""
    return omx_gripper_down_ik((x, y, -depth))


def _rotation(q):
    """Rotation matrix of a unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class PickDemo(AerialArmDemo):

    def __init__(self):
        super().__init__()
        # The posts and the take-off point in the world frame (worlds/pick.sdf).
        self.declare_parameter("part_xy", [1.5, 0.0])
        self.declare_parameter("place_xy", [1.5, 1.0])
        self.declare_parameter("post_top", 1.0)
        self.declare_parameter("spawn_xy", [0.0, 0.0])
        self.declare_parameter("fly_speed", 0.3)
        self.declare_parameter("arm_move_s", 6.0)
        self.declare_parameter("lower_s", 4.0)
        self.declare_parameter("trim_s", 3.0)
        # The last 6 cm down round the part, once the gripper has been within grasp_window
        # (m, estimated, across the fingers) of it for grasp_still_s; after
        # window_timeout_s it goes anyway.
        self.declare_parameter("grasp_window", 0.01)
        self.declare_parameter("grasp_still_s", 0.3)
        self.declare_parameter("window_timeout_s", 30.0)
        self.declare_parameter("grasp_lower_s", 1.5)
        # Grasp with the tips this far ahead of base_link (m). Joint 1 then also moves them
        # sideways (turning the fingers, 27 deg for 3 cm at 0.07 m), so the arm corrects
        # the drone's error across the fingers too and the drone needs no window for it.
        self.declare_parameter("reach_x", 0.0)
        ns = self.get_parameter("namespace").value.strip("/")
        self.joints = []
        self.truth = None
        self.truth_ee = None
        self.part = None
        self.gripper_q = None
        self.joint_q = None
        self.arm_model = None
        self.grip_result = None
        self.offset = None
        self.hover_z_ekf = None
        self.wrench = []
        self.touches = {}
        self.trim = {}
        # The finger tips' target (EKF frame) while the arm servos, and the smoothed
        # (x, depth) it commands.
        self.servo = None
        self.servo_cmd = None
        self.gripper = ActionClient(self, ParallelGripperCommand,
                                    f"/{ns}/gripper_controller/gripper_cmd")
        self.create_subscription(TFMessage, f"/{ns}/truth", self._on_truth, qos_profile_sensor_data)
        self.create_subscription(TFMessage, f"/{ns}/part/truth", self._on_part,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, f"/{ns}/joint_states", self._on_joints,
                                 qos_profile_sensor_data)
        self.create_subscription(Wrench, f"/{ns}/wrist/wrench", self._on_wrench,
                                 qos_profile_sensor_data)
        self.create_timer(0.02, self._servo_tick)

    # ---------------------------------------------------------------- inputs
    def _on_truth(self, msg):
        for tf in msg.transforms:
            if "/" not in tf.child_frame_id:  # the model itself, in the world
                self.truth = tf.transform
            elif tf.child_frame_id.endswith("/end_effector_link"):  # in the model frame
                self.truth_ee = tf.transform

    def _on_part(self, msg):
        if msg.transforms:
            self.part = msg.transforms[0].transform

    def _on_wrench(self, msg):
        self.wrench = (self.wrench + [(msg.force.x, msg.force.y, msg.force.z)])[-50:]

    def _on_joints(self, msg):
        self.joint_q = dict(zip(msg.name, msg.position))
        self.gripper_q = self.joint_q.get("gripper_left_joint")

    def _add_fields(self, row):
        row["gripper_q"] = self.gripper_q
        if self.wrench:
            row["fx"], row["fy"], row["fz"] = self.wrench[-1]
        if self.servo_cmd is not None and self.servo is not None:
            row["servo_x"], row["servo_depth"], row["servo_y"] = self.servo_cmd
        t, ee = self.truth, self.truth_ee
        if t is not None:
            o = t.rotation
            row["true_x"], row["true_y"], row["true_z"] = t.translation.x, t.translation.y, t.translation.z
            if ee is not None:
                rx, ry, rz = _quat_rotate((o.x, o.y, o.z, o.w),
                                          (ee.translation.x, ee.translation.y, ee.translation.z))
                row["true_ee_x"] = t.translation.x + rx
                row["true_ee_y"] = t.translation.y + ry
                row["true_ee_z"] = t.translation.z + rz
        p = self.part
        if p is not None:
            row["part_x"], row["part_y"], row["part_z"] = p.translation.x, p.translation.y, p.translation.z
            row["part_tilt"] = part_tilt(p.rotation)

    # ------------------------------------------------------------ the frames
    def _world_to_ekf(self, x, y, z):
        return (x + self.offset[0], y + self.offset[1], z + self.offset[2])

    def _hover_over(self, xy):
        """Setpoint that puts the gripper, pointing down, over ``xy`` (world)."""
        return self._world_to_ekf(xy[0], xy[1], 0.0)[:2] + (self.hover_z_ekf,)

    def _ee_in_map(self):
        """The finger tips' estimated place: the drone's estimate, its attitude, the joints.

        From the arm's URDF and /joint_states rather than TF, which on some starts
        never arrives.
        """
        if self.arm_model is None or self.joint_q is None or self.pose is None:
            return None
        p, o = self.pose.pose.position, self.pose.pose.orientation
        r = _quat_rotate((o.x, o.y, o.z, o.w), self.arm_model.position(self.joint_q, "end_effector_link"))
        return (p.x + r[0], p.y + r[1], p.z + r[2])

    def _in_body(self, point):
        """``point`` (EKF frame) in base_link, from the drone's estimated pose."""
        o = self.pose.pose.orientation
        return _rotation((o.x, o.y, o.z, o.w)).T @ (np.asarray(point) - self._control_position())

    # -------------------------------------------------------------- the arm
    def _grip(self, position, timeout=8.0):
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["gripper_left_joint"]
        goal.command.position = [float(position)]
        if not self.gripper.wait_for_server(timeout_sec=10.0):
            raise TimeoutError("gripper_controller action server not available")
        send = self.gripper.send_goal_async(goal)
        self._wait(send.done, 10.0, "gripper goal acceptance")
        handle = send.result()
        if not handle.accepted:
            raise RuntimeError("gripper goal rejected")
        res = handle.get_result_async()
        self._wait(res.done, timeout, "gripper result")
        r = res.result().result
        return {"stalled": r.stalled, "reached_goal": r.reached_goal,
                "position": r.state.position[0] if r.state.position else None}

    def _arm_path(self, start, end, move_s):
        """Move the finger tips, pointing down, from ``start`` to ``end`` = (x, depth[, y]).

        Smooth in time (cosine), through ``PATH_POINTS`` IK points with velocities, so
        the joint trajectory controller's splines keep the tips on the straight line.
        """
        times = [move_s * i / PATH_POINTS for i in range(PATH_POINTS + 1)]
        qs = []
        for t in times:
            s = (1.0 - math.cos(math.pi * t / move_s)) / 2.0
            a, b = (tuple(start) + (0.0,))[:3], (tuple(end) + (0.0,))[:3]
            qs.append(np.array(arm_ik(*(u + (v - u) * s for u, v in zip(a, b)))))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        dt = move_s / PATH_POINTS
        for i in range(1, PATH_POINTS + 1):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in qs[i]]
            vel = (qs[i + 1] - qs[i - 1]) / (2.0 * dt) if i < PATH_POINTS else np.zeros(len(qs[i]))
            pt.velocities = [float(v) for v in vel]
            sec = int(times[i])
            pt.time_from_start = Duration(sec=sec, nanosec=int((times[i] - sec) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.jtc.send_goal_async(goal)
        self._wait(send.done, 10.0, "trajectory goal acceptance")
        handle = send.result()
        if not handle.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = handle.get_result_async()
        self._wait(res.done, move_s + 10.0, "trajectory result")
        return res.result().result.error_code

    def _servo_tick(self):
        """While ``self.servo`` is set, keep the finger tips on it with the arm.

        The arm reaches along its plane (body x) and down; across the fingers (body y)
        only with ``reach_x`` > 0, by turning joint 1 and the fingers with it.
        """
        if self.servo is None or self.servo_cmd is None or self.pose is None:
            return
        target = self._in_body(self.servo)
        reach = float(self.get_parameter("reach_x").value)
        x = float(np.clip(target[0], reach - SERVO_X, reach + SERVO_X))
        depth = float(np.clip(-target[2], MIN_DEPTH, MAX_DEPTH))
        y = float(np.clip(target[1], -SERVO_Y, SERVO_Y)) if reach > 0.0 else 0.0
        a = min(1.0, 0.02 / SERVO_TAU)
        cmd = tuple(c + a * (v - c) for c, v in zip(self.servo_cmd, (x, depth, y)))
        try:
            q = arm_ik(*cmd)
        except ValueError:  # out of reach: keep the last reachable command
            return
        self.servo_cmd = cmd
        self._command_joints(q)

    def _force_change(self, tare, samples=20):
        """Median over the last ``samples`` of the wrist force's distance from ``tare``."""
        recent = self.wrench[-samples:]
        return statistics.median(math.dist(f, tare) for f in recent) if recent else 0.0

    def _tips_z(self, seconds):
        """The finger tips' mean estimated height (EKF frame) over ``seconds``."""
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            ee = self._ee_in_map()
            if ee is not None:
                samples.append(ee[2])
            time.sleep(0.02)
        return statistics.fmean(samples) if samples else None

    def _touch(self, depth_from, speed=TOUCH_SPEED):
        """Lower the finger tips from ``depth_from`` until they touch something.

        The drone gives way upwards, so a light touch lifts it more than it loads the
        wrist: contact is the tips (estimated, smoothed) stopping ``TOUCH_STALL_M`` short
        of how far the arm has lowered them for 0.2 s, or the wrist's force changing by
        ``TOUCH_N``. Returns the tips' estimated height (EKF frame) at contact, the depth
        commanded and what told, or None without contact by ``MAX_DEPTH``. The arm backs
        off 1 cm.
        """
        tare = tuple(statistics.fmean(f[i] for f in self.wrench[-50:]) for i in range(3))
        start = self._tips_z(1.0)
        if start is None:
            raise RuntimeError("no gripper position to touch with")
        tips = start
        depth = depth_from
        t0 = time.monotonic()
        stalled_since = None
        while depth < MAX_DEPTH:
            self._check_height()
            depth = min(MAX_DEPTH, depth + speed * 0.02)
            self._command_joints(arm_ik(0.0, depth))
            time.sleep(0.02)
            ee = self._ee_in_map()
            if ee is None:
                continue
            tips += (ee[2] - tips) * 0.1   # 0.2 s
            now = time.monotonic()
            stalled = (depth - depth_from) - (start - tips) > TOUCH_STALL_M
            stalled_since = (stalled_since or now) if stalled else None
            by = None
            if stalled_since is not None and now - stalled_since >= 0.2:
                by = "stall"
            elif now - t0 > TOUCH_SETTLE_S and self._force_change(tare) > TOUCH_N:
                by = "force"
            if by is not None:
                self._command_joints(arm_ik(0.0, max(depth_from, depth - 0.01)))
                time.sleep(0.5)
                return {"tips_z": tips, "depth": depth, "by": by}
        return None

    def _trim_over(self, target_xy, seconds):
        """Shift the setpoint by the gripper's mean offset from ``target_xy`` (EKF frame).

        The gripper's estimated place (``_ee_in_map``). The body hangs tilted by a degree or two, which puts the
        tips 0.41 m below it a centimetre or so off.
        """
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            ee = self._ee_in_map()
            if ee is not None:
                samples.append(ee[:2])
            time.sleep(0.02)
        if not samples:
            raise RuntimeError("no gripper position to trim on")
        dx = target_xy[0] - statistics.fmean(s[0] for s in samples)
        dy = target_xy[1] - statistics.fmean(s[1] for s in samples)
        self.setpoint = (self.setpoint[0] + dx, self.setpoint[1] + dy, self.setpoint[2])
        return dx, dy

    def _wait_for_window(self, target_y, window, still_s, timeout):
        """Wait until the gripper stays within ``window`` of the target across the fingers.

        The estimated place, along y (the drone holds yaw 0). Returns (seconds waited,
        whether it found the window before ``timeout``).
        """
        t0 = time.monotonic()
        inside_since = None
        while time.monotonic() - t0 < timeout:
            self._check_height()
            ee = self._ee_in_map()
            now = time.monotonic()
            if ee is not None and abs(ee[1] - target_y) < window:
                inside_since = inside_since or now
                if now - inside_since >= still_s:
                    return now - t0, True
            else:
                inside_since = None
            time.sleep(0.02)
        return timeout, False

    def _make_arm_model(self):
        from ament_index_python import get_package_share_directory
        import xacro

        path = os.path.join(get_package_share_directory("uavros2"), "arms", self.arm,
                            "urdf", f"{self.arm}.urdf.xacro")
        return ArmModel(xacro.process_file(path, mappings=self._urdf_mappings()).toxml())

    def _check_height(self):
        """Raise once the drone has been ``FALL_M`` below its hover for 0.3 s (one estimate can jump)."""
        now = time.monotonic()
        if self._control_position()[2] >= self.hover_z_ekf - FALL_M:
            self.low_since = None
        elif getattr(self, "low_since", None) is None:
            self.low_since = now
        elif now - self.low_since > 0.3:
            raise RuntimeError(f"drone fell more than {FALL_M} m in phase {self.phase}")

    def _hold(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self._check_height()
            time.sleep(0.1)

    # -------------------------------------------------------------- mission
    def _grasp(self, part, post_top_ekf, lower_s):
        """Lower the open fingers round the part until the tips touch the post, and close.

        At ``part`` (EKF xy), the arm holding the tips there against the drone's motion
        (``_servo_tick``): quickly to ``GUARD_UP`` above where the post should be
        (``post_top_ekf``), then at ``TOUCH_SPEED`` until the tips stall; there, back up
        to ``TIPS_ABOVE_POST`` and close. Returns (outcome, where the arm ended as
        (x, depth), the tips' height at contact): outcome "closed", "on_part" (the tips
        stalled on the part's top, not closed) or "no_contact" (closed at the planned
        height).
        """
        self.servo_cmd = (float(self.get_parameter("reach_x").value), GRIP_DEPTH - PREGRASP_UP, 0.0)
        z_from = post_top_ekf + TIPS_ABOVE_POST + PREGRASP_UP
        z_to = post_top_ekf + GUARD_UP
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < lower_s:
            s = (1.0 - math.cos(math.pi * t / lower_s)) / 2.0
            self.servo = (part[0], part[1], z_from + (z_to - z_from) * s)
            self._check_height()
            time.sleep(0.02)
        z = z_to
        tips = self._ee_in_map()[2]
        stalled_since, contact = None, None
        while z > post_top_ekf - GUARD_BELOW:
            self._check_height()
            z -= TOUCH_SPEED * 0.02
            self.servo = (part[0], part[1], z)
            time.sleep(0.02)
            tips += (self._ee_in_map()[2] - tips) * 0.1   # 0.2 s
            now = time.monotonic()
            stalled_since = (stalled_since or now) if tips - z > TOUCH_STALL_M else None
            if stalled_since is not None and now - stalled_since >= 0.2:
                contact = tips
                break
        if contact is not None and contact - post_top_ekf > ON_PART_M:
            self.servo = (part[0], part[1], post_top_ekf + TIPS_ABOVE_POST + PREGRASP_UP)
            time.sleep(1.0)
            end = self.servo_cmd
            self.servo = None
            return "on_part", end, contact
        grip_z = (contact if contact is not None else post_top_ekf) + TIPS_ABOVE_POST
        self.servo = (part[0], part[1], grip_z)
        time.sleep(0.5)
        self.phase = "close"
        self.grip_result.setdefault("close", []).append(self._grip(OMX_GRIPPER_CLOSED))
        self._hold(1.0)
        end = self.servo_cmd
        self.servo = None
        return ("closed" if contact is not None else "no_contact"), end, contact

    def run(self):
        part_xy = [float(v) for v in self.get_parameter("part_xy").value]
        place_xy = [float(v) for v in self.get_parameter("place_xy").value]
        post_top = float(self.get_parameter("post_top").value)
        spawn_xy = [float(v) for v in self.get_parameter("spawn_xy").value]
        speed = float(self.get_parameter("fly_speed").value)
        arm_move_s = float(self.get_parameter("arm_move_s").value)
        lower_s = float(self.get_parameter("lower_s").value)
        trim_s = float(self.get_parameter("trim_s").value)
        settle_s = float(self.get_parameter("hover_settle_s").value)

        self._wait(lambda: self.state is not None and self.state.connected, 60.0, "FCU")
        self._wait(lambda: self.pose is not None, 180.0, "local position")
        self._wait(lambda: self.ctrl is not None, 60.0, "controller_state")
        self._wait(lambda: self.gripper_q is not None, 60.0, "gripper joint state")
        self._wait(lambda: len(self.wrench) >= 50, 60.0, "wrist force sensor")
        self.joints = list(self.ctrl.joint_names)
        self.arm_model = self._make_arm_model()
        codes = {}

        # The estimate's frame against the world's: where the drone stands at take-off,
        # averaged (one sample is up to 3 cm off).
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            p = self.pose.pose.position
            samples.append((p.x, p.y, p.z))
            time.sleep(0.02)
        home = SimpleNamespace(**{a: statistics.fmean(v[i] for v in samples) for i, a in enumerate("xyz")})
        self.offset = (home.x - spawn_xy[0], home.y - spawn_xy[1], home.z - GEAR_REST_Z)
        self.hover_z_ekf = self._world_to_ekf(0.0, 0.0, post_top + TIPS_ABOVE_POST + GRIP_DEPTH)[2]
        carry = GRIP_DEPTH - CARRY_UP
        self.setpoint = (home.x, home.y, self.hover_z_ekf)
        time.sleep(2.0)
        self.phase = "takeoff"
        self._offboard_and_arm()
        self._wait(lambda: abs(self.pose.pose.position.z - self.hover_z_ekf) < 0.15, 120.0,
                   "hover altitude")
        self._start_outer_loop()
        self.phase = "hover"
        self._hold(settle_s)

        self.phase = "arm_down"
        self.grip_result = {"open": self._grip(OMX_GRIPPER_OPEN)}
        codes["arm_down"] = self._move_arm(self.joints, arm_ik(0.0, carry), arm_move_s)
        self._hold(2.0)

        # Find the post's top by touch, beside the part, and fly at the height it says.
        self.phase = "to_touch"
        touch_xy = (part_xy[0] + TOUCH_DX, part_xy[1])
        self._go(self._hover_over(touch_xy), speed)
        self.phase = "over_touch"
        self._hold(settle_s)
        self.trim["touch"] = self._trim_over(self._hover_over(touch_xy)[:2], trim_s)
        self._hold(settle_s / 2.0)
        self.phase = "touch"
        self.setpoint = self.setpoint[:2] + (self.hover_z_ekf - TOUCH_LOW,)
        self._hold(2.0)
        touch = self._touch(carry)
        self.setpoint = self.setpoint[:2] + (self.hover_z_ekf,)
        self.touches["post"] = touch
        if touch is None:
            raise RuntimeError(f"the gripper found no post within {MAX_DEPTH} m")
        post_top_ekf = touch["tips_z"]
        error = post_top_ekf - self._world_to_ekf(0.0, 0.0, post_top)[2]
        self.touches["height_error_m"] = error
        self.hover_z_ekf += error
        self.get_logger().info(f"touched the post ({touch['by']}) {error * 100:+.1f} cm "
                               "from where planned")
        self.phase = "untouch"
        codes["untouch"] = self._arm_path((0.0, max(carry, touch["depth"] - 0.01)), (0.0, carry),
                                          lower_s / 2.0)

        # Pick.
        self.phase = "to_part"
        self._go(self._hover_over(part_xy), speed)
        self.phase = "over_part"
        self._hold(settle_s)
        part_ekf = self._hover_over(part_xy)[:2]
        self.trim["part"] = self._trim_over(part_ekf, trim_s)
        self._hold(settle_s / 2.0)
        self.phase = "pregrasp"
        reach = float(self.get_parameter("reach_x").value)
        pregrasp = (reach, GRIP_DEPTH - PREGRASP_UP)
        self.setpoint = (self.setpoint[0] - reach, self.setpoint[1], self.setpoint[2])
        codes["pregrasp"] = self._arm_path((0.0, carry), pregrasp, lower_s / 2.0)
        self._hold(2.0)
        # Up to GRASP_TRIES: wait for the drone to be over the part across the fingers,
        # lower until the tips touch the post, close; again if a pad landed on the part
        # or the fingers closed on nothing.
        self.attempts = []
        for _ in range(GRASP_TRIES):
            self.phase = "window"
            window = None if reach > 0.0 else self._wait_for_window(
                part_ekf[1], float(self.get_parameter("grasp_window").value),
                float(self.get_parameter("grasp_still_s").value),
                float(self.get_parameter("window_timeout_s").value))
            self.phase = "lower"
            outcome, end, contact = self._grasp(part_ekf, post_top_ekf,
                                                float(self.get_parameter("grasp_lower_s").value))
            holding = outcome != "on_part" and self.gripper_q is not None and self.gripper_q > HOLDING_Q
            self.attempts.append({"window": window, "outcome": outcome, "holding": holding,
                                  "contact_above_post_m": None if contact is None else contact - post_top_ekf})
            self.get_logger().info(f"grasp attempt: {self.attempts[-1]}")
            if holding:
                break
            if outcome != "on_part":
                self.phase = "reopen"
                self._grip(OMX_GRIPPER_OPEN)
            self.phase = "retry"
            self._arm_path(end, pregrasp, lower_s / 2.0)
            self._hold(1.0)
        self.phase = "lift"
        codes["lift"] = self._arm_path(end, (0.0, carry), lower_s)
        self._hold(2.0)
        holding = self.gripper_q is not None and self.gripper_q > HOLDING_Q
        self.grip_result["holding_after_lift"] = holding
        self.get_logger().info(f"grasp: {self.grip_result}, holding {holding}")

        # Place: lower the part until it touches the second post, and let go.
        self.phase = "to_place"
        self._go(self._hover_over(place_xy), speed)
        self.phase = "over_place"
        self._hold(settle_s)
        self.trim["place"] = self._trim_over(self._hover_over(place_xy)[:2], trim_s)
        self._hold(settle_s / 2.0)
        self.phase = "lower_place"
        place = self._touch(carry, TOUCH_SPEED / 2.0)
        self.touches["place"] = place
        self.phase = "release"
        self.grip_result["release"] = self._grip(OMX_GRIPPER_OPEN)
        self._hold(1.0)
        self.phase = "raise"
        depth = max(carry, place["depth"] - 0.01) if place else MAX_DEPTH
        codes["raise"] = self._arm_path((0.0, depth), (0.0, carry), lower_s)
        self.phase = "arm_stow"
        self._grip(0.0)
        codes["arm_stow"] = self._move_arm(self.joints, STOW_POSE, arm_move_s)
        self._hold(2.0)

        self.phase = "return"
        self._go((home.x, home.y, self.hover_z_ekf), 0.5)
        self._hold(3.0)
        self.phase = "land"
        self._land()
        self.phase = "done"
        return self.joints, codes

    def write_results(self, joints, codes, error=None):
        summary = super().write_results(joints, codes, error)
        with self.lock:
            rows = list(self.rows)
        result = pick_metrics(rows, float(self.get_parameter("post_top").value),
                              [float(v) for v in self.get_parameter("place_xy").value])
        result.update({
            "controller": self.controller,
            "grip": self.grip_result,
            "trim_m": self.trim,
            "attempts": getattr(self, "attempts", None),
            "touch": self.touches,
            "takeoff_offset_m": self.offset,
            "error": error,
        })
        with open(os.path.join(self.output_dir, f"{self.arm}_pick.json"), "w") as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f"pick: {result}")
        return summary


def main(args=None):
    return demo_main(args, node_class=PickDemo)


if __name__ == "__main__":
    raise SystemExit(main())
