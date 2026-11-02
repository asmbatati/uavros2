#!/usr/bin/env python3
"""Check that a camera drone renders headless and still flies.

Hovers under OFFBOARD and counts the camera's frames. Starts after ``sim.launch.py``
(headless) and a ``ros_gz_bridge`` for the camera topics are up. Arms, takes off to
``--height``, holds the take-off point for ``--hold`` seconds,
then lands. Prints the frame rate and empty frames of each image topic during the hold,
the largest distance from the hold point, and any time PX4 left OFFBOARD or disarmed
(a failsafe). Exits 0 when every topic reaches ``--min-hz`` with no empty frame, the
drone stays within ``--max-error`` and PX4 never fails safe.

    camera_hover_check.py --ns drone /d435/image /d435/depth_image
"""

import argparse
import math
import sys
import threading
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

# Frames are 2.7-3.7 MB; decoding each one in Python costs more than the camera's
# rate, so count serialized messages and decode every CHECK_EVERY-th for content.
CHECK_EVERY = 10


class CameraHoverCheck:

    def __init__(self, node, ns, topics):
        self.node = node
        self.state = None
        self.pose = None
        self.setpoint = None
        self.holding = False
        self.frames = {t: [] for t in topics}
        self.empty = {t: 0 for t in topics}
        self.failsafes = []
        node.create_subscription(State, f"/{ns}/mavros/state", self._on_state, 10)
        node.create_subscription(PoseStamped, f"/{ns}/mavros/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        for topic in topics:
            node.create_subscription(Image, topic, lambda m, t=topic: self._on_image(t, m),
                                     qos_profile_sensor_data, raw=True)
        self.sp_pub = node.create_publisher(PoseStamped, f"/{ns}/mavros/setpoint_position/local", 10)
        self.arming = node.create_client(CommandBool, f"/{ns}/mavros/cmd/arming")
        self.set_mode = node.create_client(SetMode, f"/{ns}/mavros/set_mode")
        self.land = node.create_client(CommandTOL, f"/{ns}/mavros/cmd/land")
        node.create_timer(0.05, self._publish)

    def _on_state(self, msg):
        if self.holding and self.state is not None and (msg.mode != "OFFBOARD" or not msg.armed):
            self.failsafes.append(f"mode {msg.mode}, armed {msg.armed}")
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg

    def _on_image(self, topic, raw):
        if not self.holding:
            return
        self.frames[topic].append(time.monotonic())
        if len(self.frames[topic]) % CHECK_EVERY == 0:
            msg = deserialize_message(raw, Image)
            if not msg.data or not any(msg.data[::max(1, len(msg.data) // 4096)]):
                self.empty[topic] += 1

    def _publish(self):
        if self.setpoint is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.setpoint
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)

    def _wait(self, cond, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cond():
                return
            time.sleep(0.1)
        raise TimeoutError(f"timed out waiting for {what}")

    def _call(self, client, request):
        self._wait(client.service_is_ready, 30.0, client.srv_name)
        future = client.call_async(request)
        self._wait(future.done, 10.0, client.srv_name)
        return future.result()

    def run(self, height, hold_s):
        self._wait(lambda: self.state is not None and self.state.connected, 120.0, "FCU")
        self._wait(lambda: self.pose is not None, 180.0, "local position")
        home = self.pose.pose.position
        self.setpoint = (home.x, home.y, home.z + height)
        time.sleep(2.0)
        for _ in range(60):
            if self.state.mode != "OFFBOARD":
                self._call(self.set_mode, SetMode.Request(custom_mode="OFFBOARD"))
            if not self.state.armed:
                self._call(self.arming, CommandBool.Request(value=True))
            time.sleep(1.0)
            if self.state.mode == "OFFBOARD" and self.state.armed:
                break
        else:
            raise RuntimeError(f"could not enter OFFBOARD and arm ({self.state})")
        self._wait(lambda: abs(self.pose.pose.position.z - self.setpoint[2]) < 0.15, 60.0, "hover height")
        time.sleep(5.0)
        self.holding = True
        start = time.monotonic()
        worst = 0.0
        while time.monotonic() - start < hold_s:
            p = self.pose.pose.position
            worst = max(worst, math.dist((p.x, p.y, p.z), self.setpoint))
            time.sleep(0.05)
        self.holding = False
        self._call(self.land, CommandTOL.Request())
        return worst, hold_s


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("topics", nargs="+", help="ROS image topics to count")
    parser.add_argument("--ns", default="drone")
    parser.add_argument("--height", type=float, default=2.0)
    parser.add_argument("--hold", type=float, default=60.0, help="seconds")
    parser.add_argument("--min-hz", type=float, default=15.0)
    parser.add_argument("--max-error", type=float, default=0.5, help="m")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("camera_hover_check")
    check = CameraHoverCheck(node, args.ns, args.topics)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        worst, hold_s = check.run(args.height, args.hold)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    ok = worst <= args.max_error and not check.failsafes
    print(f"hold {hold_s:.0f} s: largest distance from the hold point {worst:.2f} m; "
          f"failsafes: {check.failsafes or 'none'}")
    for topic, stamps in check.frames.items():
        hz = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
        ok = ok and hz >= args.min_hz and check.empty[topic] == 0
        print(f"{topic}: {len(stamps)} frames, {hz:.1f} Hz, "
              f"{check.empty[topic]} empty of {len(stamps) // CHECK_EVERY} decoded")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
